#include "galaxy_camera.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace galaxy_camera
{
namespace
{
std::string toLower(const std::string& input)
{
  std::string output = input;
  std::transform(output.begin(), output.end(), output.begin(), ::tolower);
  return output;
}

int64_t lineSelectorFromIndex(int line)
{
  switch (line)
  {
    case 0:
      return GX_ENUM_LINE_SELECTOR_LINE0;
    case 1:
      return GX_ENUM_LINE_SELECTOR_LINE1;
    case 2:
      return GX_ENUM_LINE_SELECTOR_LINE2;
    case 3:
    default:
      return GX_ENUM_LINE_SELECTOR_LINE3;
  }
}

GX_TRIGGER_SOURCE_ENTRY triggerSourceFromLine(int line)
{
  switch (line)
  {
    case 0:
      return GX_TRIGGER_SOURCE_LINE0;
    case 1:
      return GX_TRIGGER_SOURCE_LINE1;
    case 2:
      return GX_TRIGGER_SOURCE_LINE2;
    case 3:
    default:
      return GX_TRIGGER_SOURCE_LINE3;
  }
}

DX_PIXEL_COLOR_FILTER toDxColorFilter(GX_PIXEL_COLOR_FILTER_ENTRY filter)
{
  switch (filter)
  {
    case GX_COLOR_FILTER_BAYER_RG:
      return DX_PIXEL_COLOR_FILTER::BAYERRG;
    case GX_COLOR_FILTER_BAYER_GB:
      return DX_PIXEL_COLOR_FILTER::BAYERGB;
    case GX_COLOR_FILTER_BAYER_GR:
      return DX_PIXEL_COLOR_FILTER::BAYERGR;
    case GX_COLOR_FILTER_BAYER_BG:
      return DX_PIXEL_COLOR_FILTER::BAYERBG;
    case GX_COLOR_FILTER_NONE:
    default:
      return DX_PIXEL_COLOR_FILTER::NONE;
  }
}
}  // namespace

GalaxyCameraLowLevel::GalaxyCameraLowLevel() = default;

GalaxyCameraLowLevel::~GalaxyCameraLowLevel()
{
  stop();
  close();
}

void GalaxyCameraLowLevel::open(const CameraConfig& config)
{
  config_ = config;
  checkStatus(GXInitLib(), "GXInitLib");

  uint32_t device_num = 0;
  checkStatus(GXUpdateDeviceList(&device_num, 1000), "GXUpdateDeviceList");
  if (device_num == 0)
  {
    throw std::runtime_error("No Daheng camera detected.");
  }

  GX_OPEN_PARAM open_param;
  memset(&open_param, 0, sizeof(open_param));
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;
  if (!config_.serial_number.empty())
  {
    open_param.openMode = GX_OPEN_SN;
    open_param.pszContent = const_cast<char*>(config_.serial_number.c_str());
  }
  else
  {
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = const_cast<char*>("1");
  }

  checkStatus(GXOpenDevice(&open_param, &device_handle_), "GXOpenDevice");
  configureDevice();
  configureStream();
  configureTrigger();
  configureExposure();
  configureGain();
  configureGamma();
  configureContrast();
  allocateBuffers();
  if (channels_ == 3)
  {
    int64_t filter_value = GX_COLOR_FILTER_NONE;
    checkStatus(GXGetEnum(device_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &filter_value), "GXGetEnum PIXEL_COLOR_FILTER");
    color_filter_ = static_cast<GX_PIXEL_COLOR_FILTER_ENTRY>(filter_value);
  }
  else
  {
    color_filter_ = GX_COLOR_FILTER_NONE;
  }
}

void GalaxyCameraLowLevel::start()
{
  if (!device_handle_)
  {
    throw std::runtime_error("Camera not opened.");
  }
  if (!streaming_)
  {
    checkStatus(GXStreamOn(device_handle_), "GXStreamOn");
    streaming_ = true;
  }
}

void GalaxyCameraLowLevel::stop()
{
  if (device_handle_ && streaming_)
  {
    GXStreamOff(device_handle_);
    streaming_ = false;
  }
}

void GalaxyCameraLowLevel::close()
{
  if (device_handle_)
  {
    GXCloseDevice(device_handle_);
    device_handle_ = nullptr;
  }
  GXCloseLib();
}

bool GalaxyCameraLowLevel::grabFrame(FrameBuffer& frame)
{
  if (!device_handle_ || !streaming_)
  {
    return false;
  }

  PGX_FRAME_BUFFER frame_buffer = nullptr;
  GX_STATUS status = GXDQBuf(device_handle_, &frame_buffer, 1000);
  if (status == GX_STATUS_TIMEOUT)
  {
    return false;
  }
  checkStatus(status, "GXDQBuf");

  bool success = false;
  if (frame_buffer->nStatus == GX_FRAME_STATUS_SUCCESS)
  {
    convertImage(frame_buffer, frame);
    success = true;
  }

  GX_STATUS q_status = GXQBuf(device_handle_, frame_buffer);
  if (q_status != GX_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GalaxyCameraLowLevel"), "Failed to return buffer to queue.");
  }
  return success;
}

void GalaxyCameraLowLevel::configureDevice()
{
  channels_ = (config_.pixel_format == "mono8") ? 1 : 3;
  int64_t pixel_format = pixelFormatFromString(config_.pixel_format);
  if (pixel_format != 0)
  {
    GX_STATUS status = GXSetEnum(device_handle_, GX_ENUM_PIXEL_FORMAT, pixel_format);
    if (status != GX_STATUS_SUCCESS)
    {
      RCLCPP_WARN(rclcpp::get_logger("GalaxyCameraLowLevel"),
                  "Failed to set pixel format '%s' (status %d). Using device default.",
                  config_.pixel_format.c_str(), status);
    }
  }

  checkStatus(GXSetInt(device_handle_, GX_INT_WIDTH, config_.width), "GXSetInt WIDTH");
  checkStatus(GXSetInt(device_handle_, GX_INT_HEIGHT, config_.height), "GXSetInt HEIGHT");
  checkStatus(GXSetInt(device_handle_, GX_INT_OFFSET_X, 0), "GXSetInt OFFSET_X");
  checkStatus(GXSetInt(device_handle_, GX_INT_OFFSET_Y, 0), "GXSetInt OFFSET_Y");

  checkStatus(GXSetEnum(device_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON),
              "GXSetEnum FRAME_RATE_MODE");
  checkStatus(GXSetFloat(device_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, config_.frame_rate),
              "GXSetFloat FRAME_RATE");

  int64_t actual_width = 0;
  int64_t actual_height = 0;
  checkStatus(GXGetInt(device_handle_, GX_INT_WIDTH, &actual_width), "GXGetInt WIDTH");
  checkStatus(GXGetInt(device_handle_, GX_INT_HEIGHT, &actual_height), "GXGetInt HEIGHT");
  config_.width = static_cast<uint32_t>(actual_width);
  config_.height = static_cast<uint32_t>(actual_height);
}

void GalaxyCameraLowLevel::configureStream()
{
  constexpr uint64_t kAcquisitionBuffers = 6;
  GX_STATUS status = GXSetAcqusitionBufferNumber(device_handle_, kAcquisitionBuffers);
  if (status != GX_STATUS_SUCCESS)
  {
    RCLCPP_WARN(rclcpp::get_logger("GalaxyCameraLowLevel"),
                "Failed to set acquisition buffer number (status %d). Streaming jitter may increase.", status);
  }
}

void GalaxyCameraLowLevel::configureTrigger()
{
  if (config_.trigger.enable)
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON), "GXSetEnum TRIGGER_MODE");
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SOURCE, triggerSourceFromLine(config_.trigger.line)),
                "GXSetEnum TRIGGER_SOURCE");
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_TRIGGER_ACTIVATION,
                          triggerActivationFromString(config_.trigger.activation)),
                "GXSetEnum TRIGGER_ACTIVATION");
    int64_t line_selector = lineSelectorFromIndex(config_.trigger.line);
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_LINE_SELECTOR, line_selector), "GXSetEnum LINE_SELECTOR");
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT), "GXSetEnum LINE_MODE");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_TRIGGER_DELAY, config_.trigger.delay),
                "GXSetFloat TRIGGER_DELAY");
  }
  else
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF), "GXSetEnum TRIGGER_MODE");
  }
}

void GalaxyCameraLowLevel::configureExposure()
{
  std::string mode = toLower(config_.exposure.auto_mode);
  if (mode == "continuous")
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS),
                "GXSetEnum EXPOSURE_AUTO");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, config_.exposure.min),
                "GXSetFloat EXPOSURE_MIN");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, config_.exposure.max),
                "GXSetFloat EXPOSURE_MAX");
  }
  else if (mode == "once")
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_ONCE),
                "GXSetEnum EXPOSURE_AUTO");
  }
  else
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF),
                "GXSetEnum EXPOSURE_AUTO");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_EXPOSURE_TIME, config_.exposure.value),
                "GXSetFloat EXPOSURE_TIME");
  }
}

void GalaxyCameraLowLevel::configureGain()
{
  std::string mode = toLower(config_.gain.auto_mode);
  if (mode == "continuous")
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS), "GXSetEnum GAIN_AUTO");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_AUTO_GAIN_MIN, config_.gain.min), "GXSetFloat GAIN_MIN");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_AUTO_GAIN_MAX, config_.gain.max), "GXSetFloat GAIN_MAX");
  }
  else if (mode == "once")
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_ONCE), "GXSetEnum GAIN_AUTO");
  }
  else
  {
    checkStatus(GXSetEnum(device_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF), "GXSetEnum GAIN_AUTO");
    checkStatus(GXSetFloat(device_handle_, GX_FLOAT_GAIN, config_.gain.value), "GXSetFloat GAIN");
  }
}

void GalaxyCameraLowLevel::configureGamma()
{
  gamma_lut_.clear();
  if (config_.gamma.enable)
  {
    int lut_length = 0;
    DxGetGammatLut(config_.gamma.value, nullptr, &lut_length);
    if (lut_length > 0)
    {
      gamma_lut_.resize(lut_length);
      DxGetGammatLut(config_.gamma.value, gamma_lut_.data(), &lut_length);
    }
  }
}

void GalaxyCameraLowLevel::configureContrast()
{
  contrast_lut_.clear();
  if (config_.contrast.enable)
  {
    int lut_length = 0;
    DxGetContrastLut(config_.contrast.value, nullptr, &lut_length);
    if (lut_length > 0)
    {
      contrast_lut_.resize(lut_length);
      DxGetContrastLut(config_.contrast.value, contrast_lut_.data(), &lut_length);
    }
  }
}

void GalaxyCameraLowLevel::allocateBuffers()
{
  raw8_buffer_.resize(config_.width * config_.height);
}

void GalaxyCameraLowLevel::convertImage(PGX_FRAME_BUFFER frame, FrameBuffer& output)
{
  output.width = frame->nWidth;
  output.height = frame->nHeight;
  output.encoding = (channels_ == 1) ? "mono8" : "bgr8";
  output.step = frame->nWidth * channels_;
  output.data.resize(output.step * frame->nHeight);

  if (config_.pixel_format == "mono8")
  {
    std::memcpy(output.data.data(), frame->pImgBuf, frame->nWidth * frame->nHeight);
    return;
  }

  VxInt32 dx_status = DX_OK;
  DX_PIXEL_COLOR_FILTER dx_color_filter = toDxColorFilter(color_filter_);
  unsigned char* destination = output.data.data();
  switch (frame->nPixelFormat)
  {
    case GX_PIXEL_FORMAT_BAYER_GR8:
    case GX_PIXEL_FORMAT_BAYER_RG8:
    case GX_PIXEL_FORMAT_BAYER_GB8:
    case GX_PIXEL_FORMAT_BAYER_BG8:
      dx_status = DxRaw8toRGB24(static_cast<unsigned char*>(frame->pImgBuf), destination, frame->nWidth, frame->nHeight,
                                RAW2RGB_NEIGHBOUR, dx_color_filter, false);
      break;
    case GX_PIXEL_FORMAT_BAYER_GR10:
    case GX_PIXEL_FORMAT_BAYER_RG10:
    case GX_PIXEL_FORMAT_BAYER_GB10:
    case GX_PIXEL_FORMAT_BAYER_BG10:
    case GX_PIXEL_FORMAT_BAYER_GR12:
    case GX_PIXEL_FORMAT_BAYER_RG12:
    case GX_PIXEL_FORMAT_BAYER_GB12:
    case GX_PIXEL_FORMAT_BAYER_BG12:
      dx_status = DxRaw16toRaw8(static_cast<unsigned char*>(frame->pImgBuf), raw8_buffer_.data(), frame->nWidth,
                                frame->nHeight, DX_BIT_2_9);
      if (dx_status == DX_OK)
      {
        dx_status = DxRaw8toRGB24(raw8_buffer_.data(), destination, frame->nWidth, frame->nHeight,
                                  RAW2RGB_NEIGHBOUR, dx_color_filter, false);
      }
      break;
    default:
      throw std::runtime_error("Unsupported pixel format from camera.");
  }

  if (dx_status != DX_OK)
  {
    throw std::runtime_error("Failed to convert raw image to RGB.");
  }

  float* gamma_lut = gamma_lut_.empty() ? nullptr : gamma_lut_.data();
  float* contrast_lut = contrast_lut_.empty() ? nullptr : contrast_lut_.data();
  if (gamma_lut != nullptr || contrast_lut != nullptr)
  {
    DxImageImprovment(destination, destination, frame->nWidth, frame->nHeight, 0, contrast_lut, gamma_lut);
  }

  for (size_t idx = 0; idx + 2 < output.data.size(); idx += 3)
  {
    std::swap(destination[idx], destination[idx + 2]);
  }
}

void GalaxyCameraLowLevel::checkStatus(GX_STATUS status, const std::string& msg)
{
  if (status != GX_STATUS_SUCCESS)
  {
    throw std::runtime_error(msg + " failed with error code " + std::to_string(status));
  }
}

int64_t GalaxyCameraLowLevel::pixelFormatFromString(const std::string& format)
{
  std::string lower = toLower(format);
  if (lower == "mono8")
  {
    return GX_PIXEL_FORMAT_MONO8;
  }
  if (lower == "bayer_rg8")
    return GX_PIXEL_FORMAT_BAYER_RG8;
  if (lower == "bayer_gr8")
    return GX_PIXEL_FORMAT_BAYER_GR8;
  if (lower == "bayer_gb8")
    return GX_PIXEL_FORMAT_BAYER_GB8;
  if (lower == "bgr8" || lower == "bayer_bg8")
    return GX_PIXEL_FORMAT_BAYER_BG8;
  return 0;
}

GX_TRIGGER_ACTIVATION_ENTRY GalaxyCameraLowLevel::triggerActivationFromString(const std::string& activation)
{
  std::string lower = toLower(activation);
  if (lower == "falling")
  {
    return GX_TRIGGER_ACTIVATION_FALLINGEDGE;
  }
  if (lower == "level_high")
  {
    return GX_TRIGGER_ACTIVATION_LEVELHIGH;
  }
  if (lower == "level_low")
  {
    return GX_TRIGGER_ACTIVATION_LEVELLOW;
  }
  return GX_TRIGGER_ACTIVATION_RISINGEDGE;
}

GalaxyCameraNode::GalaxyCameraNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("galaxy_camera_ros2", options)
{
  camera_config_ = loadParameters();
  camera_name_ = camera_config_.camera_name;
  frame_id_ = camera_config_.frame_id;

  info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_,
                                                                           camera_config_.cam_info_url);
  if (!info_manager_->isCalibrated())
  {
    RCLCPP_WARN(get_logger(), "Camera info could not be loaded from '%s'.", camera_config_.cam_info_url.c_str());
  }
  camera_info_ = info_manager_->getCameraInfo();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos.get_rmw_qos_profile());

  camera_.open(camera_config_);
  camera_config_.width = camera_.width();
  camera_config_.height = camera_.height();

  if (camera_info_.width != 0 && camera_info_.width != camera_config_.width)
  {
    RCLCPP_WARN(get_logger(),
                "Camera info width (%u) does not match device output width (%u). Overriding camera info dimensions.",
                camera_info_.width, camera_config_.width);
  }
  if (camera_info_.height != 0 && camera_info_.height != camera_config_.height)
  {
    RCLCPP_WARN(get_logger(),
                "Camera info height (%u) does not match device output height (%u). Overriding camera info dimensions.",
                camera_info_.height, camera_config_.height);
  }
  camera_info_.width = camera_config_.width;
  camera_info_.height = camera_config_.height;

  camera_.start();
  startStreaming();
  RCLCPP_INFO(get_logger(), "Galaxy camera node started.");
}

GalaxyCameraNode::~GalaxyCameraNode()
{
  stopStreaming();
  camera_.stop();
  camera_.close();
}

CameraConfig GalaxyCameraNode::loadParameters()
{
  CameraConfig cfg;
  cfg.camera_name = declare_parameter<std::string>("camera_name", cfg.camera_name);
  cfg.frame_id = declare_parameter<std::string>("camera_frame_id", cfg.frame_id);
  cfg.serial_number = declare_parameter<std::string>("Camera.serial_number", "");
  cfg.cam_info_url =
      declare_parameter<std::string>("Camera.cam_info_url", "package://galaxy_camera_ros2/config/ost.yaml");
  cfg.frame_rate = declare_parameter<double>("Camera.frame_rate", cfg.frame_rate);
  cfg.pixel_format = toLower(declare_parameter<std::string>("Camera.pixel_format", cfg.pixel_format));
  const std::vector<std::string> supported_formats = { "mono8", "bgr8",  "bayer_rg8", "bayer_gr8", "bayer_gb8",
                                                       "bayer_bg8" };
  if (std::find(supported_formats.begin(), supported_formats.end(), cfg.pixel_format) == supported_formats.end())
  {
    throw std::runtime_error("Unsupported pixel_format '" + cfg.pixel_format +
                             "'. Supported values are mono8, bgr8, bayer_rg8, bayer_gr8, bayer_gb8, bayer_bg8.");
  }
  cfg.width = declare_parameter<int>("Camera.image_width", static_cast<int>(cfg.width));
  cfg.height = declare_parameter<int>("Camera.image_height", static_cast<int>(cfg.height));

  cfg.trigger.enable = declare_parameter<bool>("Camera.Trigger.enable", cfg.trigger.enable);
  cfg.trigger.line = declare_parameter<int>("Camera.Trigger.line", cfg.trigger.line);
  cfg.trigger.activation =
      declare_parameter<std::string>("Camera.Trigger.activation", cfg.trigger.activation);
  cfg.trigger.delay = declare_parameter<double>("Camera.Trigger.delay", cfg.trigger.delay);

  auto parse_auto_mode = [this](const std::string& param_name, const std::string& fallback) -> std::string {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    rclcpp::ParameterValue value = declare_parameter(param_name, rclcpp::ParameterValue(fallback), descriptor);
    switch (value.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return value.get<bool>() ? "continuous" : "off";
      case rclcpp::ParameterType::PARAMETER_STRING:
        return toLower(value.get<std::string>());
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return fallback;
      default:
        throw std::runtime_error("Unsupported parameter type for " + param_name);
    }
  };

  cfg.exposure.auto_mode = parse_auto_mode("Camera.Exposure.auto_mode", cfg.exposure.auto_mode);
  cfg.exposure.value = declare_parameter<double>("Camera.Exposure.value", cfg.exposure.value);
  cfg.exposure.min = declare_parameter<double>("Camera.Exposure.min", cfg.exposure.min);
  cfg.exposure.max = declare_parameter<double>("Camera.Exposure.max", cfg.exposure.max);

  cfg.gain.auto_mode = parse_auto_mode("Camera.Gain.auto_mode", cfg.gain.auto_mode);
  cfg.gain.value = declare_parameter<double>("Camera.Gain.value", cfg.gain.value);
  cfg.gain.min = declare_parameter<double>("Camera.Gain.min", cfg.gain.min);
  cfg.gain.max = declare_parameter<double>("Camera.Gain.max", cfg.gain.max);

  cfg.gamma.enable = declare_parameter<bool>("Camera.Gamma.enable", cfg.gamma.enable);
  cfg.gamma.value = declare_parameter<double>("Camera.Gamma.value", cfg.gamma.value);
  cfg.gamma.selector = declare_parameter<int>("Camera.Gamma.selector", cfg.gamma.selector);

  cfg.contrast.enable = declare_parameter<bool>("Camera.Contrast.enable", cfg.contrast.enable);
  cfg.contrast.value = declare_parameter<int>("Camera.Contrast.value", cfg.contrast.value);

  return cfg;
}

void GalaxyCameraNode::startStreaming()
{
  running_.store(true);
  capture_thread_ = std::thread(&GalaxyCameraNode::captureLoop, this);
}

void GalaxyCameraNode::stopStreaming()
{
  running_.store(false);
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }
}

void GalaxyCameraNode::captureLoop()
{
  FrameBuffer frame;
  const auto expected_step = camera_.width() * camera_.channels();
  frame.data.resize(expected_step * camera_.height());
  while (rclcpp::ok() && running_.load())
  {
    if (!camera_.grabFrame(frame))
    {
      continue;
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = now();
    image_msg.header.frame_id = frame_id_;
    image_msg.height = frame.height;
    image_msg.width = frame.width;
    image_msg.encoding = frame.encoding;
    image_msg.step = frame.step;
    image_msg.data.swap(frame.data);

    auto camera_info = camera_info_;
    camera_info.header = image_msg.header;
    camera_pub_.publish(image_msg, camera_info);
    frame.data.swap(image_msg.data);
  }
}

}  // namespace galaxy_camera

RCLCPP_COMPONENTS_REGISTER_NODE(galaxy_camera::GalaxyCameraNode)
