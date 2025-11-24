#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "libgxiapi/GxIAPI.h"
#include "libgxiapi/DxImageProc.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace galaxy_camera
{
struct TriggerConfig
{
  bool enable{ false };
  int line{ 3 };
  std::string activation{ "rising" };
  double delay{ 0.0 };
};

struct ExposureConfig
{
  std::string auto_mode{ "continuous" };
  double value{ 2000.0 };
  double min{ 100.0 };
  double max{ 4500.0 };
};

struct GainConfig
{
  std::string auto_mode{ "continuous" };
  double value{ 2.0 };
  double min{ 0.0 };
  double max{ 16.0 };
};

struct GammaConfig
{
  bool enable{ true };
  double value{ 1.0 };
  int selector{ 1 };
};

struct ContrastConfig
{
  bool enable{ false };
  int value{ 5 };
};

struct CameraConfig
{
  std::string camera_name{ "galaxy_camera_ros2" };
  std::string frame_id{ "camera_optical_frame" };
  std::string serial_number;
  std::string cam_info_url;
  double frame_rate{ 30.0 };
  std::string pixel_format{ "bgr8" };
  uint32_t width{ 1280 };
  uint32_t height{ 1024 };
  TriggerConfig trigger;
  ExposureConfig exposure;
  GainConfig gain;
  GammaConfig gamma;
  ContrastConfig contrast;
};

struct FrameBuffer
{
  std::vector<uint8_t> data;
  uint32_t width{ 0 };
  uint32_t height{ 0 };
  uint32_t step{ 0 };
  std::string encoding;
};

class GalaxyCameraLowLevel
{
public:
  GalaxyCameraLowLevel();
  ~GalaxyCameraLowLevel();

  void open(const CameraConfig& config);
  void start();
  void stop();
  void close();
  bool is_open() const { return device_handle_ != nullptr; }
  bool grabFrame(FrameBuffer& frame);
  uint32_t width() const { return config_.width; }
  uint32_t height() const { return config_.height; }
  uint32_t channels() const { return channels_; }

private:
  void configureDevice();
  void configureStream();
  void configureTrigger();
  void configureExposure();
  void configureGain();
  void configureGamma();
  void configureContrast();
  void allocateBuffers();
  void convertImage(PGX_FRAME_BUFFER frame, FrameBuffer& output);
  void checkStatus(GX_STATUS status, const std::string& msg);
  int64_t pixelFormatFromString(const std::string& format);
  GX_TRIGGER_ACTIVATION_ENTRY triggerActivationFromString(const std::string& activation);

  CameraConfig config_;
  GX_DEV_HANDLE device_handle_{ nullptr };
  GX_PIXEL_COLOR_FILTER_ENTRY color_filter_{ GX_COLOR_FILTER_NONE };
  std::vector<uint8_t> raw8_buffer_;
  std::vector<float> gamma_lut_;
  std::vector<float> contrast_lut_;
  bool streaming_{ false };
  uint32_t channels_{ 3 };
};

class GalaxyCameraNode : public rclcpp::Node
{
public:
  explicit GalaxyCameraNode(const rclcpp::NodeOptions& options);
  ~GalaxyCameraNode() override;

private:
  CameraConfig loadParameters();
  void startStreaming();
  void stopStreaming();
  void captureLoop();

  GalaxyCameraLowLevel camera_;
  CameraConfig camera_config_;
  std::thread capture_thread_;
  std::atomic<bool> running_{ false };
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_;
  std::string camera_name_;
  std::string frame_id_;
};

}  // namespace galaxy_camera
