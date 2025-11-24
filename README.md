# Galaxy-Camera-ROS2

[English](#english) | [中文说明](#中文说明)

---

## English

ROS 2 driver for Daheng Galaxy industrial cameras. The node configures a device through the official Galaxy SDK, converts the Bayer stream to color images and publishes synchronized `image_raw` and `camera_info` topics for downstream perception components.

## Requirements

- ROS 2 Humble (or later) with `rclcpp`, `image_transport`, `camera_info_manager`
- Daheng Galaxy SDK (`libgxiapi.so`) installed on the system

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/BreCaspian/Galaxy-Camera-ROS2.git galaxy_camera_ros2
cd ..
colcon build --packages-select galaxy_camera_ros2
source install/setup.bash
```

## Configuration

Runtime settings live in `config/camera.yaml`, which is a ROS 2 parameter file following the standard
`<node_name>.ros__parameters` layout:

```yaml
galaxy_camera_ros2:
  ros__parameters:
    camera_name: galaxy_camera_ros2
    Camera:
      pixel_format: bgr8  # allowed values: mono8, bgr8
      serial_number: ""
      cam_info_url: package://galaxy_camera_ros2/config/ost.yaml
      # ...
```

Important fields include:

- `camera_name` / `camera_frame_id` – frame metadata stamped on every message.
- `Camera.serial_number` – camera SN (leave empty to open the first detected device).
- `Camera.cam_info_url` – path to the calibration YAML (`config/ost.yaml` by default).
- `Camera.frame_rate`, `Camera.pixel_format` (device formats: `mono8`, `bgr8`, `bayer_rg8`, `bayer_gr8`, `bayer_gb8`, `bayer_bg8`), `Camera.image_width`, `Camera.image_height`.  
  *When changing the resolution, update `config/ost.yaml` accordingly; the node will override mismatched dimensions but calibration accuracy will be impacted.*
- `Camera.Trigger.*`, `Camera.Exposure.*`, `Camera.Gain.*`, `Camera.Gamma.*`, `Camera.Contrast.*` – low-level SDK parameters to control triggering and image quality.

The calibration file `config/ost.yaml` must match the output resolution; the driver will abort if the dimensions disagree in order to avoid publishing inconsistent camera models.

## Running

```bash
ros2 launch galaxy_camera_ros2 galaxy_camera.launch.py \
  params_file:=/path/to/galaxy_camera_ros2/config/camera.yaml
```

Topics:

- `/image_raw` (remappable): `sensor_msgs/msg/Image` with `SensorDataQoS`
- `/camera_info`: `sensor_msgs/msg/CameraInfo` synchronized with the image timestamp

Use standard ROS 2 tools for visualization or playback:

```bash
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /camera_info
```

## Notes

- The driver currently targets a single camera instance. Multi-camera setups can be achieved by launching the component multiple times with different parameter files.
- Dynamic parameter updates are not yet implemented; modify the YAML file and restart the node to apply changes.

## Maintainer & License

- Maintainer: yaoyuzhuo (yaoyuzhuo6@gmail.com / GitHub: [BreCaspian](https://github.com/BreCaspian))
- License: GPL-3.0-or-later

compressedDepth 仅支持单通道 16-bit/32-bit 深度图，而当前驱动输出为彩色 bgr8，因此选择 /image_raw/compressedDepth 时会继续看到插件抛错，这是预期行为（需要使用深度相机才能启用）。

---

## 中文说明

Galaxy-Camera-ROS2 是针对大恒 Galaxy 工业相机的 ROS 2 驱动。节点通过官方 Galaxy SDK 配置设备、将 Bayer 数据转换为彩色图像，并同步发布 `image_raw` 与 `camera_info` 话题，方便下游感知模块使用。

### 环境需求

- ROS 2 Humble（或更高版本），需包含 `rclcpp`、`image_transport`、`camera_info_manager` 等依赖  
- 已安装大恒 Galaxy SDK（提供 `libgxiapi.so`）

### 构建步骤

```bash
cd ~/ros2_ws/src
git clone https://github.com/BreCaspian/Galaxy-Camera-ROS2.git galaxy_camera_ros2
cd ..
colcon build --packages-select galaxy_camera_ros2
source install/setup.bash
```

### 参数配置

所有运行参数位于 `config/camera.yaml`，该文件遵循 ROS 2 标准的 `<node_name>.ros__parameters` 结构：

```yaml
galaxy_camera_ros2:
  ros__parameters:
    camera_name: galaxy_camera_ros2
    Camera:
      pixel_format: bgr8  # 可选 mono8 / bgr8 / bayer_rg8 / bayer_gr8 / bayer_gb8 / bayer_bg8
      serial_number: ""
      cam_info_url: package://galaxy_camera_ros2/config/ost.yaml
      # ...
```

关键参数说明：

- `camera_name` / `camera_frame_id`：消息头里的 frame 标识
- `Camera.serial_number`：相机序列号（留空则连接首个设备）
- `Camera.cam_info_url`：相机标定文件路径（默认 `config/ost.yaml`）
- `Camera.frame_rate`、`Camera.pixel_format`、图像宽高等；当修改分辨率时请同步更新 `config/ost.yaml`，否则虽可自动覆盖，但标定精度会下降
- `Camera.Trigger.*`、`Camera.Exposure.*`、`Camera.Gain.*`、`Camera.Gamma.*`、`Camera.Contrast.*`：对应 SDK 提供的触发与图像质量控制选项。`auto_mode` 字段既可写字符串（continuous/once/off），也可直接使用布尔值（true→continuous，false→off）

### 运行方法

```bash
ros2 launch galaxy_camera_ros2 galaxy_camera.launch.py \
  params_file:=/path/to/galaxy_camera_ros2/config/camera.yaml
```

发布的话题：

- `/image_raw`：`sensor_msgs/msg/Image`，使用可靠 QoS
- `/camera_info`：`sensor_msgs/msg/CameraInfo`，时间戳与图像同步

可使用以下工具查看：

```bash
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /camera_info
```

### 其他说明

- 目前默认仅控制单台相机，如需多相机可多次启动组件并指定不同参数文件
- 暂未支持动态参数更新，修改 YAML 后需要重启节点
- `/image_raw/compressedDepth` 仅适用于单通道深度图，当前驱动输出 `bgr8`，因此该话题会持续报错属于正常现象

### 维护者 & 许可

- 维护者：yaoyuzhuo（yaoyuzhuo6@gmail.com / GitHub: [BreCaspian](https://github.com/BreCaspian)）
- 许可协议：GPL-3.0-or-later
