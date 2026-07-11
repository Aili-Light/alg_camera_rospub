# ALG Camera ROS Publisher

ROS2 package `alg_camera_rospub` for publishing camera images (single or stereo) via V4L2 and IMU data via I2C (ICM-45686).

## Prerequisites

- ROS2 Humble (or newer)
- OpenCV
- Linux with V4L2 and I2C kernel support

## Build

```bash
cd /path/to/alg_cam_rospub
colcon build --packages-select alg_camera_rospub
source install/setup.bash
```

## Executables

| Name | Description |
|------|-------------|
| `single_camera_publisher` | Reads one camera via V4L2, publishes `mono8` images |
| `stereo_camera_publisher` | Reads two cameras via V4L2, publishes left + right `mono8` images |
| `imu_publisher` | Reads ICM-45686 via I2C, publishes `sensor_msgs/msg/Imu` |

## Launch

All parameters have defaults — only override what you need.

### Single camera + IMU

```bash
source install/setup.bash
ros2 launch alg_camera_rospub single_camera_with_imu.launch.py \
  device_path:=/dev/video0 \
  requested_width:=2560 \
  requested_height:=800 \
  target_width:=2560 \
  target_height:=800 \
  pixel_format:=UYVY \
  fps:=30.0 \
  i2c_bus:=/dev/i2c-3 \
  imu_address:=105 \
  imu_start_delay:=2.0
```

### Stereo camera + IMU

```bash
source install/setup.bash
ros2 launch alg_camera_rospub stereo_camera_with_imu.launch.py \
  left_device_path:=/dev/video0 \
  right_device_path:=/dev/video1 \
  i2c_bus:=/dev/i2c-3
```

### Camera-only (no IMU)

```bash
ros2 run alg_camera_rospub single_camera_publisher --ros-args \
  -p device_path:=/dev/video0 \
  -p requested_width:=2560 \
  -p requested_height:=800 \
  -p target_width:=2560 \
  -p target_height:=800 \
  -p pixel_format:=UYVY
```

### IMU-only (no camera)

```bash
ros2 run alg_camera_rospub imu_publisher --ros-args \
  -p i2c_bus:=/dev/i2c-3 \
  -p imu_address:=105 \
  -p frequency:=100.0
```

## Parameters

### Camera node (single)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_path` | string | `/dev/video0` | V4L2 device node |
| `requested_width` | int | `1280` | Width requested from device |
| `requested_height` | int | `800` | Height requested from device |
| `target_width` | int | `640` | Published image width (set equal to requested for no resize) |
| `target_height` | int | `400` | Published image height |
| `pixel_format` | string | `YUYV` | FourCC: `YUYV`, `UYVY`, `YVYU`, `VYUY`, `NV12`, `NV21` |
| `fps` | double | `60.0` | Target frame rate |
| `topic_name` | string | `/image` | ROS topic name |
| `frame_id` | string | `camera_link` | ROS frame ID |
| `publish_rectified` | bool | `false` | Whether to also publish a rectified image |
| `calib_path` | string | `./camera_calib.yaml` | Path to calibration YAML (for rectification) |

### Camera node (stereo)

Same parameters as single, but prefixed with `left_` and `right_`. Example: `left_device_path`, `right_device_path`, `left_topic_name`, etc.

### IMU node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_bus` | string | `/dev/i2c-3` | I2C bus device path |
| `imu_address` | int | `105` | IMU I2C address — `104` (0x68) or `105` (0x69) |
| `frequency` | double | `100.0` | Publishing rate in Hz (max 200) |
| `topic_name` | string | `imu/data` | ROS topic name |
| `frame_id` | string | `imu_link` | ROS frame ID |

### Launch-only parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `imu_start_delay` | double | `2.0` | Delay (seconds) before starting IMU node. Increase if camera streaming takes longer to enable the I2C bus. |

## Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/image` (or custom) | `sensor_msgs/msg/Image` | Grayscale `mono8` image |
| `/image_rect` | `sensor_msgs/msg/Image` | Rectified image (if `publish_rectified:=true`) |
| `imu/data` (or custom) | `sensor_msgs/msg/Imu` | IMU data (accel in m/s², gyro in rad/s) |

## Permissions

The user running the nodes must have access to V4L2 and I2C devices:

```bash
# Add user to required groups
sudo usermod -a -G video,i2c $USER

# Or grant direct access (temporary)
sudo chmod 666 /dev/video0 /dev/video1 /dev/i2c-3
```

Log out and back in after group changes.

## Architecture

```
                    ┌──────────────────────┐
                    │   Launch Files        │
                    │  (single or stereo)   │
                    └──────┬───────────────┘
                           │
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ SingleCameraNode│ │ StereoCameraNode│ │    ImuNode      │
│  (composable)   │ │  (composable)   │ │  (composable)   │
└────────┬────────┘ └────────┬────────┘ └────────┬────────┘
         │                   │                   │
         ▼                   ▼                   ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│  V4L2Capture     │ │  V4L2Capture    │ │  ImuDevice      │
│  (ROS-agnostic)  │ │  (ROS-agnostic)  │ │  (abstract iface)│
└────────┬────────┘ └────────┬────────┘ └────────┬────────┘
         │                   │                   │
         ▼                   ▼                   ▼
   /dev/video0          /dev/video1          i2c_drive_linux
   (V4L2 kernel)        (V4L2 kernel)        I2C HAL (C)
                                             └─────┬─────┘
                                                ICM-45686
```
