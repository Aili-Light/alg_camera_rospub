# Design: Unified Camera + IMU Publisher for ROS2

**Date:** 2026-07-11
**Status:** Approved
**Package:** `v4l2_publisher`

## Motivation

The existing codebase has three problems:

1. **Mixed languages:** Camera publishing is in C++ (`stereo_compressed_publisher.cpp`); IMU publishing is in Python (`start_imu_after_camera_ready.py`) which wraps an external binary.
2. **Stereo-only:** No single-camera publisher exists. For a single-camera setup, the stereo publisher can't be used.
3. **Code duplication risk:** The stereo publisher contains a reusable `V4L2CameraPipeline` class, but it's not extracted. Without a shared backend, a single-camera publisher would be a copy-paste fork.

## Goals

- Unify everything in C++ (no Python, no external IMU binary)
- Extract V4L2 capture into a reusable, ROS-agnostic library
- Build IMU I2C driver as a reusable C++ library (leveraging existing C HAL from `imu_demo`)
- Support both single-camera and stereo-camera configurations
- Each camera and the IMU are independently launchable nodes
- Precise, offset-corrected timestamps on all published messages
- All hardware parameters configurable via launch arguments

## Non-Goals

- Support for non-V4L2 camera backends (GStreamer, etc.)
- Support for IMU chips other than ICM-45686 (though the design makes it easy to add)
- Hardware synchronization between camera and IMU timestamps (software correction only)

---

## Architecture Overview

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
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│  /dev/video0     │ │  /dev/video1     │ │  i2c_drive_linux │
│  (V4L2 kernel)   │ │  (V4L2 kernel)   │ │  I2C HAL (C)     │
└─────────────────┘ └─────────────────┘ └────────┬────────┘
                                                 │
                                                 ▼
                                        ┌─────────────────┐
                                        │  ICM-45686       │
                                        │  (I2C registers)  │
                                        └─────────────────┘
```

Three independent ROS2 nodes sharing two libraries:

| Library | Language | Purpose |
|---|---|---|
| `libv4l2_capture` | C++ only | V4L2 device open, mmap, dequeue, luma extraction |
| `libi2c_imu` | C (I2C HAL) + C++ (IMU wrapper) | I2C bus communication, ICM-45686 register read/write |

Each node compiles as both a standalone executable and an `rclcpp_components` shared library for container loading.

---

## Source Tree

```
alg_cam_rospub/
├── include/v4l2_publisher/
│   ├── v4l2_capture.hpp         # V4L2Capture class
│   ├── clock_offset.hpp         # ClockOffset utility
│   ├── imu_device.hpp           # ImuDevice abstract interface
│   ├── imu_icm_45686.hpp        # ImuIcm45686 concrete driver
│   └── rectification.hpp        # RectificationMaps loader (optional)
├── src/
│   ├── v4l2_capture.cpp
│   ├── clock_offset.cpp
│   ├── i2c_driver_linux.c       # From imu_demo, unchanged
│   ├── i2c_driver_linux.h       # From imu_demo, unchanged
│   ├── i2c_hal.h                # From imu_demo, unchanged
│   ├── imu_core.h               # From imu_demo (C structs, ops table)
│   ├── imu_device.cpp           # C++ ImuDevice base impl
│   ├── imu_icm_45686.cpp        # C++ wrapper — absorbs register logic from imu_icm_45686.c
│   ├── rectification.cpp
│   ├── single_camera_node.cpp   # SingleCameraNode (composable component)
│   ├── stereo_camera_node.cpp   # StereoCameraNode (composable component)
│   ├── imu_node.cpp             # ImuNode (composable component)
│   ├── single_camera_main.cpp   # Standalone entry point
│   ├── stereo_camera_main.cpp   # Standalone entry point
│   └── imu_main.cpp             # Standalone entry point
├── launch/
│   ├── single_camera_with_imu.launch.py
│   └── stereo_camera_with_imu.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Component Designs

### 1. V4L2Capture (ROS-agnostic)

Extracted from the existing `V4L2CameraPipeline` class. Removes all ROS dependencies (`rclcpp::Node`, `rclcpp::Time`, publishers).

```cpp
struct V4L2Config {
    std::string device_path;          // e.g. "/dev/video0"
    uint32_t requested_width  = 1280;
    uint32_t requested_height = 800;
    uint32_t target_width     = 640;
    uint32_t target_height    = 400;
    uint32_t pixel_format     = V4L2_PIX_FMT_YUYV;
    uint32_t buffer_count     = 4;
    double   requested_fps    = 60.0;
};

class V4L2Capture {
public:
    explicit V4L2Capture(const V4L2Config & config);
    ~V4L2Capture();

    void start();
    void stop();

    // Returns the latest grayscale frame, or nullopt on failure/miss
    std::optional<cv::Mat> grab();

    // Timestamp of last captured frame: V4L2 buffer timestamp (monotonic)
    // converted to system-clock nanoseconds via ClockOffset.
    // Valid after a successful grab(). Returns 0 if no frame yet.
    int64_t frame_timestamp_ns() const;

    // Query helpers
    uint32_t actual_width() const;
    uint32_t actual_height() const;
    double   actual_fps() const;
    uint32_t publish_width() const;
    uint32_t publish_height() const;
    const std::string & fourcc_string() const;

    struct Stats { uint64_t capture_count; uint64_t publish_count; };
    Stats snapshot_and_reset_stats();
};
```

**Key behaviors (unchanged from existing code):**
- Opens device with `O_RDWR | O_NONBLOCK`
- Configures format via `VIDIOC_S_FMT`, falls back to actual device capabilities
- Falls back to `publish_actual_no_crop` mode if aspect ratio differs
- MMAPs `buffer_count` buffers, queues them, streams on
- Blocking `grab()`: `select()` with 2-second timeout, `VIDIOC_DQBUF`, luma extraction
- Supports YUYV, YVYU, UYVY, VYUY, NV12, NV21 pixel formats
- Resizes to target dimensions if aspect ratio matches
- Publishes `mono8` grayscale only

### 2. ClockOffset

Shared utility for precise, drift-corrected timestamps:

```cpp
class ClockOffset {
public:
    ClockOffset();

    // Recalibrate: measures offset between system_clock and steady_clock
    void calibrate();

    // Convert a monotonic (steady_clock) timestamp to system-clock ns
    int64_t corrected_system_ns(int64_t steady_ns) const;

    // Current offset-corrected system time in ns
    int64_t now_ns() const;

private:
    int64_t offset_ns_;
};
```

**Usage:**
- **Camera:** After `VIDIOC_DQBUF`, the `buffer.timestamp` (monotonic) is passed through `corrected_system_ns()`. The node sets `msg.header.stamp` from this value.
- **IMU:** After `imu_read_accel()`/`imu_read_gyro()`, call `now_ns()`. The node sets `msg.header.stamp` from this value.
- **Recalibration:** Each node periodically recalibrates (e.g., every 1 second) to guard against clock drift.

### 3. ImuDevice (abstract interface)

```cpp
class ImuDevice {
public:
    virtual ~ImuDevice() = default;
    virtual bool read_accel(float & x, float & y, float & z) = 0;
    virtual bool read_gyro(float & x, float & y, float & z) = 0;
    virtual bool read_temp(float & temp) = 0;
};
```

### 4. ImuIcm45686 (concrete implementation)

Wraps the existing C code (`imu_icm_45686.c`, `imu_core.h`, `i2c_drive_linux.c`):

```cpp
class ImuIcm45686 : public ImuDevice {
public:
    ImuIcm45686(const std::string & i2c_device_path, uint8_t i2c_addr);
    ~ImuIcm45686() override;

    bool read_accel(float & x, float & y, float & z) override;
    bool read_gyro(float & x, float & y, float & z) override;
    bool read_temp(float & temp) override;

private:
    struct hal_i2c i2c_bus_;
    struct imu_device * dev_;   // from imu_icm_45686_create()
};
```

**Initialization:**
1. `hal_i2c_linux_init(&i2c_bus_, device_path)` — opens I2C bus once
2. `imu_icm_45686_create(&i2c_bus_, i2c_addr)` — configures registers (PWR_MGMT0, ACCEL_CONFIG0, GYRO_CONFIG0)
3. Sensor configured for: 8G accelerometer, 2000 dps gyro, 200 Hz ODR, low-noise mode

**I2C bus:** Kept open for the lifetime of the node (not opened/closed per transfer). The `hal_i2c_linux_init` pattern does this correctly.

### 5. Nodes

#### SingleCameraNode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `device_path` | string | `/dev/video0` | V4L2 device node |
| `requested_width` | int | 1280 | Width requested from device |
| `requested_height` | int | 800 | Height requested from device |
| `target_width` | int | 640 | Published image width |
| `target_height` | int | 400 | Published image height |
| `pixel_format` | string | `YUYV` | FourCC pixel format (string, converted internally to V4L2 fourcc code) |
| `fps` | double | 60.0 | Target frame rate |
| `topic_name` | string | `/image` | ROS topic for published image |
| `frame_id` | string | `camera_link` | ROS frame ID |
| `publish_rectified` | bool | false | Enable rectified output |
| `calib_path` | string | `./camera_calib.yaml` | Calibration YAML for rectification maps |

**Runtime:** Creates one `V4L2Capture`, one `ClockOffset`. Main loop: `grab()` → timestamp → publish. If `publish_rectified`, loads rectification maps and publishes a second topic `{topic_name}_rect`.

#### StereoCameraNode

Same parameters as single, but duplicated with `left_` and `right_` prefixes:
`left_device_path`, `right_device_path`, `left_topic_name`, `right_topic_name`, `left_frame_id`, `right_frame_id`, etc.

Additionally: `publish_rectified` + `stereo_calib_path` to load left and right rectification maps from a stereo calibration file.

**Runtime:** Creates two `V4L2Capture` instances, two publisher pairs. Two capture threads (one per camera). Stats logging at 1 Hz.

#### ImuNode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `i2c_bus` | string | `/dev/i2c-3` | I2C bus device |
| `imu_address` | int | `0x69` | IMU I2C address (7-bit) |
| `frequency` | double | 100.0 | Publishing rate in Hz |
| `topic_name` | string | `imu/data` | ROS topic |
| `frame_id` | string | `imu_link` | ROS frame ID |

**Unit conversions** (from raw sensor values):
- Gyroscope: sensor reads deg/s → publish rad/s (`× π / 180`)
- Accelerometer: sensor reads g → publish m/s² (`× 9.80665`)

**Runtime:** Wall timer at requested frequency. Each tick: `read_accel()` + `read_gyro()` → timestamp via `ClockOffset::now_ns()` → unit conversion → publish `sensor_msgs/msg::Imu`.

---

## CMake Build Plan

```cmake
# Shared libraries
add_library(v4l2_capture STATIC
    src/v4l2_capture.cpp
    src/clock_offset.cpp
    src/rectification.cpp
)

add_library(i2c_imu STATIC
    src/i2c_driver_linux.c      # C HAL (unchanged from imu_demo)
    src/imu_device.cpp
    src/imu_icm_45686.cpp       # wraps imu_icm_45686.c logic
)

# Composable node libraries
add_library(single_camera_component SHARED src/single_camera_node.cpp)
target_link_libraries(single_camera_component v4l2_capture)
ament_target_dependencies(single_camera_component rclcpp rclcpp_components sensor_msgs)

add_library(stereo_camera_component SHARED src/stereo_camera_node.cpp)
target_link_libraries(stereo_camera_component v4l2_capture)
ament_target_dependencies(stereo_camera_component rclcpp rclcpp_components sensor_msgs)

add_library(imu_component SHARED src/imu_node.cpp)
target_link_libraries(imu_component i2c_imu)
ament_target_dependencies(imu_component rclcpp rclcpp_components sensor_msgs)

# Standalone executables
foreach(NODE single_camera stereo_camera imu)
    add_executable(${NODE}_publisher src/${NODE}_main.cpp)
    target_link_libraries(${NODE}_publisher ${NODE}_component)
endforeach()

# Install
install(TARGETS ... DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Component registration
rclcpp_components_register_node(single_camera_component
    PLUGIN "v4l2_publisher::SingleCameraNode" EXECUTABLE single_camera_publisher)
rclcpp_components_register_node(stereo_camera_component
    PLUGIN "v4l2_publisher::StereoCameraNode" EXECUTABLE stereo_camera_publisher)
rclcpp_components_register_node(imu_component
    PLUGIN "v4l2_publisher::ImuNode" EXECUTABLE imu_publisher)
```

---

## Launch Files

### single_camera_with_imu.launch.py

Starts `single_camera_publisher` + `imu_publisher`. All camera and IMU parameters exposed as launch arguments with defaults.

### stereo_camera_with_imu.launch.py

Starts `stereo_camera_publisher` + `imu_publisher`. Left/right camera parameters plus IMU parameters.

Both launch files recreate the parameter-passing pattern from the existing `stereo_camera_with_imu.launch.py`.

---

## Error Handling

- **Device not found:** Throw `std::runtime_error` at node init → node fails to start, ROS logs FATAL.
- **I2C bus not found:** `ImuIcm45686` constructor throws → node fails to start.
- **Frame timeout (2s):** Log warning, continue loop. No crash.
- **IMU read failure:** Log warning, skip that publish cycle. Continue.
- **Device disconnect mid-run:** `select()` or `ioctl` fails → log error, attempt re-open in a backoff loop.

---

## What Goes Away

- `scripts/start_imu_after_camera_ready.py` — replaced by `imu_publisher` C++ node
- `imu_ros2_pub_bin/imu_ros2_pub_test` — IMU reading is now in-process, no external binary
- `imu_ros2_pub_bin/libSDK.so` — no longer needed
- `src/camera_backend_probe.cpp` — removed (was a diagnostic tool, not part of the publish pipeline)

---

## What Stays (logic preserved, files restructured)

- The `V4L2CameraPipeline` logic from `src/stereo_compressed_publisher.cpp` — extracted into `V4L2Capture` in `v4l2_capture.cpp`. The original `.cpp` is removed.
- `launch/stereo_camera_with_imu.launch.py` — rewritten to use new executables
- `package.xml` — updated dependencies (add `rclcpp_components`, remove `rclpy`, `ament_index_python`)
- The I2C C HAL from `imu_demo` — `i2c_drive_linux.c`, `i2c_drive_linux.h`, `i2c_hal.h`, `imu_core.h` — incorporated into `src/`
- ICM-45686 register definitions and init/read routines — absorbed from `imu_icm_45686.c` into the C++ `imu_icm_45686.cpp`

---

## Dependencies

| Dependency | Purpose |
|---|---|
| `rclcpp` | ROS2 C++ client library |
| `rclcpp_components` | Composable node support |
| `sensor_msgs` | Image and Imu message types |
| `OpenCV` (core + imgproc) | Frame manipulation, luma extraction, resize, rectification |
| Linux kernel headers | V4L2 ioctl, I2C ioctl |

No Python, no external IMU binary, no rclpy at runtime.

---

## Future Extensibility

- **Additional IMU chips:** Implement `ImuDevice` interface, plug in. Node code unchanged.
- **Additional camera backends:** Implement a `CameraCapture` abstract interface, add GStreamer or USB backends.
- **Calibration rectification:** The `RectificationMaps` loader is already factored out. An additional calibration service could live-reload maps.
