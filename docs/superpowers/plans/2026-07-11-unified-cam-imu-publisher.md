# Unified Camera + IMU Publisher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build three independent, composable ROS2 nodes (single camera, stereo camera, IMU) sharing two reusable libraries (V4L2 capture, I2C IMU driver) — all in C++ with precise offset-corrected timestamps.

**Architecture:** Two static libraries (`libv4l2_capture`, `libi2c_imu`) provide hardware abstraction. Three composable component nodes consume them and publish `sensor_msgs/msg/Image` and `sensor_msgs/msg/Imu`. Each component also has a standalone executable entry point.

**Tech Stack:** C++17, ROS2 (rclcpp, rclcpp_components, sensor_msgs), OpenCV (core + imgproc), Linux V4L2 + I2C kernel interfaces.

## Global Constraints

- C++17, ROS-agnostic libraries, composable nodes via rclcpp_components
- All hardware parameters configurable via ROS2 launch arguments
- V4L2Capture must not depend on any ROS headers
- Timestamps use offset-corrected system-clock nanoseconds
- Single-camera and stereo-camera share the same V4L2Capture backend
- ImuDevice abstract interface allows future IMU chip additions
- Unit conversions: gyro deg/s → rad/s, accel g → m/s²

---

## Task 1: Project Scaffolding and C HAL Files

**Files:**
- Create: `include/v4l2_publisher/` (directory)
- Create: `src/i2c_hal.h`
- Create: `src/i2c_drive_linux.h`
- Create: `src/i2c_drive_linux.c`
- Create: `src/imu_core.h`

**Interfaces:**
- Produces: C HAL headers and implementation for I2C + IMU vtable pattern, consumed by Task 5

- [ ] **Step 1: Create include directory**

```bash
mkdir -p /home/pi/workspace/alg_cam_rospub/include/v4l2_publisher
```

- [ ] **Step 2: Copy i2c_hal.h from imu_demo**

```bash
cp /home/pi/Downloads/imu_demo/source/driver/i2c/i2c_hal.h /home/pi/workspace/alg_cam_rospub/src/i2c_hal.h
```

- [ ] **Step 3: Copy i2c_drive_linux.h from imu_demo**

```bash
cp /home/pi/Downloads/imu_demo/source/driver/i2c/i2c_drive_linux.h /home/pi/workspace/alg_cam_rospub/src/i2c_drive_linux.h
```

- [ ] **Step 4: Copy i2c_drive_linux.c from imu_demo**

```bash
cp /home/pi/Downloads/imu_demo/source/driver/i2c/i2c_drive_linux.c /home/pi/workspace/alg_cam_rospub/src/i2c_drive_linux.c
```

- [ ] **Step 5: Copy imu_core.h from imu_demo**

```bash
cp /home/pi/Downloads/imu_demo/source/app/imu/include/imu_core.h /home/pi/workspace/alg_cam_rospub/src/imu_core.h
```

- [ ] **Step 6: Verify files were copied**

```bash
ls -la /home/pi/workspace/alg_cam_rospub/src/i2c_hal.h \
       /home/pi/workspace/alg_cam_rospub/src/i2c_drive_linux.h \
       /home/pi/workspace/alg_cam_rospub/src/i2c_drive_linux.c \
       /home/pi/workspace/alg_cam_rospub/src/imu_core.h \
       /home/pi/workspace/alg_cam_rospub/include/v4l2_publisher/
```
Expected: five paths exist, no errors.

---

## Task 2: ClockOffset Utility

**Files:**
- Create: `include/v4l2_publisher/clock_offset.hpp`
- Create: `src/clock_offset.cpp`

**Interfaces:**
- Produces: `ClockOffset` class — `calibrate()`, `corrected_system_ns(int64_t) -> int64_t`, `now_ns() -> int64_t`

- [ ] **Step 1: Write the header file**

Write `include/v4l2_publisher/clock_offset.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__CLOCK_OFFSET_HPP_
#define V4L2_PUBLISHER__CLOCK_OFFSET_HPP_

#include <cstdint>

namespace v4l2_publisher
{

class ClockOffset
{
public:
  ClockOffset();

  /// Re-measure the offset between system_clock and steady_clock.
  /// Call at init and periodically to guard against clock drift.
  void calibrate();

  /// Convert a monotonic (steady_clock) timestamp in nanoseconds
  /// to an offset-corrected system-clock timestamp in nanoseconds.
  int64_t corrected_system_ns(int64_t steady_ns) const;

  /// Current offset-corrected system time in nanoseconds.
  /// Equivalent to corrected_system_ns(steady_clock::now()).
  int64_t now_ns() const;

private:
  int64_t offset_ns_;
};

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__CLOCK_OFFSET_HPP_
```

- [ ] **Step 2: Write the implementation**

Write `src/clock_offset.cpp`:

```cpp
#include "v4l2_publisher/clock_offset.hpp"

#include <chrono>

namespace v4l2_publisher
{

ClockOffset::ClockOffset()
: offset_ns_(0)
{
  calibrate();
}

void ClockOffset::calibrate()
{
  const auto system_now = std::chrono::system_clock::now();
  const auto steady_now = std::chrono::steady_clock::now();

  const auto system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    system_now.time_since_epoch()).count();
  const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();

  offset_ns_ = system_ns - steady_ns;
}

int64_t ClockOffset::corrected_system_ns(int64_t steady_ns) const
{
  return steady_ns + offset_ns_;
}

int64_t ClockOffset::now_ns() const
{
  const auto steady_now = std::chrono::steady_clock::now();
  const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();
  return corrected_system_ns(steady_ns);
}

}  // namespace v4l2_publisher
```

---

## Task 3: V4L2Capture — ROS-agnostic V4L2 Backend

**Files:**
- Create: `include/v4l2_publisher/v4l2_capture.hpp`
- Create: `src/v4l2_capture.cpp`

**Interfaces:**
- Produces: `V4L2Config` struct, `V4L2Capture` class with `start()`, `stop()`, `grab() -> std::optional<cv::Mat>`, `frame_timestamp_ns() -> int64_t`, and query methods

- [ ] **Step 1: Write the header file**

Write `include/v4l2_publisher/v4l2_capture.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__V4L2_CAPTURE_HPP_
#define V4L2_PUBLISHER__V4L2_CAPTURE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace v4l2_publisher
{

struct V4L2Config
{
  std::string device_path;
  uint32_t requested_width  = 1280;
  uint32_t requested_height = 800;
  uint32_t target_width     = 640;
  uint32_t target_height    = 400;
  uint32_t pixel_format     = 0;  // V4L2_PIX_FMT_YUYV or similar, set by caller
  uint32_t buffer_count     = 4;
  double   requested_fps    = 60.0;
};

class V4L2Capture
{
public:
  explicit V4L2Capture(const V4L2Config & config);
  ~V4L2Capture();

  // Non-copyable, non-movable (owns mmap buffers and device fd)
  V4L2Capture(const V4L2Capture &) = delete;
  V4L2Capture & operator=(const V4L2Capture &) = delete;
  V4L2Capture(V4L2Capture &&) = delete;
  V4L2Capture & operator=(V4L2Capture &&) = delete;

  /// Start capture: open device, configure, mmap, stream on.
  void start();

  /// Stop capture: stream off, unmap, close.
  void stop();

  /// Block until a frame is available (2-second timeout), dequeue it,
  /// extract luminance, optionally resize. Returns the grayscale frame
  /// or std::nullopt on timeout/failure.
  /// After a successful return, frame_timestamp_ns() gives the raw
  /// V4L2 buffer timestamp in monotonic-clock nanoseconds.
  std::optional<cv::Mat> grab();

  /// Timestamp (monotonic-clock nanoseconds) of the last successfully
  /// grabbed frame, from the V4L2 buffer. Returns 0 if no frame yet.
  /// Callers apply ClockOffset::corrected_system_ns() to get system time.
  int64_t frame_timestamp_ns() const { return last_timestamp_ns_.load(); }

  // --- Query helpers ---
  uint32_t actual_width()    const { return actual_width_; }
  uint32_t actual_height()   const { return actual_height_; }
  double   actual_fps()      const { return actual_fps_; }
  uint32_t publish_width()   const { return publish_width_; }
  uint32_t publish_height()  const { return publish_height_; }
  const std::string & fourcc_string() const { return active_fourcc_string_; }

  struct Stats
  {
    uint64_t capture_count{0};
    uint64_t publish_count{0};
  };
  Stats snapshot_and_reset_stats();

private:
  void open_device();
  void configure_device();
  void init_mmap();
  void start_streaming();
  void stop_streaming();
  void unmap_buffers();
  void close_device();
  bool wait_for_frame();

  V4L2Config config_;
  int fd_{-1};
  std::vector<struct MmapBuffer> buffers_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> capture_count_{0};
  std::atomic<std::uint64_t> publish_count_{0};
  std::atomic<int64_t> last_timestamp_ns_{0};
  bool streaming_{false};
  bool uses_mplane_{false};
  uint32_t buffer_type_{0};
  uint32_t actual_width_{0};
  uint32_t actual_height_{0};
  double actual_fps_{0.0};
  uint32_t active_pixel_format_{0};
  uint32_t bytes_per_line_{0};
  std::string active_fourcc_string_{"????"};
  uint32_t publish_width_{0};
  uint32_t publish_height_{0};
  bool resize_to_target_{false};
  std::string publish_mode_{"unknown"};

  // Internal helper struct
  struct MmapBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };
};

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__V4L2_CAPTURE_HPP_
```

- [ ] **Step 2: Write the implementation**

Write `src/v4l2_capture.cpp`. This is the extracted `V4L2CameraPipeline` logic from `src/stereo_compressed_publisher.cpp`, with ROS dependencies removed — replace `RCLCPP_WARN`/`RCLCPP_INFO` with `fprintf(stderr, ...)`:

```cpp
#include "v4l2_publisher/v4l2_capture.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace v4l2_publisher
{

namespace
{

constexpr uint32_t kMaxPlanes = VIDEO_MAX_PLANES;

int xioctl(int fd, unsigned long request, void * arg)
{
  int result = 0;
  do {
    result = ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::string fourcc_to_string(uint32_t fourcc)
{
  std::ostringstream oss;
  oss
    << static_cast<char>(fourcc & 0xFF)
    << static_cast<char>((fourcc >> 8) & 0xFF)
    << static_cast<char>((fourcc >> 16) & 0xFF)
    << static_cast<char>((fourcc >> 24) & 0xFF);
  return oss.str();
}

bool has_same_aspect_ratio(
  uint32_t width_a, uint32_t height_a,
  uint32_t width_b, uint32_t height_b)
{
  if (height_a == 0 || height_b == 0) {
    return false;
  }
  const double ratio_a = static_cast<double>(width_a) / static_cast<double>(height_a);
  const double ratio_b = static_cast<double>(width_b) / static_cast<double>(height_b);
  constexpr double kAspectTolerance = 0.01;
  return std::abs(ratio_a - ratio_b) <= kAspectTolerance;
}

bool extract_luma_frame(
  uint32_t pixel_format,
  uint32_t width,
  uint32_t height,
  uint32_t bytes_per_line,
  const void * data,
  size_t bytes_used,
  cv::Mat & output)
{
  if (data == nullptr || width == 0 || height == 0) {
    return false;
  }

  switch (pixel_format) {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVYU:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_VYUY: {
      const size_t minimum_stride = static_cast<size_t>(width) * 2;
      const size_t stride = bytes_per_line != 0
        ? static_cast<size_t>(bytes_per_line) : minimum_stride;
      if (stride < minimum_stride || bytes_used < stride * static_cast<size_t>(height)) {
        return false;
      }

      output.create(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
      const auto * source = static_cast<const uint8_t *>(data);
      const int y_offset =
        (pixel_format == V4L2_PIX_FMT_YUYV || pixel_format == V4L2_PIX_FMT_YVYU) ? 0 : 1;

      for (uint32_t row = 0; row < height; ++row) {
        const uint8_t * src_row = source + row * stride;
        uint8_t * dst_row = output.ptr<uint8_t>(static_cast<int>(row));
        for (uint32_t col = 0; col < width; ++col) {
          dst_row[col] = src_row[col * 2 + y_offset];
        }
      }
      return true;
    }
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21: {
      const size_t minimum_stride = static_cast<size_t>(width);
      const size_t stride = bytes_per_line != 0
        ? static_cast<size_t>(bytes_per_line) : minimum_stride;
      if (stride < minimum_stride || bytes_used < stride * static_cast<size_t>(height)) {
        return false;
      }

      output.create(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
      const auto * source = static_cast<const uint8_t *>(data);
      for (uint32_t row = 0; row < height; ++row) {
        const uint8_t * src_row = source + row * stride;
        uint8_t * dst_row = output.ptr<uint8_t>(static_cast<int>(row));
        std::memcpy(dst_row, src_row, width);
      }
      return true;
    }
    default:
      return false;
  }
}

std::string fps_to_string(double fps)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(1);
  oss << fps;
  return oss.str();
}

}  // anonymous namespace

// ================================================================
// V4L2Capture implementation
// ================================================================

V4L2Capture::V4L2Capture(const V4L2Config & config)
: config_(config)
{
}

V4L2Capture::~V4L2Capture()
{
  stop();
}

void V4L2Capture::start()
{
  open_device();
  configure_device();
  init_mmap();
  start_streaming();
  running_.store(true);
}

void V4L2Capture::stop()
{
  running_.store(false);
  stop_streaming();
  unmap_buffers();
  close_device();
}

std::optional<cv::Mat> V4L2Capture::grab()
{
  if (!running_.load()) {
    return std::nullopt;
  }

  if (!wait_for_frame()) {
    return std::nullopt;
  }

  v4l2_buffer buffer{};
  v4l2_plane planes[kMaxPlanes]{};
  buffer.type = buffer_type_;
  buffer.memory = V4L2_MEMORY_MMAP;
  if (uses_mplane_) {
    buffer.length = kMaxPlanes;
    buffer.m.planes = planes;
  }
  if (xioctl(fd_, VIDIOC_DQBUF, &buffer) == -1) {
    if (errno == EAGAIN) {
      return std::nullopt;
    }
    fprintf(stderr, "[V4L2Capture] %s VIDIOC_DQBUF failed: %s\n",
            config_.device_path.c_str(), std::strerror(errno));
    return std::nullopt;
  }

  ++capture_count_;

  // Store raw monotonic timestamp from V4L2 buffer
  const auto buffer_timestamp_ns =
    static_cast<int64_t>(buffer.timestamp.tv_sec) * 1000000000LL +
    static_cast<int64_t>(buffer.timestamp.tv_usec) * 1000LL;
  last_timestamp_ns_.store(buffer_timestamp_ns);

  const void * frame_data = buffers_[buffer.index].start;
  size_t bytes_used = uses_mplane_
    ? static_cast<size_t>(buffer.m.planes[0].bytesused)
    : static_cast<size_t>(buffer.bytesused);

  cv::Mat gray_frame;
  const bool converted = extract_luma_frame(
    active_pixel_format_, actual_width_, actual_height_,
    bytes_per_line_, frame_data, bytes_used, gray_frame);

  // Requeue the buffer
  if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
    fprintf(stderr, "[V4L2Capture] %s VIDIOC_QBUF failed: %s\n",
            config_.device_path.c_str(), std::strerror(errno));
  }

  if (!converted) {
    fprintf(stderr, "[V4L2Capture] %s unsupported pixel format %s\n",
            config_.device_path.c_str(), active_fourcc_string_.c_str());
    return std::nullopt;
  }

  cv::Mat output_frame = gray_frame;
  if (resize_to_target_) {
    cv::Mat resized;
    cv::resize(gray_frame, resized,
               cv::Size(static_cast<int>(config_.target_width),
                        static_cast<int>(config_.target_height)),
               0.0, 0.0, cv::INTER_LINEAR);
    output_frame = resized;
  }

  ++publish_count_;
  return output_frame;
}

V4L2Capture::Stats V4L2Capture::snapshot_and_reset_stats()
{
  Stats stats;
  stats.capture_count = capture_count_.exchange(0);
  stats.publish_count = publish_count_.exchange(0);
  return stats;
}

void V4L2Capture::open_device()
{
  if (!std::filesystem::exists(config_.device_path)) {
    throw std::runtime_error("Device does not exist: " + config_.device_path);
  }

  fd_ = open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd_ < 0) {
    throw std::runtime_error(
      "Failed to open device " + config_.device_path + ": " + std::strerror(errno));
  }

  v4l2_capability caps{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &caps) == -1) {
    throw std::runtime_error(
      "VIDIOC_QUERYCAP failed for " + config_.device_path + ": " + std::strerror(errno));
  }
  if ((caps.capabilities & V4L2_CAP_STREAMING) == 0) {
    throw std::runtime_error(config_.device_path + " does not support streaming I/O");
  }

  if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0) {
    buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    uses_mplane_ = true;
  } else if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0) {
    buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uses_mplane_ = false;
  } else {
    throw std::runtime_error(
      config_.device_path + " does not expose a supported V4L2 capture capability");
  }
}

void V4L2Capture::configure_device()
{
  v4l2_format format{};
  format.type = buffer_type_;
  if (uses_mplane_) {
    format.fmt.pix_mp.width = config_.requested_width;
    format.fmt.pix_mp.height = config_.requested_height;
    format.fmt.pix_mp.pixelformat = config_.pixel_format;
    format.fmt.pix_mp.field = V4L2_FIELD_ANY;
    format.fmt.pix_mp.num_planes = 1;
  } else {
    format.fmt.pix.width = config_.requested_width;
    format.fmt.pix.height = config_.requested_height;
    format.fmt.pix.pixelformat = config_.pixel_format;
    format.fmt.pix.field = V4L2_FIELD_ANY;
  }

  if (xioctl(fd_, VIDIOC_S_FMT, &format) == -1) {
    throw std::runtime_error(
      "VIDIOC_S_FMT failed for " + config_.device_path + ": " + std::strerror(errno));
  }

  if (uses_mplane_) {
    actual_width_ = format.fmt.pix_mp.width;
    actual_height_ = format.fmt.pix_mp.height;
    active_pixel_format_ = format.fmt.pix_mp.pixelformat;
    bytes_per_line_ = format.fmt.pix_mp.plane_fmt[0].bytesperline;
  } else {
    actual_width_ = format.fmt.pix.width;
    actual_height_ = format.fmt.pix.height;
    active_pixel_format_ = format.fmt.pix.pixelformat;
    bytes_per_line_ = format.fmt.pix.bytesperline;
  }
  active_fourcc_string_ = fourcc_to_string(active_pixel_format_);

  // Configure FPS
  v4l2_streamparm streamparm{};
  streamparm.type = buffer_type_;
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator =
    static_cast<uint32_t>(config_.requested_fps);
  if (xioctl(fd_, VIDIOC_S_PARM, &streamparm) == -1) {
    fprintf(stderr, "[V4L2Capture] %s VIDIOC_S_PARM failed: %s\n",
            config_.device_path.c_str(), std::strerror(errno));
  }

  std::memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = buffer_type_;
  if (xioctl(fd_, VIDIOC_G_PARM, &streamparm) == -1) {
    actual_fps_ = 0.0;
  } else if (
    streamparm.parm.capture.timeperframe.numerator != 0 &&
    streamparm.parm.capture.timeperframe.denominator != 0)
  {
    actual_fps_ =
      static_cast<double>(streamparm.parm.capture.timeperframe.denominator) /
      static_cast<double>(streamparm.parm.capture.timeperframe.numerator);
  } else {
    actual_fps_ = 0.0;
  }

  // Determine publish dimensions
  if (actual_width_ == config_.target_width && actual_height_ == config_.target_height) {
    publish_width_ = actual_width_;
    publish_height_ = actual_height_;
    resize_to_target_ = false;
    publish_mode_ = "direct";
  } else if (has_same_aspect_ratio(
      actual_width_, actual_height_,
      config_.target_width, config_.target_height))
  {
    publish_width_ = config_.target_width;
    publish_height_ = config_.target_height;
    resize_to_target_ = true;
    publish_mode_ = "resize";
  } else {
    publish_width_ = actual_width_;
    publish_height_ = actual_height_;
    resize_to_target_ = false;
    publish_mode_ = "actual_no_crop";
  }

  fprintf(stderr, "[V4L2Capture] %s: fourcc=%s capture=%ux%u @ %s fps publish=%ux%u mode=%s io=%s\n",
          config_.device_path.c_str(),
          active_fourcc_string_.c_str(),
          actual_width_, actual_height_,
          fps_to_string(actual_fps_).c_str(),
          publish_width_, publish_height_,
          publish_mode_.c_str(),
          uses_mplane_ ? "mplane" : "single-plane");
}

void V4L2Capture::init_mmap()
{
  v4l2_requestbuffers request{};
  request.count = config_.buffer_count;
  request.type = buffer_type_;
  request.memory = V4L2_MEMORY_MMAP;

  if (xioctl(fd_, VIDIOC_REQBUFS, &request) == -1) {
    throw std::runtime_error(
      "VIDIOC_REQBUFS failed for " + config_.device_path + ": " + std::strerror(errno));
  }
  if (request.count < 2) {
    throw std::runtime_error(config_.device_path + " returned too few V4L2 mmap buffers");
  }

  buffers_.resize(request.count);
  for (uint32_t i = 0; i < request.count; ++i) {
    v4l2_buffer buffer{};
    v4l2_plane planes[kMaxPlanes]{};
    buffer.type = buffer_type_;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;
    if (uses_mplane_) {
      buffer.length = kMaxPlanes;
      buffer.m.planes = planes;
    }

    if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) == -1) {
      throw std::runtime_error(
        "VIDIOC_QUERYBUF failed for " + config_.device_path + ": " + std::strerror(errno));
    }

    size_t length = 0;
    off_t offset = 0;
    if (uses_mplane_) {
      if (buffer.length < 1) {
        throw std::runtime_error(config_.device_path + " returned zero planes for mplane buffer");
      }
      length = buffer.m.planes[0].length;
      offset = static_cast<off_t>(buffer.m.planes[0].m.mem_offset);
    } else {
      length = buffer.length;
      offset = static_cast<off_t>(buffer.m.offset);
    }

    MmapBuffer mb;
    mb.length = length;
    mb.start = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, offset);
    if (mb.start == MAP_FAILED) {
      mb.start = nullptr;
      throw std::runtime_error(
        "mmap failed for " + config_.device_path + ": " + std::strerror(errno));
    }
    buffers_[i] = mb;

    if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
      throw std::runtime_error(
        "VIDIOC_QBUF failed for " + config_.device_path + ": " + std::strerror(errno));
    }
  }
}

void V4L2Capture::start_streaming()
{
  auto type = buffer_type_;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    throw std::runtime_error(
      "VIDIOC_STREAMON failed for " + config_.device_path + ": " + std::strerror(errno));
  }
  streaming_ = true;
}

void V4L2Capture::stop_streaming()
{
  if (!streaming_ || fd_ < 0) {
    return;
  }
  auto type = buffer_type_;
  if (xioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
    fprintf(stderr, "[V4L2Capture] %s VIDIOC_STREAMOFF failed: %s\n",
            config_.device_path.c_str(), std::strerror(errno));
  }
  streaming_ = false;
}

void V4L2Capture::unmap_buffers()
{
  for (auto & buf : buffers_) {
    if (buf.start != nullptr && buf.length > 0) {
      munmap(buf.start, buf.length);
      buf.start = nullptr;
      buf.length = 0;
    }
  }
  buffers_.clear();
}

void V4L2Capture::close_device()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool V4L2Capture::wait_for_frame()
{
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  timeval timeout{};
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  const int result = select(fd_ + 1, &fds, nullptr, nullptr, &timeout);
  if (result == -1) {
    if (errno == EINTR) {
      return false;
    }
    throw std::runtime_error(
      "select failed for " + config_.device_path + ": " + std::strerror(errno));
  }
  if (result == 0) {
    fprintf(stderr, "[V4L2Capture] %s timed out waiting for frame\n",
            config_.device_path.c_str());
    return false;
  }
  return true;
}

}  // namespace v4l2_publisher
```

- [ ] **Step 3: Verify compilation of the library**

Not compilable standalone yet — will be verified once CMakeLists.txt is updated in Task 10.

---

## Task 4: Rectification Utility

**Files:**
- Create: `include/v4l2_publisher/rectification.hpp`
- Create: `src/rectification.cpp`

**Interfaces:**
- Produces: `RectificationMaps` struct, `load_rectification_maps(const std::string & yaml_path, const std::string & prefix) -> RectificationMaps`

- [ ] **Step 1: Write the header**

Write `include/v4l2_publisher/rectification.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__RECTIFICATION_HPP_
#define V4L2_PUBLISHER__RECTIFICATION_HPP_

#include <string>

#include <opencv2/core.hpp>

namespace v4l2_publisher
{

struct RectificationMaps
{
  cv::Mat map1;
  cv::Mat map2;
};

/// Load rectification maps from an OpenCV YAML calibration file.
/// prefix is "left" or "right" to match map1_left/map2_left or map1_right/map2_right.
RectificationMaps load_rectification_maps(const std::string & yaml_path,
                                          const std::string & prefix);

/// Validate maps for a given output size. Throws std::runtime_error on mismatch.
void validate_rectification_maps(const RectificationMaps & maps,
                                 const std::string & name,
                                 uint32_t width,
                                 uint32_t height);

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__RECTIFICATION_HPP_
```

- [ ] **Step 2: Write the implementation**

Write `src/rectification.cpp`:

```cpp
#include "v4l2_publisher/rectification.hpp"

#include <stdexcept>

#include <opencv2/core/persistence.hpp>

namespace v4l2_publisher
{

RectificationMaps load_rectification_maps(const std::string & yaml_path,
                                          const std::string & prefix)
{
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open calibration file: " + yaml_path);
  }

  const std::string key_map1 = "map1_" + prefix;
  const std::string key_map2 = "map2_" + prefix;

  RectificationMaps maps;
  fs[key_map1] >> maps.map1;
  fs[key_map2] >> maps.map2;

  if (maps.map1.empty() || maps.map2.empty()) {
    throw std::runtime_error(
      "Missing rectification map(s) " + key_map1 + "/" + key_map2 +
      " in " + yaml_path);
  }
  if (maps.map1.size() != maps.map2.size()) {
    throw std::runtime_error(
      "Rectification maps for '" + prefix + "' have mismatched sizes");
  }
  if (maps.map1.type() != CV_32FC1 || maps.map2.type() != CV_32FC1) {
    throw std::runtime_error(
      "Rectification maps for '" + prefix + "' must be CV_32FC1");
  }

  return maps;
}

void validate_rectification_maps(const RectificationMaps & maps,
                                 const std::string & name,
                                 uint32_t width,
                                 uint32_t height)
{
  const cv::Size expected(static_cast<int>(width), static_cast<int>(height));
  if (maps.map1.size() != expected || maps.map2.size() != expected) {
    throw std::runtime_error(
      "Rectification maps for '" + name + "' have size " +
      std::to_string(maps.map1.cols) + "x" + std::to_string(maps.map1.rows) +
      " but publish output is " + std::to_string(width) + "x" + std::to_string(height));
  }
}

}  // namespace v4l2_publisher
```

---

## Task 5: ImuDevice + ImuIcm45686

**Files:**
- Create: `include/v4l2_publisher/imu_device.hpp`
- Create: `include/v4l2_publisher/imu_icm_45686.hpp`
- Create: `src/imu_device.cpp`
- Create: `src/imu_icm_45686.cpp`

**Interfaces:**
- Produces: `ImuDevice` abstract class, `ImuIcm45686` concrete class
- Consumes: `i2c_hal.h`, `i2c_drive_linux.h`, `imu_core.h` from Task 1

- [ ] **Step 1: Write ImuDevice abstract interface**

Write `include/v4l2_publisher/imu_device.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__IMU_DEVICE_HPP_
#define V4L2_PUBLISHER__IMU_DEVICE_HPP_

namespace v4l2_publisher
{

/// Abstract IMU device interface.
/// New IMU chips are supported by implementing this interface.
class ImuDevice
{
public:
  virtual ~ImuDevice() = default;

  /// Read accelerometer data in g.
  /// Returns true on success.
  virtual bool read_accel(float & x, float & y, float & z) = 0;

  /// Read gyroscope data in degrees per second.
  /// Returns true on success.
  virtual bool read_gyro(float & x, float & y, float & z) = 0;

  /// Read temperature in degrees Celsius.
  /// Returns true on success.
  virtual bool read_temp(float & temp) = 0;
};

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__IMU_DEVICE_HPP_
```

- [ ] **Step 2: Write ImuIcm45686 header**

Write `include/v4l2_publisher/imu_icm_45686.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__IMU_ICM_45686_HPP_
#define V4L2_PUBLISHER__IMU_ICM_45686_HPP_

#include <cstdint>
#include <string>

#include "v4l2_publisher/imu_device.hpp"

// Forward declarations for C types
struct hal_i2c;
struct imu_device;

namespace v4l2_publisher
{

/// ICM-45686 IMU driver.
/// Wraps the C HAL (i2c_drive_linux) and chip-specific register logic.
class ImuIcm45686 : public ImuDevice
{
public:
  /// Construct and initialize the IMU.
  /// @param i2c_device_path  e.g. "/dev/i2c-3"
  /// @param i2c_addr         7-bit I2C address (e.g. 0x69)
  /// @throws std::runtime_error if I2C init or chip init fails
  ImuIcm45686(const std::string & i2c_device_path, uint8_t i2c_addr);
  ~ImuIcm45686() override;

  ImuIcm45686(const ImuIcm45686 &) = delete;
  ImuIcm45686 & operator=(const ImuIcm45686 &) = delete;
  ImuIcm45686(ImuIcm45686 &&) = delete;
  ImuIcm45686 & operator=(ImuIcm45686 &&) = delete;

  bool read_accel(float & x, float & y, float & z) override;
  bool read_gyro(float & x, float & y, float & z) override;
  bool read_temp(float & temp) override;

private:
  struct hal_i2c i2c_bus_;
  struct imu_device * dev_{nullptr};
  float accel_scale_{4096.0f};
  float gyro_scale_{16.4f};
};

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__IMU_ICM_45686_HPP_
```

- [ ] **Step 3: Write ImuDevice .cpp (empty base, just ensures the header is self-contained)**

Write `src/imu_device.cpp`:

```cpp
#include "v4l2_publisher/imu_device.hpp"

// ImuDevice is abstract; no implementation needed.
// This file exists to ensure the header is self-contained.
```

- [ ] **Step 4: Write ImuIcm45686 implementation**

Write `src/imu_icm_45686.cpp`:

```cpp
#include "v4l2_publisher/imu_icm_45686.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

extern "C" {
#include "i2c_hal.h"
#include "i2c_drive_linux.h"
#include "imu_core.h"
}

namespace v4l2_publisher
{

// ICM-45686 register definitions (absorbed from imu_icm_45686.c)
#define IMU_ICM_45686_PWR_MGMT0         0x10
#define IMU_ICM_45686_ACCEL_CONFIG0     0x1B
#define IMU_ICM_45686_GYRO_CONFIG0      0x1C
#define IMU_ICM_45686_ACCEL_DATA_X1_UI  0x00
#define IMU_ICM_45686_GYRO_DATA_X1_UI   0x06
#define IMU_ICM_45686_TEMP_DATA1_UI     0x0C

// Low-level I2C read helper using the HAL vtable
static int icm_read_regs(struct imu_device * dev, uint8_t reg, uint8_t * buf, uint16_t len)
{
  return dev->i2c_bus->transfer(dev->i2c_bus, dev->i2c_addr, &reg, 1, buf, len);
}

// Low-level I2C write helper using the HAL vtable
static int icm_write_reg(struct imu_device * dev, uint8_t reg, uint8_t val)
{
  uint8_t buf_w[2] = {reg, val};
  return dev->i2c_bus->transfer(dev->i2c_bus, dev->i2c_addr, buf_w, 2, nullptr, 0);
}

ImuIcm45686::ImuIcm45686(const std::string & i2c_device_path, uint8_t i2c_addr)
{
  std::memset(&i2c_bus_, 0, sizeof(i2c_bus_));

  // Initialize the I2C bus (file descriptor kept open for lifetime)
  if (hal_i2c_linux_init(&i2c_bus_, i2c_device_path.c_str()) < 0) {
    throw std::runtime_error("Failed to init I2C bus on " + i2c_device_path);
  }

  // Create and initialize the IMU device
  dev_ = imu_icm_45686_create(&i2c_bus_, i2c_addr);
  if (!dev_) {
    throw std::runtime_error(
      "Failed to create IMU ICM-45686 at address 0x" +
      std::to_string(i2c_addr));
  }

  fprintf(stderr, "[ImuIcm45686] Initialized on %s at 0x%02X\n",
          i2c_device_path.c_str(), i2c_addr);
}

ImuIcm45686::~ImuIcm45686()
{
  if (dev_) {
    imu_icm_45686_destroy(dev_);
    dev_ = nullptr;
  }
  // No explicit close for i2c_bus_ — fd was opened in hal_i2c_linux_init
  // and stored in i2c_bus_.bus_handle. The HAL does not expose a close;
  // the fd is closed when the process exits.
}

bool ImuIcm45686::read_accel(float & x, float & y, float & z)
{
  uint8_t buf[6];
  if (icm_read_regs(dev_, IMU_ICM_45686_ACCEL_DATA_X1_UI, buf, 6) < 0) {
    return false;
  }

  int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

  x = raw_x / accel_scale_;    // result in g
  y = raw_y / accel_scale_;
  z = raw_z / accel_scale_;
  return true;
}

bool ImuIcm45686::read_gyro(float & x, float & y, float & z)
{
  uint8_t buf[6];
  if (icm_read_regs(dev_, IMU_ICM_45686_GYRO_DATA_X1_UI, buf, 6) < 0) {
    return false;
  }

  int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

  x = raw_x / gyro_scale_;    // result in deg/s
  y = raw_y / gyro_scale_;
  z = raw_z / gyro_scale_;
  return true;
}

bool ImuIcm45686::read_temp(float & temp)
{
  uint8_t buf[2];
  if (icm_read_regs(dev_, IMU_ICM_45686_TEMP_DATA1_UI, buf, 2) < 0) {
    return false;
  }

  int16_t raw_temp = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  temp = (raw_temp / 128.0f) + 25.0f;
  return true;
}

}  // namespace v4l2_publisher
```

---

## Task 6: SingleCameraNode (composable)

**Files:**
- Create: `include/v4l2_publisher/single_camera_node.hpp`
- Create: `src/single_camera_node.cpp`

**Interfaces:**
- Produces: `v4l2_publisher::SingleCameraNode` (composable rclcpp::Node)
- Consumes: `V4L2Capture`, `ClockOffset`, `RectificationMaps`

- [ ] **Step 1: Write the public header**

Write `include/v4l2_publisher/single_camera_node.hpp`:

Write `include/v4l2_publisher/single_camera_node.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__SINGLE_CAMERA_NODE_HPP_
#define V4L2_PUBLISHER__SINGLE_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace v4l2_publisher
{
class SingleCameraNode : public rclcpp::Node
{
public:
  explicit SingleCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SingleCameraNode() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__SINGLE_CAMERA_NODE_HPP_
```

- [ ] **Step 2: Write the node implementation**

Write `src/single_camera_node.cpp`:

```cpp
#include <chrono>
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "v4l2_publisher/clock_offset.hpp"
#include "v4l2_publisher/rectification.hpp"
#include "v4l2_publisher/v4l2_capture.hpp"

namespace v4l2_publisher
{

// Helper: convert a fourcc string like "YUYV" to a V4L2_PIX_FMT_* constant
static uint32_t fourcc_from_string(const std::string & s)
{
  if (s.size() != 4) {
    return V4L2_PIX_FMT_YUYV;
  }
  return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

// Helper: copy cv::Mat to sensor_msgs::msg::Image
static void copy_mat_to_image(const cv::Mat & frame, sensor_msgs::msg::Image & msg)
{
  msg.height = static_cast<uint32_t>(frame.rows);
  msg.width = static_cast<uint32_t>(frame.cols);
  msg.encoding = "mono8";
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);

  const size_t data_size = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
  msg.data.resize(data_size);
  if (frame.isContinuous()) {
    std::memcpy(msg.data.data(), frame.data, data_size);
    return;
  }
  for (int row = 0; row < frame.rows; ++row) {
    std::memcpy(
      msg.data.data() + static_cast<size_t>(row) * frame.step,
      frame.ptr(row),
      frame.step);
  }
}

class SingleCameraNode : public rclcpp::Node
{
public:
  SingleCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("single_camera_publisher", options)
  {
    // Declare parameters
    device_path_ = declare_parameter<std::string>("device_path", "/dev/video0");
    const int req_w = declare_parameter<int>("requested_width", 1280);
    const int req_h = declare_parameter<int>("requested_height", 800);
    target_width_ = static_cast<uint32_t>(declare_parameter<int>("target_width", 640));
    target_height_ = static_cast<uint32_t>(declare_parameter<int>("target_height", 400));
    const std::string fourcc_str = declare_parameter<std::string>("pixel_format", "YUYV");
    const double fps = declare_parameter<double>("fps", 60.0);
    const std::string topic = declare_parameter<std::string>("topic_name", "/image");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    const bool rect_enable = declare_parameter<bool>("publish_rectified", false);
    const std::string calib_path = declare_parameter<std::string>("calib_path", "./camera_calib.yaml");

    // Configure and start V4L2 capture
    V4L2Config config;
    config.device_path = device_path_;
    config.requested_width = static_cast<uint32_t>(req_w);
    config.requested_height = static_cast<uint32_t>(req_h);
    config.target_width = target_width_;
    config.target_height = target_height_;
    config.pixel_format = fourcc_from_string(fourcc_str);
    config.requested_fps = fps;

    capture_ = std::make_unique<V4L2Capture>(config);
    capture_->start();

    // Create publisher
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(topic, 10);

    // Optional rectified publisher
    if (rect_enable) {
      rect_maps_ = load_rectification_maps(calib_path, "left");
      validate_rectification_maps(*rect_maps_, "camera",
                                  capture_->publish_width(),
                                  capture_->publish_height());
      rect_pub_ = create_publisher<sensor_msgs::msg::Image>(topic + "_rect", 10);
    }

    // Log setup
    RCLCPP_INFO(get_logger(),
      "Single camera publisher: device=%s capture=%ux%u publish=%ux%u fps=%.1f topic=%s",
      device_path_.c_str(),
      capture_->actual_width(), capture_->actual_height(),
      capture_->publish_width(), capture_->publish_height(),
      capture_->actual_fps(), topic.c_str());

    // Stats timer (also recalibrates clock offset)
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        clock_offset_.calibrate();
        const auto stats = capture_->snapshot_and_reset_stats();
        RCLCPP_INFO(get_logger(),
          "capture=%.2f publish=%.2f fps output=%ux%u capture=%ux%u fourcc=%s",
          static_cast<double>(stats.capture_count),
          static_cast<double>(stats.publish_count),
          capture_->publish_width(), capture_->publish_height(),
          capture_->actual_width(), capture_->actual_height(),
          capture_->fourcc_string().c_str());
      });

    // Main capture timer — pull model, runs at requested FPS
    const auto period = std::chrono::duration<double>(1.0 / fps);
    capture_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SingleCameraNode::capture_and_publish, this));
  }

  ~SingleCameraNode() override
  {
    if (capture_) {
      capture_->stop();
    }
  }

private:
  void capture_and_publish()
  {
    auto frame = capture_->grab();
    if (!frame.has_value()) {
      return;
    }

    // Convert monotonic V4L2 timestamp to system-clock nanoseconds
    const int64_t monotonic_ns = capture_->frame_timestamp_ns();
    const int64_t system_ns = clock_offset_.corrected_system_ns(monotonic_ns);
    const auto ros_time = rclcpp::Time(system_ns, RCL_SYSTEM_TIME);

    sensor_msgs::msg::Image msg;
    msg.header.stamp = ros_time;
    msg.header.frame_id = frame_id_;
    copy_mat_to_image(*frame, msg);

    const auto header = msg.header;
    image_pub_->publish(std::move(msg));

    if (rect_pub_ && rect_maps_) {
      cv::Mat rectified;
      cv::remap(*frame, rectified, rect_maps_->map1, rect_maps_->map2, cv::INTER_LINEAR);

      sensor_msgs::msg::Image rect_msg;
      rect_msg.header = header;
      copy_mat_to_image(rectified, rect_msg);
      rect_pub_->publish(std::move(rect_msg));
    }
  }

  std::string device_path_;
  std::string frame_id_{"camera_link"};
  uint32_t target_width_{640};
  uint32_t target_height_{400};
  std::unique_ptr<V4L2Capture> capture_;
  ClockOffset clock_offset_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  std::optional<RectificationMaps> rect_maps_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
};

}  // namespace v4l2_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_publisher::SingleCameraNode)
```

---

## Task 7: StereoCameraNode (composable)

**Files:**
- Create: `include/v4l2_publisher/stereo_camera_node.hpp`
- Create: `src/stereo_camera_node.cpp`

**Interfaces:**
- Produces: `v4l2_publisher::StereoCameraNode` (composable rclcpp::Node)
- Consumes: `V4L2Capture`, `ClockOffset`, `RectificationMaps`

- [ ] **Step 1: Write the public header**

Write `include/v4l2_publisher/stereo_camera_node.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__STEREO_CAMERA_NODE_HPP_
#define V4L2_PUBLISHER__STEREO_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace v4l2_publisher
{
class StereoCameraNode : public rclcpp::Node
{
public:
  explicit StereoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoCameraNode() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__STEREO_CAMERA_NODE_HPP_
```

- [ ] **Step 2: Write the node implementation**

Write `src/stereo_camera_node.cpp`:

```cpp
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "v4l2_publisher/clock_offset.hpp"
#include "v4l2_publisher/rectification.hpp"
#include "v4l2_publisher/v4l2_capture.hpp"

namespace v4l2_publisher
{

namespace
{

uint32_t fourcc_from_string(const std::string & s)
{
  if (s.size() != 4) return V4L2_PIX_FMT_YUYV;
  return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

void copy_mat_to_image(const cv::Mat & frame, sensor_msgs::msg::Image & msg)
{
  msg.height = static_cast<uint32_t>(frame.rows);
  msg.width = static_cast<uint32_t>(frame.cols);
  msg.encoding = "mono8";
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  const size_t data_size = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
  msg.data.resize(data_size);
  if (frame.isContinuous()) {
    std::memcpy(msg.data.data(), frame.data, data_size);
    return;
  }
  for (int row = 0; row < frame.rows; ++row) {
    std::memcpy(
      msg.data.data() + static_cast<size_t>(row) * frame.step,
      frame.ptr(row), frame.step);
  }
}

V4L2Config make_config(rclcpp::Node & node, const std::string & prefix)
{
  V4L2Config config;
  config.device_path = node.declare_parameter<std::string>(
    prefix + "device_path", prefix == "left_" ? "/dev/video0" : "/dev/video1");
  config.requested_width = static_cast<uint32_t>(
    node.declare_parameter<int>(prefix + "requested_width", 1280));
  config.requested_height = static_cast<uint32_t>(
    node.declare_parameter<int>(prefix + "requested_height", 800));
  config.target_width = static_cast<uint32_t>(
    node.declare_parameter<int>(prefix + "target_width", 640));
  config.target_height = static_cast<uint32_t>(
    node.declare_parameter<int>(prefix + "target_height", 400));
  config.pixel_format = fourcc_from_string(
    node.declare_parameter<std::string>(prefix + "pixel_format", "YUYV"));
  config.requested_fps = node.declare_parameter<double>(prefix + "fps", 60.0);
  return config;
}

}  // anonymous namespace

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stereo_camera_publisher", options)
  {
    V4L2Config left_config = make_config(*this, "left_");
    V4L2Config right_config = make_config(*this, "right_");

    left_topic_ = declare_parameter<std::string>("left_topic_name", "/image/left");
    right_topic_ = declare_parameter<std::string>("right_topic_name", "/image/right");
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "left_camera_link");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "right_camera_link");
    const bool rect_enable = declare_parameter<bool>("publish_rectified", false);
    const std::string stereo_calib = declare_parameter<std::string>("stereo_calib_path", "./stereo_calib.yaml");

    left_cam_ = std::make_unique<V4L2Capture>(left_config);
    right_cam_ = std::make_unique<V4L2Capture>(right_config);

    try {
      left_cam_->start();
      right_cam_->start();
    } catch (...) {
      left_cam_->stop();
      right_cam_->stop();
      throw;
    }

    left_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_, 10);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_, 10);

    if (rect_enable) {
      left_rect_maps_ = load_rectification_maps(stereo_calib, "left");
      right_rect_maps_ = load_rectification_maps(stereo_calib, "right");
      validate_rectification_maps(*left_rect_maps_, "left",
        left_cam_->publish_width(), left_cam_->publish_height());
      validate_rectification_maps(*right_rect_maps_, "right",
        right_cam_->publish_width(), right_cam_->publish_height());
      left_rect_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_ + "_rect", 10);
      right_rect_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_ + "_rect", 10);
    }

    RCLCPP_INFO(get_logger(),
      "left: capture=%ux%u publish=%ux%u fourcc=%s | right: capture=%ux%u publish=%ux%u fourcc=%s",
      left_cam_->actual_width(), left_cam_->actual_height(),
      left_cam_->publish_width(), left_cam_->publish_height(),
      left_cam_->fourcc_string().c_str(),
      right_cam_->actual_width(), right_cam_->actual_height(),
      right_cam_->publish_width(), right_cam_->publish_height(),
      right_cam_->fourcc_string().c_str());

    last_stats_time_ = now();
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StereoCameraNode::log_stats, this));

    // Capture timer — pull model at FPS
    const double fps = std::max(left_config.requested_fps, right_config.requested_fps);
    const auto period = std::chrono::duration<double>(1.0 / fps);
    capture_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StereoCameraNode::capture_and_publish, this));
  }

  ~StereoCameraNode() override
  {
    if (left_cam_) left_cam_->stop();
    if (right_cam_) right_cam_->stop();
  }

private:
  void capture_and_publish()
  {
    publish_one(left_cam_.get(), left_pub_, left_rect_pub_,
                left_rect_maps_, left_frame_id_);
    publish_one(right_cam_.get(), right_pub_, right_rect_pub_,
                right_rect_maps_, right_frame_id_);
  }

  void publish_one(V4L2Capture * cam,
                   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
                   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & rect_pub,
                   const std::optional<RectificationMaps> & rect_maps,
                   const std::string & frame_id)
  {
    auto frame = cam->grab();
    if (!frame.has_value()) {
      return;
    }

    const int64_t monotonic_ns = cam->frame_timestamp_ns();
    const int64_t system_ns = clock_offset_.corrected_system_ns(monotonic_ns);
    const auto ros_time = rclcpp::Time(system_ns, RCL_SYSTEM_TIME);

    sensor_msgs::msg::Image msg;
    msg.header.stamp = ros_time;
    msg.header.frame_id = frame_id;
    copy_mat_to_image(*frame, msg);

    const auto header = msg.header;
    pub->publish(std::move(msg));

    if (rect_pub && rect_maps) {
      cv::Mat rectified;
      cv::remap(*frame, rectified, rect_maps->map1, rect_maps->map2, cv::INTER_LINEAR);

      sensor_msgs::msg::Image rect_msg;
      rect_msg.header = header;
      copy_mat_to_image(rectified, rect_msg);
      rect_pub->publish(std::move(rect_msg));
    }
  }

  void log_stats()
  {
    const auto current = now();
    const double elapsed = (current - last_stats_time_).seconds();
    if (elapsed <= 0.0) return;

    const auto l_stats = left_cam_->snapshot_and_reset_stats();
    const auto r_stats = right_cam_->snapshot_and_reset_stats();

    RCLCPP_INFO(get_logger(),
      "left capture/pub=%.2f/%.2f fps %ux%u | right capture/pub=%.2f/%.2f fps %ux%u",
      l_stats.capture_count / elapsed, l_stats.publish_count / elapsed,
      left_cam_->publish_width(), left_cam_->publish_height(),
      r_stats.capture_count / elapsed, r_stats.publish_count / elapsed,
      right_cam_->publish_width(), right_cam_->publish_height());

    clock_offset_.calibrate();
    last_stats_time_ = current;
  }

  ClockOffset clock_offset_;
  std::string left_topic_;
  std::string right_topic_;
  std::string left_frame_id_{"left_camera_link"};
  std::string right_frame_id_{"right_camera_link"};
  std::unique_ptr<V4L2Capture> left_cam_;
  std::unique_ptr<V4L2Capture> right_cam_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_pub_;
  std::optional<RectificationMaps> left_rect_maps_;
  std::optional<RectificationMaps> right_rect_maps_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Time last_stats_time_;
};

}  // namespace v4l2_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_publisher::StereoCameraNode)
```

---

## Task 8: ImuNode (composable)

**Files:**
- Create: `include/v4l2_publisher/imu_node.hpp`
- Create: `src/imu_node.cpp`

**Interfaces:**
- Produces: `v4l2_publisher::ImuNode` (composable rclcpp::Node)
- Consumes: `ImuDevice` (ImuIcm45686), `ClockOffset`

- [ ] **Step 1: Write the public header**

Write `include/v4l2_publisher/imu_node.hpp`:

```cpp
#ifndef V4L2_PUBLISHER__IMU_NODE_HPP_
#define V4L2_PUBLISHER__IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace v4l2_publisher
{
class ImuNode : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImuNode() override = default;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__IMU_NODE_HPP_
```

- [ ] **Step 2: Write the IMU node implementation**

Write `src/imu_node.cpp`:

```cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "v4l2_publisher/clock_offset.hpp"
#include "v4l2_publisher/imu_device.hpp"
#include "v4l2_publisher/imu_icm_45686.hpp"

namespace v4l2_publisher
{

namespace
{
constexpr double kStandardGravity = 9.80665;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kMaxFrequency = 200.0;
}  // anonymous namespace

class ImuNode : public rclcpp::Node
{
public:
  ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("imu_publisher", options)
  {
    const std::string i2c_bus = declare_parameter<std::string>("i2c_bus", "/dev/i2c-3");
    const int imu_addr = declare_parameter<int>("imu_address", 0x69);
    double freq = declare_parameter<double>("frequency", 100.0);
    const std::string topic = declare_parameter<std::string>("topic_name", "imu/data");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    if (freq > kMaxFrequency) {
      RCLCPP_WARN(get_logger(),
        "Requested frequency %.1f Hz exceeds max 200 Hz; limiting to 200 Hz", freq);
      freq = kMaxFrequency;
    } else if (freq <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid frequency %.2f; using 10 Hz", freq);
      freq = 10.0;
    }

    // Initialize I2C and IMU device
    imu_ = std::make_unique<ImuIcm45686>(i2c_bus, static_cast<uint8_t>(imu_addr));

    // Create publisher
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topic, 10);

    // Recalibrate clock periodically
    clock_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { clock_offset_.calibrate(); });

    // Publish timer
    const auto period = std::chrono::duration<double>(1.0 / freq);
    pub_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ImuNode::publish_imu, this));

    RCLCPP_INFO(get_logger(),
      "IMU Publisher started: bus=%s addr=0x%02X topic=%s freq=%.1f Hz",
      i2c_bus.c_str(), imu_addr, topic.c_str(), freq);
  }

private:
  void publish_imu()
  {
    float ax, ay, az, gx, gy, gz;

    if (!imu_->read_accel(ax, ay, az) || !imu_->read_gyro(gx, gy, gz)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Failed to read IMU data");
      return;
    }

    auto msg = sensor_msgs::msg::Imu();

    // Offset-corrected timestamp
    const int64_t system_ns = clock_offset_.now_ns();
    msg.header.stamp = rclcpp::Time(system_ns, RCL_SYSTEM_TIME);
    msg.header.frame_id = frame_id_;

    // Accelerometer: g → m/s²
    msg.linear_acceleration.x = ax * kStandardGravity;
    msg.linear_acceleration.y = ay * kStandardGravity;
    msg.linear_acceleration.z = az * kStandardGravity;

    // Gyroscope: deg/s → rad/s
    msg.angular_velocity.x = gx * kDegToRad;
    msg.angular_velocity.y = gy * kDegToRad;
    msg.angular_velocity.z = gz * kDegToRad;

    // Set covariances to -1 (unknown) — caller can override
    msg.linear_acceleration_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.orientation_covariance[0] = -1.0;

    imu_pub_->publish(std::move(msg));
  }

  std::unique_ptr<ImuDevice> imu_;
  ClockOffset clock_offset_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  std::string frame_id_;
};

}  // namespace v4l2_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_publisher::ImuNode)
```

---

## Task 9: Standalone Entry Points

**Files:**
- Create: `src/single_camera_main.cpp`
- Create: `src/stereo_camera_main.cpp`
- Create: `src/imu_main.cpp`

**Interfaces:**
- Each `main.cpp` instantiates one node and calls `rclcpp::spin()`.
- The `_main.cpp` files link against the component shared library, so they just include the node header.

- [ ] **Step 1: Write single_camera_main.cpp**

Write `src/single_camera_main.cpp`:

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "v4l2_publisher/single_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<v4l2_publisher::SingleCameraNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("single_camera_publisher"), "%s", ex.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Write stereo_camera_main.cpp**

Write `src/stereo_camera_main.cpp`:

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "v4l2_publisher/stereo_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<v4l2_publisher::StereoCameraNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_camera_publisher"), "%s", ex.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Write imu_main.cpp**

Write `src/imu_main.cpp`:

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "v4l2_publisher/imu_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<v4l2_publisher::ImuNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_publisher"), "%s", ex.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

---

## Task 10: CMakeLists.txt and package.xml

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

- [ ] **Step 1: Write the new CMakeLists.txt**

Write `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(v4l2_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# --- Include directories ---
include_directories(include)
include_directories(src)   # for C HAL headers

# --- Shared library: V4L2 capture (ROS-agnostic) ---
add_library(v4l2_capture STATIC
  src/v4l2_capture.cpp
  src/clock_offset.cpp
  src/rectification.cpp
)
target_compile_features(v4l2_capture PUBLIC cxx_std_17)
target_include_directories(v4l2_capture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(v4l2_capture PUBLIC ${OpenCV_LIBS})

# --- Shared library: I2C IMU driver ---
add_library(i2c_imu STATIC
  src/i2c_drive_linux.c
  src/imu_device.cpp
  src/imu_icm_45686.cpp
)
target_compile_features(i2c_imu PUBLIC cxx_std_17)
target_include_directories(i2c_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# --- Node headers (for standalone executables and component consumers) ---
# Each node's header declares its class (subclass of rclcpp::Node)

# --- Single Camera Component ---
add_library(single_camera_component SHARED src/single_camera_node.cpp)
target_compile_features(single_camera_component PUBLIC cxx_std_17)
target_link_libraries(single_camera_component v4l2_capture)
target_include_directories(single_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(single_camera_component
  rclcpp rclcpp_components sensor_msgs OpenCV
)

# --- Stereo Camera Component ---
add_library(stereo_camera_component SHARED src/stereo_camera_node.cpp)
target_compile_features(stereo_camera_component PUBLIC cxx_std_17)
target_link_libraries(stereo_camera_component v4l2_capture)
target_include_directories(stereo_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(stereo_camera_component
  rclcpp rclcpp_components sensor_msgs OpenCV
)

# --- IMU Component ---
add_library(imu_component SHARED src/imu_node.cpp)
target_compile_features(imu_component PUBLIC cxx_std_17)
target_link_libraries(imu_component i2c_imu)
target_include_directories(imu_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(imu_component
  rclcpp rclcpp_components sensor_msgs
)

# --- Standalone executables ---
add_executable(single_camera_publisher src/single_camera_main.cpp)
target_link_libraries(single_camera_publisher single_camera_component)
target_include_directories(single_camera_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(stereo_camera_publisher src/stereo_camera_main.cpp)
target_link_libraries(stereo_camera_publisher stereo_camera_component)
target_include_directories(stereo_camera_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(imu_publisher src/imu_main.cpp)
target_link_libraries(imu_publisher imu_component)
target_include_directories(imu_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# --- Install targets ---
install(TARGETS
  v4l2_capture
  i2c_imu
  single_camera_component
  stereo_camera_component
  imu_component
  single_camera_publisher
  stereo_camera_publisher
  imu_publisher
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Component registration ---
rclcpp_components_register_node(single_camera_component
  PLUGIN "v4l2_publisher::SingleCameraNode"
  EXECUTABLE single_camera_publisher
)

rclcpp_components_register_node(stereo_camera_component
  PLUGIN "v4l2_publisher::StereoCameraNode"
  EXECUTABLE stereo_camera_publisher
)

rclcpp_components_register_node(imu_component
  PLUGIN "v4l2_publisher::ImuNode"
  EXECUTABLE imu_publisher
)

ament_package()
```

- [ ] **Step 2: Update package.xml**

Write `package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>v4l2_publisher</name>
  <version>0.0.1</version>
  <description>Unified camera (single/stereo V4L2) and IMU (I2C ICM-45686) publisher for ROS2.</description>
  <maintainer email="pi@example.com">pi</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>sensor_msgs</depend>
  <depend>launch</depend>
  <depend>launch_ros</depend>

  <build_depend>opencv</build_depend>
  <exec_depend>opencv</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Task 11: Launch Files

**Files:**
- Create: `launch/single_camera_with_imu.launch.py`
- Modify: `launch/stereo_camera_with_imu.launch.py`

- [ ] **Step 1: Write single_camera_with_imu.launch.py**

Write `launch/single_camera_with_imu.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Camera parameters ---
        DeclareLaunchArgument('device_path', default_value='/dev/video0'),
        DeclareLaunchArgument('requested_width', default_value='1280'),
        DeclareLaunchArgument('requested_height', default_value='800'),
        DeclareLaunchArgument('target_width', default_value='640'),
        DeclareLaunchArgument('target_height', default_value='400'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('fps', default_value='60.0'),
        DeclareLaunchArgument('topic_name', default_value='/image'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('publish_rectified', default_value='false'),
        DeclareLaunchArgument('calib_path', default_value='./camera_calib.yaml'),

        # --- IMU parameters ---
        DeclareLaunchArgument('i2c_bus', default_value='/dev/i2c-3'),
        DeclareLaunchArgument('imu_address', default_value='105'),
        DeclareLaunchArgument('imu_frequency', default_value='100.0'),
        DeclareLaunchArgument('imu_topic_name', default_value='imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),

        # --- Single camera node ---
        Node(
            package='v4l2_publisher',
            executable='single_camera_publisher',
            name='single_camera_publisher',
            output='screen',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'requested_width': LaunchConfiguration('requested_width'),
                'requested_height': LaunchConfiguration('requested_height'),
                'target_width': LaunchConfiguration('target_width'),
                'target_height': LaunchConfiguration('target_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'fps': LaunchConfiguration('fps'),
                'topic_name': LaunchConfiguration('topic_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rectified': LaunchConfiguration('publish_rectified'),
                'calib_path': LaunchConfiguration('calib_path'),
            }],
        ),

        # --- IMU node ---
        Node(
            package='v4l2_publisher',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'imu_address': LaunchConfiguration('imu_address'),
                'frequency': LaunchConfiguration('imu_frequency'),
                'topic_name': LaunchConfiguration('imu_topic_name'),
                'frame_id': LaunchConfiguration('imu_frame_id'),
            }],
        ),
    ])
```

- [ ] **Step 2: Rewrite stereo_camera_with_imu.launch.py**

Write `launch/stereo_camera_with_imu.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Left camera parameters ---
        DeclareLaunchArgument('left_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument('left_requested_width', default_value='1280'),
        DeclareLaunchArgument('left_requested_height', default_value='800'),
        DeclareLaunchArgument('left_target_width', default_value='640'),
        DeclareLaunchArgument('left_target_height', default_value='400'),
        DeclareLaunchArgument('left_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('left_fps', default_value='60.0'),
        DeclareLaunchArgument('left_topic_name', default_value='/image/left'),
        DeclareLaunchArgument('left_frame_id', default_value='left_camera_link'),

        # --- Right camera parameters ---
        DeclareLaunchArgument('right_device_path', default_value='/dev/video1'),
        DeclareLaunchArgument('right_requested_width', default_value='1280'),
        DeclareLaunchArgument('right_requested_height', default_value='800'),
        DeclareLaunchArgument('right_target_width', default_value='640'),
        DeclareLaunchArgument('right_target_height', default_value='400'),
        DeclareLaunchArgument('right_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('right_fps', default_value='60.0'),
        DeclareLaunchArgument('right_topic_name', default_value='/image/right'),
        DeclareLaunchArgument('right_frame_id', default_value='right_camera_link'),

        # --- Rectification ---
        DeclareLaunchArgument('publish_rectified', default_value='false'),
        DeclareLaunchArgument('stereo_calib_path', default_value='./stereo_calib.yaml'),

        # --- IMU parameters ---
        DeclareLaunchArgument('i2c_bus', default_value='/dev/i2c-3'),
        DeclareLaunchArgument('imu_address', default_value='105'),
        DeclareLaunchArgument('imu_frequency', default_value='100.0'),
        DeclareLaunchArgument('imu_topic_name', default_value='imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),

        # --- Stereo camera node ---
        Node(
            package='v4l2_publisher',
            executable='stereo_camera_publisher',
            name='stereo_camera_publisher',
            output='screen',
            parameters=[{
                'left_device_path': LaunchConfiguration('left_device_path'),
                'left_requested_width': LaunchConfiguration('left_requested_width'),
                'left_requested_height': LaunchConfiguration('left_requested_height'),
                'left_target_width': LaunchConfiguration('left_target_width'),
                'left_target_height': LaunchConfiguration('left_target_height'),
                'left_pixel_format': LaunchConfiguration('left_pixel_format'),
                'left_fps': LaunchConfiguration('left_fps'),
                'left_topic_name': LaunchConfiguration('left_topic_name'),
                'left_frame_id': LaunchConfiguration('left_frame_id'),
                'right_device_path': LaunchConfiguration('right_device_path'),
                'right_requested_width': LaunchConfiguration('right_requested_width'),
                'right_requested_height': LaunchConfiguration('right_requested_height'),
                'right_target_width': LaunchConfiguration('right_target_width'),
                'right_target_height': LaunchConfiguration('right_target_height'),
                'right_pixel_format': LaunchConfiguration('right_pixel_format'),
                'right_fps': LaunchConfiguration('right_fps'),
                'right_topic_name': LaunchConfiguration('right_topic_name'),
                'right_frame_id': LaunchConfiguration('right_frame_id'),
                'publish_rectified': LaunchConfiguration('publish_rectified'),
                'stereo_calib_path': LaunchConfiguration('stereo_calib_path'),
            }],
        ),

        # --- IMU node ---
        Node(
            package='v4l2_publisher',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'imu_address': LaunchConfiguration('imu_address'),
                'frequency': LaunchConfiguration('imu_frequency'),
                'topic_name': LaunchConfiguration('imu_topic_name'),
                'frame_id': LaunchConfiguration('imu_frame_id'),
            }],
        ),
    ])
```

---

## Task 12: Build and Verify

- [ ] **Step 1: Clean any prior build artifacts**

```bash
rm -rf /home/pi/workspace/alg_cam_rospub/src/build /home/pi/workspace/alg_cam_rospub/src/install /home/pi/workspace/alg_cam_rospub/src/log
```

- [ ] **Step 2: Build the package**

```bash
cd /home/pi/workspace/alg_cam_rospub && colcon build --packages-select v4l2_publisher
```
Expected: Build succeeds with no errors.

- [ ] **Step 3: Source the setup and verify executables exist**

```bash
source /home/pi/workspace/alg_cam_rospub/src/install/setup.bash
ros2 pkg executables v4l2_publisher
```
Expected output:
```
v4l2_publisher single_camera_publisher
v4l2_publisher stereo_camera_publisher
v4l2_publisher imu_publisher
```

- [ ] **Step 4: Verify launch files are parseable**

```bash
ros2 launch v4l2_publisher single_camera_with_imu.launch.py --show-arguments
ros2 launch v4l2_publisher stereo_camera_with_imu.launch.py --show-arguments
```
Expected: Lists launch arguments with defaults, no errors.

---

## Task 13: Cleanup — Remove Replaced Files

- [ ] **Step 1: Remove old stereo publisher source**

```bash
rm /home/pi/workspace/alg_cam_rospub/src/stereo_compressed_publisher.cpp
```

- [ ] **Step 2: Remove old Python IMU script**

```bash
rm /home/pi/workspace/alg_cam_rospub/scripts/start_imu_after_camera_ready.py
```

- [ ] **Step 3: Remove diagnostic probe (if present)**

```bash
rm -f /home/pi/workspace/alg_cam_rospub/src/camera_backend_probe.cpp
```

- [ ] **Step 4: Remove external IMU binary (no longer needed)**

```bash
rm -rf /home/pi/workspace/alg_cam_rospub/imu_ros2_pub_bin
```

- [ ] **Step 5: Remove Python cache files**

```bash
rm -rf /home/pi/workspace/alg_cam_rospub/scripts/__pycache__
```

- [ ] **Step 6: Rebuild to confirm cleanup didn't break anything**

```bash
cd /home/pi/workspace/alg_cam_rospub && colcon build --packages-select v4l2_publisher
```
Expected: Build succeeds.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: unified camera + IMU publisher in C++

Replace stereo publisher + Python IMU gate with three composable
C++ nodes (single_camera, stereo_camera, imu) sharing two libraries
(v4l2_capture, i2c_imu). All parameters configurable via launch args.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
