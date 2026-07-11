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
    fprintf(stderr, "[V4L2Capture] %s unsupported pixel format %s (code 0x%08x)\n",
            config_.device_path.c_str(), active_fourcc_string_.c_str(), active_pixel_format_);
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

  // Fallback: if we got here and the frame is empty (luma extraction succeeded
  // but produced no data), return nullopt.
  if (output_frame.empty()) {
    return std::nullopt;
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
