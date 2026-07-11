#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
constexpr uint32_t kTargetWidth = 640;
constexpr uint32_t kTargetHeight = 400;
constexpr uint32_t kRequestedWidth = 1280;
constexpr uint32_t kRequestedHeight = 800;
constexpr uint32_t kRequestedPixelFormat = V4L2_PIX_FMT_YUYV;
constexpr uint32_t kBufferCount = 4;
constexpr double kAspectTolerance = 0.01;
constexpr char kLeftDevicePath[] = "/dev/video0";
constexpr char kRightDevicePath[] = "/dev/video1";
constexpr char kLeftFrameId[] = "left_camera_link";
constexpr char kRightFrameId[] = "right_camera_link";
constexpr char kLeftTopic[] = "/image/left";
constexpr char kRightTopic[] = "/image/right";
constexpr char kLeftRectTopic[] = "/image/left_rect";
constexpr char kRightRectTopic[] = "/image/right_rect";
constexpr char kDefaultStereoCalibPath[] = "./stereo_calib.yaml";

struct MmapBuffer
{
  void * start{nullptr};
  size_t length{0};
};

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

bool has_same_aspect_ratio(uint32_t width_a, uint32_t height_a, uint32_t width_b, uint32_t height_b)
{
  if (height_a == 0 || height_b == 0) {
    return false;
  }

  const double ratio_a = static_cast<double>(width_a) / static_cast<double>(height_a);
  const double ratio_b = static_cast<double>(width_b) / static_cast<double>(height_b);
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
      const size_t stride = bytes_per_line != 0 ? static_cast<size_t>(bytes_per_line) : minimum_stride;
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
      const size_t stride = bytes_per_line != 0 ? static_cast<size_t>(bytes_per_line) : minimum_stride;
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

void copy_mat_to_image_message(const cv::Mat & frame, sensor_msgs::msg::Image & msg)
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

std::string fps_to_string(double fps)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(1);
  oss << fps;
  return oss.str();
}

struct RectificationMaps
{
  cv::Mat map1;
  cv::Mat map2;
};

class StereoRectifier
{
public:
  static StereoRectifier load(const std::string & yaml_path)
  {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      throw std::runtime_error("Failed to open stereo calibration file: " + yaml_path);
    }

    StereoRectifier rectifier;
    rectifier.left_maps_.map1 = fs["map1_left"].mat();
    rectifier.left_maps_.map2 = fs["map2_left"].mat();
    rectifier.right_maps_.map1 = fs["map1_right"].mat();
    rectifier.right_maps_.map2 = fs["map2_right"].mat();

    rectifier.validate_maps(rectifier.left_maps_, "left");
    rectifier.validate_maps(rectifier.right_maps_, "right");
    return rectifier;
  }

  const RectificationMaps & left_maps() const { return left_maps_; }
  const RectificationMaps & right_maps() const { return right_maps_; }

  static void validate_for_output_size(
    const RectificationMaps & maps,
    const std::string & camera_name,
    uint32_t width,
    uint32_t height)
  {
    const cv::Size expected_size(static_cast<int>(width), static_cast<int>(height));
    if (maps.map1.size() != expected_size || maps.map2.size() != expected_size) {
      throw std::runtime_error(
              "Rectification maps for " + camera_name + " camera have size " +
              std::to_string(maps.map1.cols) + "x" + std::to_string(maps.map1.rows) +
              " but publish output is " + std::to_string(width) + "x" + std::to_string(height));
    }
  }

private:
  static void validate_maps(const RectificationMaps & maps, const std::string & camera_name)
  {
    if (maps.map1.empty() || maps.map2.empty()) {
      throw std::runtime_error("Missing rectification map(s) for " + camera_name + " camera");
    }
    if (maps.map1.size() != maps.map2.size()) {
      throw std::runtime_error("Rectification maps for " + camera_name + " camera have mismatched sizes");
    }
    if (maps.map1.type() != CV_32FC1 || maps.map2.type() != CV_32FC1) {
      throw std::runtime_error(
              "Rectification maps for " + camera_name +
              " camera must be CV_32FC1 to match fisheye calibration output");
    }
  }

  RectificationMaps left_maps_;
  RectificationMaps right_maps_;
};
}  // namespace

class V4L2CameraPipeline
{
public:
  V4L2CameraPipeline(
    rclcpp::Node & node,
    std::string device_path,
    std::string camera_name,
    std::string frame_id,
    std::string topic_name,
    std::optional<std::string> rectified_topic_name = std::nullopt,
    std::optional<RectificationMaps> rectification_maps = std::nullopt)
  : node_(node),
    device_path_(std::move(device_path)),
    camera_name_(std::move(camera_name)),
    frame_id_(std::move(frame_id)),
    rectification_maps_(std::move(rectification_maps))
  {
    publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    if (rectified_topic_name.has_value()) {
      rectified_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(*rectified_topic_name, 10);
    }
  }

  ~V4L2CameraPipeline()
  {
    stop();
  }

  void start()
  {
    open_device();
    configure_device();
    init_mmap();
    start_streaming();
    running_.store(true);
    worker_thread_ = std::thread(&V4L2CameraPipeline::capture_loop, this);
  }

  void stop()
  {
    running_.store(false);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    stop_streaming();
    unmap_buffers();
    close_device();
  }

  std::pair<std::uint64_t, std::uint64_t> snapshot_and_reset_counts()
  {
    return {capture_count_.exchange(0), publish_count_.exchange(0)};
  }

  void validate_rectification_output() const
  {
    if (!rectification_maps_.has_value()) {
      return;
    }
    StereoRectifier::validate_for_output_size(
      *rectification_maps_, camera_name_, publish_width_, publish_height_);
  }

  uint32_t actual_width() const { return actual_width_; }
  uint32_t actual_height() const { return actual_height_; }
  double actual_fps() const { return actual_fps_; }
  uint32_t publish_width() const { return publish_width_; }
  uint32_t publish_height() const { return publish_height_; }
  const std::string & fourcc() const { return active_fourcc_string_; }

private:
  void open_device()
  {
    if (!std::filesystem::exists(device_path_)) {
      throw std::runtime_error("Device does not exist: " + device_path_);
    }

    fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open device " + device_path_ + ": " + std::strerror(errno));
    }

    v4l2_capability caps{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &caps) == -1) {
      throw std::runtime_error("VIDIOC_QUERYCAP failed for " + device_path_ + ": " + std::strerror(errno));
    }
    if ((caps.capabilities & V4L2_CAP_STREAMING) == 0) {
      throw std::runtime_error(device_path_ + " does not support streaming I/O");
    }

    if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0) {
      buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      uses_mplane_ = true;
    } else if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0) {
      buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      uses_mplane_ = false;
    } else {
      throw std::runtime_error(device_path_ + " does not expose a supported V4L2 capture capability");
    }
  }

  void configure_device()
  {
    v4l2_format format{};
    format.type = buffer_type_;
    if (uses_mplane_) {
      format.fmt.pix_mp.width = kRequestedWidth;
      format.fmt.pix_mp.height = kRequestedHeight;
      format.fmt.pix_mp.pixelformat = kRequestedPixelFormat;
      format.fmt.pix_mp.field = V4L2_FIELD_ANY;
      format.fmt.pix_mp.num_planes = 1;
    } else {
      format.fmt.pix.width = kRequestedWidth;
      format.fmt.pix.height = kRequestedHeight;
      format.fmt.pix.pixelformat = kRequestedPixelFormat;
      format.fmt.pix.field = V4L2_FIELD_ANY;
    }

    if (xioctl(fd_, VIDIOC_S_FMT, &format) == -1) {
      throw std::runtime_error("VIDIOC_S_FMT failed for " + device_path_ + ": " + std::strerror(errno));
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

    v4l2_streamparm streamparm{};
    streamparm.type = buffer_type_;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = 60;
    if (xioctl(fd_, VIDIOC_S_PARM, &streamparm) == -1) {
      RCLCPP_WARN(
        node_.get_logger(),
        "%s camera %s VIDIOC_S_PARM failed: %s",
        camera_name_.c_str(), device_path_.c_str(), std::strerror(errno));
    }

    std::memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = buffer_type_;
    if (xioctl(fd_, VIDIOC_G_PARM, &streamparm) == -1) {
      actual_fps_ = 0.0;
    } else if (
      streamparm.parm.capture.timeperframe.numerator != 0 &&
      streamparm.parm.capture.timeperframe.denominator != 0)
    {
      actual_fps_ = static_cast<double>(streamparm.parm.capture.timeperframe.denominator) /
        static_cast<double>(streamparm.parm.capture.timeperframe.numerator);
    } else {
      actual_fps_ = 0.0;
    }

    if (actual_width_ == kTargetWidth && actual_height_ == kTargetHeight) {
      publish_width_ = actual_width_;
      publish_height_ = actual_height_;
      resize_to_target_ = false;
      publish_mode_ = "direct_640x400";
    } else if (has_same_aspect_ratio(actual_width_, actual_height_, kTargetWidth, kTargetHeight)) {
      publish_width_ = kTargetWidth;
      publish_height_ = kTargetHeight;
      resize_to_target_ = true;
      publish_mode_ = "resize_to_640x400";
      if (actual_width_ != kRequestedWidth || actual_height_ != kRequestedHeight) {
        RCLCPP_WARN(
          node_.get_logger(),
          "%s camera %s requested %ux%u but got %ux%u; resizing to 640x400 without cropping to preserve FOV",
          camera_name_.c_str(), device_path_.c_str(), kRequestedWidth, kRequestedHeight, actual_width_, actual_height_);
      }
    } else {
      publish_width_ = actual_width_;
      publish_height_ = actual_height_;
      resize_to_target_ = false;
      publish_mode_ = "publish_actual_no_crop";
      RCLCPP_WARN(
        node_.get_logger(),
        "%s camera %s fell back to %ux%u with different aspect ratio; publishing actual frames to avoid FOV loss",
        camera_name_.c_str(), device_path_.c_str(), actual_width_, actual_height_);
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "%s camera %s active format: fourcc=%s capture=%ux%u @ %s fps publish=%ux%u mode=%s io=%s",
      camera_name_.c_str(),
      device_path_.c_str(),
      active_fourcc_string_.c_str(),
      actual_width_,
      actual_height_,
      fps_to_string(actual_fps_).c_str(),
      publish_width_,
      publish_height_,
      publish_mode_.c_str(),
      uses_mplane_ ? "mplane" : "single-plane");
  }

  void init_mmap()
  {
    v4l2_requestbuffers request{};
    request.count = kBufferCount;
    request.type = buffer_type_;
    request.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &request) == -1) {
      throw std::runtime_error("VIDIOC_REQBUFS failed for " + device_path_ + ": " + std::strerror(errno));
    }
    if (request.count < 2) {
      throw std::runtime_error(device_path_ + " returned too few V4L2 mmap buffers");
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
        throw std::runtime_error("VIDIOC_QUERYBUF failed for " + device_path_ + ": " + std::strerror(errno));
      }

      size_t length = 0;
      off_t offset = 0;
      if (uses_mplane_) {
        if (buffer.length < 1) {
          throw std::runtime_error(device_path_ + " returned zero planes for mplane buffer");
        }
        length = buffer.m.planes[0].length;
        offset = static_cast<off_t>(buffer.m.planes[0].m.mem_offset);
      } else {
        length = buffer.length;
        offset = static_cast<off_t>(buffer.m.offset);
      }

      buffers_[i].length = length;
      buffers_[i].start = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        throw std::runtime_error("mmap failed for " + device_path_ + ": " + std::strerror(errno));
      }

      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
        throw std::runtime_error("VIDIOC_QBUF failed for " + device_path_ + ": " + std::strerror(errno));
      }
    }
  }

  void start_streaming()
  {
    auto type = buffer_type_;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
      throw std::runtime_error("VIDIOC_STREAMON failed for " + device_path_ + ": " + std::strerror(errno));
    }
    streaming_ = true;
  }

  void stop_streaming()
  {
    if (!streaming_ || fd_ < 0) {
      return;
    }
    auto type = buffer_type_;
    if (xioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
      RCLCPP_WARN(
        node_.get_logger(),
        "%s camera %s VIDIOC_STREAMOFF failed: %s",
        camera_name_.c_str(), device_path_.c_str(), std::strerror(errno));
    }
    streaming_ = false;
  }

  void unmap_buffers()
  {
    for (auto & buffer : buffers_) {
      if (buffer.start != nullptr && buffer.length > 0) {
        munmap(buffer.start, buffer.length);
        buffer.start = nullptr;
        buffer.length = 0;
      }
    }
    buffers_.clear();
  }

  void close_device()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  bool wait_for_frame()
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
      throw std::runtime_error("select failed for " + device_path_ + ": " + std::strerror(errno));
    }
    if (result == 0) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "%s camera %s timed out waiting for frame",
        camera_name_.c_str(), device_path_.c_str());
      return false;
    }
    return true;
  }

  void capture_loop()
  {
    while (rclcpp::ok() && running_.load()) {
      if (!wait_for_frame()) {
        continue;
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
          continue;
        }
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 2000,
          "VIDIOC_DQBUF failed for %s camera %s: %s",
          camera_name_.c_str(), device_path_.c_str(), std::strerror(errno));
        continue;
      }

      ++capture_count_;

      const void * frame_data = buffers_[buffer.index].start;
      size_t bytes_used = uses_mplane_ ? static_cast<size_t>(buffer.m.planes[0].bytesused) : static_cast<size_t>(buffer.bytesused);

      cv::Mat gray_frame;
      const bool converted = extract_luma_frame(
        active_pixel_format_, actual_width_, actual_height_, bytes_per_line_, frame_data, bytes_used, gray_frame);
      if (!converted) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 2000,
          "%s camera %s unsupported or invalid pixel format %s (bytesperline=%u bytesused=%zu)",
          camera_name_.c_str(), device_path_.c_str(), active_fourcc_string_.c_str(), bytes_per_line_, bytes_used);
      } else {
        cv::Mat output_frame = gray_frame;
        cv::Mat resized_frame;
        if (resize_to_target_) {
          cv::resize(
            gray_frame,
            resized_frame,
            cv::Size(static_cast<int>(kTargetWidth), static_cast<int>(kTargetHeight)),
            0.0,
            0.0,
            cv::INTER_LINEAR);
          output_frame = resized_frame;
        }

        sensor_msgs::msg::Image msg;
        const auto system_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        const auto monotonic_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto offset_ns = system_now_ns - monotonic_now_ns;
        const auto buffer_timestamp_ns =
          static_cast<std::int64_t>(buffer.timestamp.tv_sec) * 1000000000LL +
          static_cast<std::int64_t>(buffer.timestamp.tv_usec) * 1000LL;
        const std::int64_t ros_timestamp_ns = buffer_timestamp_ns + offset_ns;
        msg.header.stamp = rclcpp::Time(ros_timestamp_ns, RCL_SYSTEM_TIME);
        msg.header.frame_id = frame_id_;
        copy_mat_to_image_message(output_frame, msg);

        const auto header = msg.header;
        publisher_->publish(std::move(msg));
        ++publish_count_;

        if (rectified_publisher_ != nullptr && rectification_maps_.has_value()) {
          cv::Mat rectified_frame;
          cv::remap(
            output_frame,
            rectified_frame,
            rectification_maps_->map1,
            rectification_maps_->map2,
            cv::INTER_LINEAR);

          sensor_msgs::msg::Image rectified_msg;
          rectified_msg.header = header;
          copy_mat_to_image_message(rectified_frame, rectified_msg);
          rectified_publisher_->publish(std::move(rectified_msg));
        }
      }

      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 2000,
          "VIDIOC_QBUF failed for %s camera %s: %s",
          camera_name_.c_str(), device_path_.c_str(), std::strerror(errno));
      }
    }
  }

  rclcpp::Node & node_;
  std::string device_path_;
  std::string camera_name_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rectified_publisher_;
  std::optional<RectificationMaps> rectification_maps_;
  int fd_{-1};
  std::vector<MmapBuffer> buffers_;
  std::atomic<bool> running_{false};
  std::thread worker_thread_;
  std::atomic<std::uint64_t> capture_count_{0};
  std::atomic<std::uint64_t> publish_count_{0};
  bool streaming_{false};
  bool uses_mplane_{false};
  v4l2_buf_type buffer_type_{V4L2_BUF_TYPE_VIDEO_CAPTURE};
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
};

class StereoImagePublisher : public rclcpp::Node
{
public:
  StereoImagePublisher()
  : Node("stereo_image_publisher")
  {
    const bool publish_rectified_images = declare_parameter<bool>("publish_rectified_images", false);

    std::optional<StereoRectifier> rectifier;
    if (publish_rectified_images) {
      rectifier = StereoRectifier::load(kDefaultStereoCalibPath);
      RCLCPP_INFO(
        get_logger(),
        "Rectified image publishing enabled using calibration file: %s",
        kDefaultStereoCalibPath);
    }

    left_camera_ = std::make_unique<V4L2CameraPipeline>(
      *this,
      kLeftDevicePath,
      "left",
      kLeftFrameId,
      kLeftTopic,
      publish_rectified_images ? std::optional<std::string>(kLeftRectTopic) : std::nullopt,
      rectifier.has_value() ? std::optional<RectificationMaps>(rectifier->left_maps()) : std::nullopt);
    right_camera_ = std::make_unique<V4L2CameraPipeline>(
      *this,
      kRightDevicePath,
      "right",
      kRightFrameId,
      kRightTopic,
      publish_rectified_images ? std::optional<std::string>(kRightRectTopic) : std::nullopt,
      rectifier.has_value() ? std::optional<RectificationMaps>(rectifier->right_maps()) : std::nullopt);

    try {
      left_camera_->start();
      right_camera_->start();
      if (publish_rectified_images) {
        left_camera_->validate_rectification_output();
        right_camera_->validate_rectification_output();
      }
    } catch (...) {
      shutdown_cameras();
      throw;
    }

    last_stats_time_ = now();
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&StereoImagePublisher::log_stats, this));
    RCLCPP_INFO(
      get_logger(),
      "Stereo image publisher started with pure V4L2 capture; publishing mono8 grayscale images from Y/luma");
  }

  ~StereoImagePublisher() override
  {
    shutdown_cameras();
  }

private:
  void log_stats()
  {
    const auto current_time = now();
    const double elapsed = (current_time - last_stats_time_).seconds();
    if (elapsed <= 0.0) {
      return;
    }

    const auto [left_capture_count, left_publish_count] = left_camera_->snapshot_and_reset_counts();
    const auto [right_capture_count, right_publish_count] = right_camera_->snapshot_and_reset_counts();

    RCLCPP_INFO(
      get_logger(),
      "left capture/publish=%.2f/%.2f fps output=%ux%u capture=%ux%u fourcc=%s | right capture/publish=%.2f/%.2f fps output=%ux%u capture=%ux%u fourcc=%s",
      static_cast<double>(left_capture_count) / elapsed,
      static_cast<double>(left_publish_count) / elapsed,
      left_camera_->publish_width(),
      left_camera_->publish_height(),
      left_camera_->actual_width(),
      left_camera_->actual_height(),
      left_camera_->fourcc().c_str(),
      static_cast<double>(right_capture_count) / elapsed,
      static_cast<double>(right_publish_count) / elapsed,
      right_camera_->publish_width(),
      right_camera_->publish_height(),
      right_camera_->actual_width(),
      right_camera_->actual_height(),
      right_camera_->fourcc().c_str());

    last_stats_time_ = current_time;
  }

  void shutdown_cameras()
  {
    if (left_camera_) {
      left_camera_->stop();
    }
    if (right_camera_) {
      right_camera_->stop();
    }
  }

  std::unique_ptr<V4L2CameraPipeline> left_camera_;
  std::unique_ptr<V4L2CameraPipeline> right_camera_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Time last_stats_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<StereoImagePublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_image_publisher"), "%s", ex.what());
  }

  rclcpp::shutdown();
  return 0;
}
