#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include <linux/videodev2.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "alg_camera_rospub/clock_offset.hpp"
#include "alg_camera_rospub/stereo_camera_node.hpp"
#include "alg_camera_rospub/v4l2_capture.hpp"

namespace alg_camera_rospub
{

namespace
{

uint32_t fourcc_from_string(const std::string & s)
{
  if (s.size() != 4) {
    return V4L2_PIX_FMT_YUYV;
  }
  return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

std::string encoding_for_mat(const cv::Mat & frame)
{
  if (frame.channels() == 3) {
    return "bgr8";
  }
  return "mono8";
}

void copy_mat_to_image(const cv::Mat & frame, sensor_msgs::msg::Image & msg)
{
  msg.height = static_cast<uint32_t>(frame.rows);
  msg.width = static_cast<uint32_t>(frame.cols);
  msg.encoding = encoding_for_mat(frame);
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
  config.output_color = node.declare_parameter<bool>(prefix + "output_color", false);
  return config;
}

}  // namespace

struct StereoCameraNode::Impl
{
  explicit Impl(StereoCameraNode * node)
  : node_{node}
  {
    // --- Declare per-camera parameters ---
    V4L2Config left_config = make_config(*node_, "left_");
    V4L2Config right_config = make_config(*node_, "right_");

    // --- Declare topic name and frame ID parameters ---
    left_topic_ = node_->declare_parameter<std::string>("left_topic_name", "/image/left");
    right_topic_ = node_->declare_parameter<std::string>("right_topic_name", "/image/right");
    left_frame_id_ = node_->declare_parameter<std::string>("left_frame_id", "left_camera_link");
    right_frame_id_ = node_->declare_parameter<std::string>("right_frame_id", "right_camera_link");
    const bool publish_compressed = node_->declare_parameter<bool>("publish_compressed", false);
    compressed_quality_ = node_->declare_parameter<int>("compressed_quality", 80);

    // --- Create and start V4L2 captures ---
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

    // --- Create publishers ---
    left_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(left_topic_, 10);
    right_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(right_topic_, 10);

    // --- Optional compressed publishers ---
    if (publish_compressed) {
      left_compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        left_topic_ + "/compressed", 10);
      right_compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        right_topic_ + "/compressed", 10);
    }

    // --- Log setup ---
    RCLCPP_INFO(node_->get_logger(),
      "left: capture=%ux%u publish=%ux%u fourcc=%s | right: capture=%ux%u publish=%ux%u fourcc=%s compressed=%d",
      left_cam_->actual_width(), left_cam_->actual_height(),
      left_cam_->publish_width(), left_cam_->publish_height(),
      left_cam_->fourcc_string().c_str(),
      right_cam_->actual_width(), right_cam_->actual_height(),
      right_cam_->publish_width(), right_cam_->publish_height(),
      right_cam_->fourcc_string().c_str(),
      static_cast<int>(publish_compressed));

    // --- Stats timer (also recalibrates clock offset) ---
    last_stats_time_ = node_->now();
    stats_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { log_stats(); });

    // --- Main capture timer (pull model at FPS) ---
    const double fps = std::max(left_config.requested_fps, right_config.requested_fps);
    const auto period = std::chrono::duration<double>(1.0 / fps);
    capture_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { capture_and_publish(); });
  }

  ~Impl()
  {
    if (left_cam_) {
      left_cam_->stop();
    }
    if (right_cam_) {
      right_cam_->stop();
    }
  }

  void capture_and_publish()
  {
    publish_one(left_cam_.get(), left_pub_, left_compressed_pub_, left_frame_id_);
    publish_one(right_cam_.get(), right_pub_, right_compressed_pub_, right_frame_id_);
  }

  void publish_one(V4L2Capture * cam,
                   const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
                   const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & comp_pub,
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

    if (comp_pub) {
      std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, compressed_quality_};
      std::vector<uchar> jpeg_buf;
      if (cv::imencode(".jpg", *frame, jpeg_buf, jpeg_params)) {
        sensor_msgs::msg::CompressedImage comp_msg;
        comp_msg.header = header;
        comp_msg.format = "jpeg";
        comp_msg.data = std::move(jpeg_buf);
        comp_pub->publish(std::move(comp_msg));
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Failed to encode compressed image");
      }
    }
  }

  void log_stats()
  {
    const auto current = node_->now();
    const double elapsed = (current - last_stats_time_).seconds();
    if (elapsed <= 0.0) {
      return;
    }

    const auto l_stats = left_cam_->snapshot_and_reset_stats();
    const auto r_stats = right_cam_->snapshot_and_reset_stats();

    RCLCPP_INFO(node_->get_logger(),
      "left capture/pub=%.2f/%.2f fps %ux%u | right capture/pub=%.2f/%.2f fps %ux%u",
      static_cast<double>(l_stats.capture_count) / elapsed,
      static_cast<double>(l_stats.publish_count) / elapsed,
      left_cam_->publish_width(), left_cam_->publish_height(),
      static_cast<double>(r_stats.capture_count) / elapsed,
      static_cast<double>(r_stats.publish_count) / elapsed,
      right_cam_->publish_width(), right_cam_->publish_height());

    clock_offset_.calibrate();
    last_stats_time_ = current;
  }

  StereoCameraNode * node_;
  ClockOffset clock_offset_;
  std::string left_topic_;
  std::string right_topic_;
  std::string left_frame_id_{"left_camera_link"};
  std::string right_frame_id_{"right_camera_link"};
  std::unique_ptr<V4L2Capture> left_cam_;
  std::unique_ptr<V4L2Capture> right_cam_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_pub_;
  int compressed_quality_{80};
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Time last_stats_time_;
};

StereoCameraNode::StereoCameraNode(const rclcpp::NodeOptions & options)
: Node("stereo_camera_publisher", options),
  impl_(std::make_unique<Impl>(this))
{
}

StereoCameraNode::~StereoCameraNode() = default;

}  // namespace alg_camera_rospub

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(alg_camera_rospub::StereoCameraNode)
