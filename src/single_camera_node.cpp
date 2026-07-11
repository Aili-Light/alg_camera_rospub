#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include <linux/videodev2.h>

#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "alg_camera_rospub/clock_offset.hpp"
#include "alg_camera_rospub/rectification.hpp"
#include "alg_camera_rospub/single_camera_node.hpp"
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
      frame.ptr(row),
      frame.step);
  }
}

}  // namespace

struct SingleCameraNode::Impl
{
  explicit Impl(SingleCameraNode * node)
  : node_{node}
  {
    // --- Declare parameters ---
    const auto device_path = node_->declare_parameter<std::string>("device_path", "/dev/video0");
    const int req_w = node_->declare_parameter<int>("requested_width", 1280);
    const int req_h = node_->declare_parameter<int>("requested_height", 800);
    const uint32_t target_w = static_cast<uint32_t>(node_->declare_parameter<int>("target_width", 640));
    const uint32_t target_h = static_cast<uint32_t>(node_->declare_parameter<int>("target_height", 400));
    const std::string fourcc_str = node_->declare_parameter<std::string>("pixel_format", "YUYV");
    const double fps = node_->declare_parameter<double>("fps", 60.0);
    const std::string topic = node_->declare_parameter<std::string>("topic_name", "/image");
    frame_id_ = node_->declare_parameter<std::string>("frame_id", "camera_link");
    const bool rect_enable = node_->declare_parameter<bool>("publish_rectified", false);
    const std::string calib_path = node_->declare_parameter<std::string>("calib_path", "./camera_calib.yaml");

    // --- Configure and start V4L2 capture ---
    V4L2Config config;
    config.device_path = device_path;
    config.requested_width = static_cast<uint32_t>(req_w);
    config.requested_height = static_cast<uint32_t>(req_h);
    config.target_width = target_w;
    config.target_height = target_h;
    config.pixel_format = fourcc_from_string(fourcc_str);
    config.requested_fps = fps;

    capture_ = std::make_unique<V4L2Capture>(config);
    capture_->start();

    // --- Create publisher ---
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic, 10);

    // --- Optional rectified publisher ---
    if (rect_enable) {
      rect_maps_ = load_rectification_maps(calib_path, "left");
      validate_rectification_maps(*rect_maps_, "camera",
                                  capture_->publish_width(),
                                  capture_->publish_height());
      rect_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic + "_rect", 10);
    }

    // --- Log setup ---
    RCLCPP_INFO(node_->get_logger(),
      "Single camera publisher: device=%s capture=%ux%u publish=%ux%u fps=%.1f topic=%s",
      device_path.c_str(),
      capture_->actual_width(), capture_->actual_height(),
      capture_->publish_width(), capture_->publish_height(),
      capture_->actual_fps(), topic.c_str());

    // --- Stats timer (also recalibrates clock offset) ---
    stats_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        clock_offset_.calibrate();
        const auto stats = capture_->snapshot_and_reset_stats();
        RCLCPP_INFO(node_->get_logger(),
          "capture=%.2f publish=%.2f fps output=%ux%u capture=%ux%u fourcc=%s",
          static_cast<double>(stats.capture_count),
          static_cast<double>(stats.publish_count),
          capture_->publish_width(), capture_->publish_height(),
          capture_->actual_width(), capture_->actual_height(),
          capture_->fourcc_string().c_str());
      });

    // --- Main capture timer ---
    const auto period = std::chrono::duration<double>(1.0 / fps);
    capture_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { capture_and_publish(); });
  }

  ~Impl()
  {
    if (capture_) {
      capture_->stop();
    }
  }

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

  SingleCameraNode * node_;
  std::string frame_id_{"camera_link"};
  std::unique_ptr<V4L2Capture> capture_;
  ClockOffset clock_offset_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  std::optional<RectificationMaps> rect_maps_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
};

SingleCameraNode::SingleCameraNode(const rclcpp::NodeOptions & options)
: Node("single_camera_publisher", options),
  impl_(std::make_unique<Impl>(this))
{
}

SingleCameraNode::~SingleCameraNode() = default;

}  // namespace alg_camera_rospub

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(alg_camera_rospub::SingleCameraNode)
