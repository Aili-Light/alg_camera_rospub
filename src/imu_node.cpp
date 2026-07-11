#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "v4l2_publisher/clock_offset.hpp"
#include "v4l2_publisher/imu_device.hpp"
#include "v4l2_publisher/imu_icm_45686.hpp"
#include "v4l2_publisher/imu_node.hpp"

namespace v4l2_publisher
{

namespace
{
constexpr double kStandardGravity = 9.80665;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kMaxFrequency = 200.0;
}  // anonymous namespace

struct ImuNode::Impl
{
  explicit Impl(ImuNode * node)
  : node_{node}
  {
    // --- Declare parameters ---
    const std::string i2c_bus =
      node_->declare_parameter<std::string>("i2c_bus", "/dev/i2c-3");
    const int imu_addr = node_->declare_parameter<int>("imu_address", 0x69);
    double freq = node_->declare_parameter<double>("frequency", 100.0);
    const std::string topic =
      node_->declare_parameter<std::string>("topic_name", "imu/data");
    frame_id_ = node_->declare_parameter<std::string>("frame_id", "imu_link");

    // Validate frequency
    if (freq > kMaxFrequency) {
      RCLCPP_WARN(node_->get_logger(),
        "Requested frequency %.1f Hz exceeds max %.0f Hz; limiting to %.0f Hz",
        freq, kMaxFrequency, kMaxFrequency);
      freq = kMaxFrequency;
    } else if (freq <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(),
        "Invalid frequency %.2f; using 10 Hz", freq);
      freq = 10.0;
    }

    // Initialize I2C and IMU device
    imu_ = std::make_unique<ImuIcm45686>(i2c_bus,
                                         static_cast<uint8_t>(imu_addr));

    // Create publisher
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic, 10);

    // Recalibrate clock offset periodically (every 1 second)
    clock_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { clock_offset_.calibrate(); });

    // Publish timer at the configured frequency
    const auto period = std::chrono::duration<double>(1.0 / freq);
    pub_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Impl::publish_imu, this));

    RCLCPP_INFO(node_->get_logger(),
      "IMU Publisher started: bus=%s addr=0x%02X topic=%s freq=%.1f Hz",
      i2c_bus.c_str(), imu_addr, topic.c_str(), freq);
  }

  void publish_imu()
  {
    float ax, ay, az, gx, gy, gz;

    if (!imu_->read_accel(ax, ay, az) || !imu_->read_gyro(gx, gy, gz)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Failed to read IMU data");
      return;
    }

    auto msg = sensor_msgs::msg::Imu();

    // Offset-corrected system-clock timestamp
    const int64_t system_ns = clock_offset_.now_ns();
    msg.header.stamp = rclcpp::Time(system_ns, RCL_SYSTEM_TIME);
    msg.header.frame_id = frame_id_;

    // Accelerometer: g -> m/s^2
    msg.linear_acceleration.x = ax * kStandardGravity;
    msg.linear_acceleration.y = ay * kStandardGravity;
    msg.linear_acceleration.z = az * kStandardGravity;

    // Gyroscope: deg/s -> rad/s
    msg.angular_velocity.x = gx * kDegToRad;
    msg.angular_velocity.y = gy * kDegToRad;
    msg.angular_velocity.z = gz * kDegToRad;

    // Set covariances to -1 (unknown) for all three axes
    msg.linear_acceleration_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.orientation_covariance[0] = -1.0;

    imu_pub_->publish(std::move(msg));
  }

  ImuNode * node_;
  std::unique_ptr<ImuDevice> imu_;
  ClockOffset clock_offset_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  std::string frame_id_;
};

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: Node("imu_publisher", options),
  impl_(std::make_unique<Impl>(this))
{
}

ImuNode::~ImuNode() = default;

}  // namespace v4l2_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_publisher::ImuNode)
