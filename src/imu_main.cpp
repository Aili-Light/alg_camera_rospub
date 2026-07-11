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
