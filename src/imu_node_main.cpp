#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "v4l2_publisher/imu_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<v4l2_publisher::ImuNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
