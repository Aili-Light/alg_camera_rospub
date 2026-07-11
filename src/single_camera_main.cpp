#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "alg_camera_rospub/single_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<alg_camera_rospub::SingleCameraNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("single_camera_publisher"), "%s", ex.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
