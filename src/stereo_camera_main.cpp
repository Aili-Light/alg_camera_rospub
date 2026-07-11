#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "alg_camera_rospub/stereo_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<alg_camera_rospub::StereoCameraNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_camera_publisher"), "%s", ex.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
