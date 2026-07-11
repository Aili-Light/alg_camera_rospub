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
