#ifndef V4L2_PUBLISHER__STEREO_CAMERA_NODE_HPP_
#define V4L2_PUBLISHER__STEREO_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace v4l2_publisher
{
class StereoCameraNode : public rclcpp::Node
{
public:
  explicit StereoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoCameraNode() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__STEREO_CAMERA_NODE_HPP_
