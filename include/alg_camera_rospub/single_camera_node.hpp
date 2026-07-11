#ifndef ALG_CAMERA_ROSPUB__SINGLE_CAMERA_NODE_HPP_
#define ALG_CAMERA_ROSPUB__SINGLE_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace alg_camera_rospub
{
class SingleCameraNode : public rclcpp::Node
{
public:
  explicit SingleCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SingleCameraNode() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace alg_camera_rospub

#endif  // ALG_CAMERA_ROSPUB__SINGLE_CAMERA_NODE_HPP_
