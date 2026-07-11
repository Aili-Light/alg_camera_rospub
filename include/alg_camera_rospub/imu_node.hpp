#ifndef ALG_CAMERA_ROSPUB__IMU_NODE_HPP_
#define ALG_CAMERA_ROSPUB__IMU_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace alg_camera_rospub
{
class ImuNode : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImuNode() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace alg_camera_rospub

#endif  // ALG_CAMERA_ROSPUB__IMU_NODE_HPP_
