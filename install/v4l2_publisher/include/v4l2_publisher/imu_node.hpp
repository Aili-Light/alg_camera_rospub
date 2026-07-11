#ifndef V4L2_PUBLISHER__IMU_NODE_HPP_
#define V4L2_PUBLISHER__IMU_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace v4l2_publisher
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
}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__IMU_NODE_HPP_
