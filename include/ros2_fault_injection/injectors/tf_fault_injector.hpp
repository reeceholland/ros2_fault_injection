#ifndef ROS2_FAULT_INJECTION__INJECTORS__TF_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__INJECTORS__TF_FAULT_INJECTOR_HPP_

#include <random>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection
{

class TfFaultInjector : public FaultInjectorBase {
public:
  explicit TfFaultInjector(rclcpp::Node & node, const InjectorConfig & config);

private:
  void on_message(const tf2_msgs::msg::TFMessage & msg);
  bool should_drop_transform(const geometry_msgs::msg::TransformStamped & transform);
  void apply_translation_bias(geometry_msgs::msg::TransformStamped & transform);
  void apply_rotation_bias(geometry_msgs::msg::TransformStamped & transform);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

}  // namespace ros2_fault_injection

#endif
