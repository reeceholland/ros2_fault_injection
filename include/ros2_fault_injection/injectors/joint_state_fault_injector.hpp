#ifndef ROS2_FAULT_INJECTION__JOINT_STATE_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__JOINT_STATE_FAULT_INJECTOR_HPP_

#include <deque>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection {

/**
 * @brief Fault injector for `sensor_msgs/msg/JointState` streams.
 *
 * Subscribes to the configured input topic, applies active joint state faults,
 * and republishes the result on the configured output topic.
 */
class JointStateFaultInjector : public FaultInjectorBase {
public:
  /**
   * @brief Create the joint state fault injector.
   *
   * @param node Node used to create publishers, subscriptions, and timers.
   * @param config Injector topic and QoS configuration.
   */
  explicit JointStateFaultInjector(rclcpp::Node& node, const InjectorConfig& config);

private:
  struct DelayedJointState {
    sensor_msgs::msg::JointState msg;
    rclcpp::Time release_time;
  };

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void flush_delayed();

  void apply_bias(sensor_msgs::msg::JointState& msg);
  void apply_noise(sensor_msgs::msg::JointState& msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedJointState> delayed_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__JOINT_STATE_FAULT_INJECTOR_HPP_