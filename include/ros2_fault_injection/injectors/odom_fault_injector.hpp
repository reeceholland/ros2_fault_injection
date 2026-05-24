#ifndef ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_

#include <deque>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection {

/**
 * @brief Fault injector for `nav_msgs/msg/Odometry` streams.
 *
 * Subscribes to the configured input topic, applies active odometry faults,
 * and republishes the result on the configured output topic.
 */
class OdomFaultInjector : public FaultInjectorBase {
public:
  /**
   * @brief Create the odometry fault injector.
   *
   * @param node Node used to create publishers, subscriptions, and timers.
   * @param config Injector topic and QoS configuration.
   */
  explicit OdomFaultInjector(rclcpp::Node& node, const InjectorConfig& config);

private:
  struct DelayedOdom {
    nav_msgs::msg::Odometry msg;
    rclcpp::Time release_time;
  };

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void flush_delayed();

  void apply_bias(nav_msgs::msg::Odometry& msg);
  void apply_noise(nav_msgs::msg::Odometry& msg);
  void apply_yaw_bias(nav_msgs::msg::Odometry& msg);
  void apply_yaw_noise(nav_msgs::msg::Odometry& msg);
  void apply_covariance_scale(nav_msgs::msg::Odometry& msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedOdom> delayed_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_
