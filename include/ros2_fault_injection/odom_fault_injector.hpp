#ifndef ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_

#include <deque>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "ros2_fault_injection/fault_injector_base.hpp"

namespace ros2_fault_injection {

class OdomFaultInjector : public FaultInjectorBase {
public:
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
  void warn_unknown_config_keys(const FaultConfig& fault_config) const override;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedOdom> delayed_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_
