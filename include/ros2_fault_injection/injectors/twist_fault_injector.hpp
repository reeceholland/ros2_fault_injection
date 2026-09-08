// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__TWIST_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__TWIST_FAULT_INJECTOR_HPP_

#include <chrono>
#include <deque>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection::injectors
{

/**
 * @brief Fault injector for `geometry_msgs/msg/Twist` command streams.
 *
 * Subscribes to the configured input topic, applies active command faults,
 * and republishes the result on the configured output topic.
 */
class TwistFaultInjector : public FaultInjectorBase
{
public:
  /**
   * @brief Create the Twist fault injector.
   *
   * @param node Node used to create publishers, subscriptions, and timers.
   * @param config Injector topic and QoS configuration.
   */
  explicit TwistFaultInjector(rclcpp::Node & node, const InjectorConfig & config);

  static std::vector<FaultConfigField> static_config_schema();

  std::vector<FaultConfigField> config_schema() const override;

private:
  struct DelayedTwist
  {
    geometry_msgs::msg::Twist msg;
    rclcpp::Time release_time;
  };

  void on_twist(const geometry_msgs::msg::Twist::SharedPtr msg);
  void flush_delayed();
  bool stale_replay_enabled() const;
  std::chrono::milliseconds stale_replay_duration() const;
  double active_product_double(const std::string & key, double fallback) const;
  void apply_scale(geometry_msgs::msg::Twist & msg) const;
  void apply_clamp(geometry_msgs::msg::Twist & msg) const;
  void apply_force_stop(geometry_msgs::msg::Twist & msg) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedTwist> delayed_;
  std::optional<geometry_msgs::msg::Twist> last_command_;
  rclcpp::Time last_command_time_;
};

} // namespace ros2_fault_injection::injectors

namespace ros2_fault_injection
{
using injectors::TwistFaultInjector;
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__TWIST_FAULT_INJECTOR_HPP_
