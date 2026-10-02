// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTIONS__TWIST_STOPPED_ASSERTION_HPP_
#define ROS2_FAULT_INJECTION__ASSERTIONS__TWIST_STOPPED_ASSERTION_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"

#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection::assertions
{
class TwistStoppedAssertion
{
public:
  TwistStoppedAssertion(const AssertionConfig & config, const rclcpp::Time & start_time);

  const std::string & topic() const;
  void observe_fault_event(const msg::FaultEvent & event, const rclcpp::Time & stamp);
  void observe_message(const geometry_msgs::msg::Twist & message, const rclcpp::Time & stamp);
  void update(const rclcpp::Time & now);
  AssertionResult result() const;

private:
  bool is_zero(const geometry_msgs::msg::Twist & message) const;
  bool is_finite(const geometry_msgs::msg::Twist & message) const;
  void fail(const std::string & message);

  AssertionConfig config_;
  AssertionResult result_;
  rclcpp::Time start_time_;
  bool fault_active_{false};
  std::optional<rclcpp::Time> fault_activation_stamp_;
  std::optional<rclcpp::Time> first_zero_stamp_;
  std::optional<rclcpp::Time> last_zero_stamp_;
  std::vector<std::pair<geometry_msgs::msg::Twist, rclcpp::Time>> pending_messages_;
};
}  // namespace ros2_fault_injection::assertions

#endif  // ROS2_FAULT_INJECTION__ASSERTIONS__TWIST_STOPPED_ASSERTION_HPP_
