// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_ASSERTION_RUNNER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_ASSERTION_RUNNER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/fault_event_assertion.hpp"

namespace ros2_fault_injection
{
class FaultAssertionRunner
{
public:
  FaultAssertionRunner(rclcpp::Node & node);

  void start(const std::vector<AssertionConfig> & assertions);
  std::vector<AssertionResult> results() const;

private:
  void on_fault_event(const msg::FaultEvent & event);
  void update();

  rclcpp::Node & node_;
  rclcpp::Time start_time_;

  std::vector<FaultEventAssertion> fault_event_assertions_;

  rclcpp::Subscription<msg::FaultEvent>::SharedPtr fault_event_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_ASSERTION_RUNNER_HPP_
