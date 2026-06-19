// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTIONS__SCENARIO__MONITOR_HPP_
#define ROS2_FAULT_INJECTION__ASSERTIONS__SCENARIO__MONITOR_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros2_fault_injection/assertions/assertion_result.hpp>
#include <ros2_fault_injection/msg/scenario_status.hpp>

namespace ros2_fault_injection::assertions
{
class ScenarioMonitor
{
public:
  ScenarioMonitor(rclcpp::Node & node);

  void publish(const std::vector<AssertionResult> & results);

private:
  rclcpp::Publisher<msg::ScenarioStatus>::SharedPtr publisher_;
  rclcpp::Node & node_;
};
} // namespace ros2_fault_injection::assertions

#endif // ROS2_FAULT_INJECTION__ASSERTIONS__SCENARIO__MONITOR_HPP_
