// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_SCENARIO_RUNNER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_SCENARIO_RUNNER_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/core/fault_controller.hpp"

namespace ros2_fault_injection::core
{

struct ScenarioRunnerResult
{
  bool success{false};
  std::string message;
  std::string final_result;
  std::string report_markdown;
};

class FaultScenarioRunner
{
public:
  FaultScenarioRunner(
    rclcpp::Node & node,
    FaultController & controller,
    std::chrono::milliseconds timeout);

  ScenarioRunnerResult run();

private:
  bool assertions_complete() const;
  bool assertions_passed() const;

  rclcpp::Node & node_;
  FaultController & controller_;
  std::chrono::milliseconds timeout_;
};

} // namespace ros2_fault_injection::core

#endif // ROS2_FAULT_INJECTION__FAULT_SCENARIO_RUNNER_HPP_
