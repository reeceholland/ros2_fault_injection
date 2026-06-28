// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/fault_scenario_runner.hpp"

#include <chrono>
#include <thread>

namespace ros2_fault_injection::core
{

FaultScenarioRunner::FaultScenarioRunner(
  rclcpp::Node & node,
  FaultController & controller,
  std::chrono::milliseconds timeout)
: node_(node), controller_(controller), timeout_(timeout)
{
}

ScenarioRunnerResult FaultScenarioRunner::run()
{
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_.get_node_base_interface());

    if (assertions_complete()) {
      const auto report = controller_.create_report();
      const auto passed = assertions_passed();

      return ScenarioRunnerResult{
        passed,
        passed ? "Scenario passed" : "Scenario failed",
        report.final_result,
        controller_.create_report_markdown()};
    }

    if (std::chrono::steady_clock::now() - start > timeout_) {
      return ScenarioRunnerResult{
        false,
        "Scenario timed out",
        "timeout",
        controller_.create_report_markdown()};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{20});
  }

  return ScenarioRunnerResult{
    false,
    "ROS shutdown before scenario completed",
    "interrupted",
    controller_.create_report_markdown()};
}

bool FaultScenarioRunner::assertions_complete() const
{
  return controller_.assertions_complete();
}

bool FaultScenarioRunner::assertions_passed() const
{
  return controller_.assertions_passed();
}

} // namespace ros2_fault_injection::core
