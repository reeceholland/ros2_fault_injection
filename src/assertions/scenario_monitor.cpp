// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/assertions/scenario_monitor.hpp"

#include <string>
#include <vector>

#include <ros2_fault_injection/msg/scenario_status.hpp>

namespace ros2_fault_injection::assertions
{
namespace
{
std::string scenario_state_from_counts(std::size_t pending_count, std::size_t failed_count)
{
  if (failed_count > 0) {
    return "failed";
  } else if (pending_count > 0) {
    return "running";
  } else {
    return "passed";
  }
}
}
ScenarioMonitor::ScenarioMonitor(rclcpp::Node & node)
: node_(node)
{
  publisher_ = node_.create_publisher<msg::ScenarioStatus>("/fault_injection/scenario_status", 10);
}

void ScenarioMonitor::publish(const std::vector<AssertionResult> & results)
{
  msg::ScenarioStatus status;
  status.header.stamp = node_.now();

  for (const auto & result : results) {
    switch (result.state) {
      case AssertionState::Pending:
        {
          ++status.pending_count;
          break;
        }
      case AssertionState::Passed:
        {
          ++status.passed_count;
          break;
        }
      case AssertionState::Failed:
        {
          ++status.failed_count;
          status.failed_assertion_ids.push_back(result.id);
          status.failed_msgs.push_back(result.message);
          break;
        }
    }
  }

  status.state = scenario_state_from_counts(status.pending_count, status.failed_count);
  publisher_->publish(status);
}
} // namespace ros2_fault_injection::assertions
