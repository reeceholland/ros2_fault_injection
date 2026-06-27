// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros2_fault_injection/core/scenario_report.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"

namespace ros2_fault_injection::core
{
  class ReportCreator
  {
  public:
    explicit ReportCreator(rclcpp::Node &node);

    ScenarioReport create_report(
        const std::string &scenario_file, const InjectorMap &injectors,
        const std::vector<assertions::AssertionResult> &assertion_results,
        const std::vector<FaultEventRecord> &fault_events);

    std::string to_markdown(const ScenarioReport &report) const;

  private:
    rclcpp::Node &node_;
  };

} // namespace ros2_fault_injection::core
