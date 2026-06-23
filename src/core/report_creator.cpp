// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/report_creator.hpp"

#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/scenario_report.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"

namespace ros2_fault_injection::core
{
namespace
{
std::string assertion_state_to_string(assertions::AssertionState state)
{
  switch (state) {
    case assertions::AssertionState::Pending:
      return "pending";
    case assertions::AssertionState::Passed:
      return "passed";
    case assertions::AssertionState::Failed:
      return "failed";
    default:
      return "unknown";
  }
}
}
ReportCreator::ReportCreator(rclcpp::Node & node)
: node_(node)
{
}

ScenarioReport ReportCreator::create_report(
  const std::string & scenario_file,
  const InjectorMap & injectors, const std::vector<assertions::AssertionResult> & assertion_results)
{
  ScenarioReport report;
  report.scenario_file = scenario_file;
  report.finished_at = node_.now();
  report.assertion_results = assertion_results;

  for (const auto &[injector_id, injector] : injectors) {
    report.injector_ids.push_back(injector_id);
    const auto fault_ids = injector->fault_ids();
    for (const auto & fault_id : fault_ids) {
      report.fault_ids.push_back(fault_id);
    }
  }

  bool has_failed = false;
  bool has_pending = false;
  for (const auto & assertion_result : assertion_results) {
    if (assertion_result.state == assertions::AssertionState::Failed) {
      has_failed = true;
    } else if (assertion_result.state == assertions::AssertionState::Pending) {
      has_pending = true;
    }
  }

  if (has_failed) {
    report.final_result = "failed";
  } else if (has_pending) {
    report.final_result = "pending";
  } else {
    report.final_result = "passed";
  }

  return report;
}

std::string ReportCreator::to_markdown(const ScenarioReport & report) const
{
  std::ostringstream out;

  out << "# Fault Injection Scenario Report\n\n";

  out << "## Summary\n\n";
  out << "- Scenario file: `" << report.scenario_file << "`\n";
  out << "- Final result: `" << report.final_result << "`\n";
  out << "- Injectors: " << report.injector_ids.size() << "\n";
  out << "- Faults: " << report.fault_ids.size() << "\n";
  out << "- Assertions: " << report.assertion_results.size() << "\n\n";

  out << "## Injectors\n\n";

  if (report.injector_ids.empty()) {
    out << "_No injectors registered._\n\n";
  } else {
    for (const auto & injector_id : report.injector_ids) {
      out << "- `" << injector_id << "`\n";
    }

    out << "\n";
  }

  out << "## Faults\n\n";

  if (report.fault_ids.empty()) {
    out << "_No faults registered._\n\n";
  } else {
    for (const auto & fault_id : report.fault_ids) {
      out << "- `" << fault_id << "`\n";
    }

    out << "\n";
  }

  out << "## Assertions\n\n";

  if (report.assertion_results.empty()) {
    out << "_No assertions configured._\n";
    return out.str();
  }

  out << "| Assertion ID | Type | State | Message |\n";
  out << "| --- | --- | --- | --- |\n";

  for (const auto & result : report.assertion_results) {
    out   << "| `" << result.id << "` "
          << "| `" << result.type << "` "
          << "| `" << assertion_state_to_string(result.state) << "` "
          << "| " << result.message << " |\n";
  }

  return out.str();
}

} // namespace ros2_fault_injection::core
