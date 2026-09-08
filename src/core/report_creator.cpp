// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/report_creator.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/scenario_report.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/utils/fault_descriptions.hpp"

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
  const InjectorMap & injectors, const std::vector<assertions::AssertionResult> & assertion_results,
  const std::vector<FaultEventRecord> & fault_events)
{
  ScenarioReport report;
  report.scenario_file = scenario_file;
  report.finished_at = node_.now();
  report.assertion_results = assertion_results;
  report.fault_events = fault_events;

  for (const auto &[injector_id, injector] : injectors) {
    report.injector_ids.push_back(injector_id);
    report.injectors.push_back(InjectorReportEntry{injector_id, injector->type()});

    const auto fault_ids = injector->fault_ids();
    const auto active_fault_ids = injector->active_fault_ids();
    const std::unordered_set<std::string> active_faults(active_fault_ids.begin(),
      active_fault_ids.end());

    for (const auto & fault_id : fault_ids) {
      report.fault_ids.push_back(fault_id);

      FaultReportEntry fault;
      fault.id = fault_id;
      fault.injector_id = injector_id;
      fault.state = active_faults.count(fault_id) > 0 ? "active" : "inactive";

      const auto fault_config = injector->get_fault_config(fault_id);
      fault.details = fault_config.has_value() ? describe_fault(fault_config.value()) :
        "config unavailable";

      report.faults.push_back(fault);
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

  if (report.injectors.empty()) {
    out << "_No injectors registered._\n\n";
  } else {
    out << "| Injector ID | Type |\n";
    out << "| --- | --- |\n";

    for (const auto & injector : report.injectors) {
      out << "| `" << injector.id << "` "
          << "| `" << injector.type << "` |\n";
    }

    out << "\n";
  }

  out << "## Faults\n\n";

  if (report.faults.empty()) {
    out << "_No faults registered._\n\n";
  } else {
    out << "| Fault ID | Injector | State | Details |\n";
    out << "| --- | --- | --- | --- |\n";

    for (const auto & fault : report.faults) {
      out << "| `" << fault.id << "` "
          << "| `" << fault.injector_id << "` "
          << "| `" << fault.state << "` "
          << "| " << fault.details << " |\n";
    }

    out << "\n";
  }

  out << "## Fault Event Timeline\n\n";

  if (report.fault_events.empty()) {
    out << "_No fault events recorded._\n\n";
  } else {
    out << "| Time (s) | Fault | Injector | State | Source | Details |\n";
    out << "| ---: | --- | --- | --- | --- | --- |\n";

    for (const auto & event : report.fault_events) {
      out   << "| " << std::fixed << std::setprecision(3) << event.stamp.seconds() << " "
            << "| `" << event.fault_id << "` "
            << "| `" << event.injector_id << "` "
            << "| `" << event.state << "` "
            << "| `" << event.source << "` "
            << "| " << event.details << " |\n";
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
