// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/fault_controller.hpp"

#include <exception>
#include <string>
#include <utility>
#include <fstream>
#include <optional>
#include <sstream>

#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/config/scenario_validator.hpp"
#include "ros2_fault_injection/core/report_creator.hpp"
#include "ros2_fault_injection/core/scenario_report.hpp"

namespace ros2_fault_injection
{

FaultController::FaultController(
  rclcpp::Node & node, std::string scenario_file, ScenarioConfig scenario,
  FaultEventPublisher & events)
: node_(node), scenario_file_(std::move(scenario_file)), scenario_(std::move(scenario)),
  events_(events), factory_(node), scheduler_(node, events)
{
  assertion_runner_ = std::make_unique<assertions::FaultAssertionRunner>(node_);
  assertion_runner_->start(scenario_.assertions);

  create_injectors();
  register_faults();
  schedule_faults();
}

const InjectorMap & FaultController::injectors() const
{
  return injectors_;
}

void FaultController::create_injectors()
{
  for (const auto & injector_config : scenario_.injectors) {
    auto injector = factory_.create(injector_config);

    if (!injector) {
      RCLCPP_ERROR(node_.get_logger(), "Unknown injector type '%s' for injector '%s'",
                     injector_config.type.c_str(), injector_config.id.c_str());

      continue;
    }

    injectors_[injector_config.id] = injector;
  }
}

void FaultController::register_faults()
{
  for (const auto & fault : scenario_.faults) {
    const auto injector_it = injectors_.find(fault.injector_id);

    if (injector_it == injectors_.end()) {
      RCLCPP_WARN(node_.get_logger(), "Fault '%s' targets unknown injector_id '%s', skipping",
                    fault.id.c_str(), fault.injector_id.c_str());

      continue;
    }

    injector_it->second->add_fault(fault);
  }
}

void FaultController::schedule_faults()
{
  for (const auto &[injector_id, injector] : injectors_) {
    std::vector<FaultConfig> targeted_faults;

    for (const auto & fault : scenario_.faults) {
      if (fault.injector_id == injector_id) {
        targeted_faults.push_back(fault);
      }
    }

    scheduler_.schedule(targeted_faults, *injector, scenario_.initially_active_faults);
  }
}

ReloadScenarioResult FaultController::reload_scenario()
{
  ScenarioConfig new_scenario;
  RCLCPP_INFO(node_.get_logger(), "Attempting to reload scenario from file '%s'",
                scenario_file_.c_str());

  try {
    new_scenario = load_scenario_config(scenario_file_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to reload scenario config %s : %s",
                   scenario_file_.c_str(), error.what());

    return ReloadScenarioResult{false,
      "failed to load scenario config: " + std::string(error.what())};
  }

  RCLCPP_INFO(node_.get_logger(), "Validating new scenario configuration from file '%s'",
                scenario_file_.c_str());
  const auto validation_result = validate_scenario(new_scenario);

  if (!validation_result.ok()) {
    std::string message = "Scenario validation failed";

    for (const auto & error : validation_result.errors) {
      message += "; " + error;
    }
    RCLCPP_WARN(node_.get_logger(), "Scenario reload failed validation: %s", message.c_str());
    return ReloadScenarioResult{
      false,
      message,
    };
  }

  for (const auto & warning : validation_result.warnings) {
    RCLCPP_WARN(node_.get_logger(), "Scenario reload warning: %s", warning.c_str());
  }
  const auto compatibility = validate_reload_compatible(new_scenario);
  if (!compatibility.success) {
    return compatibility;
  }

  scheduler_.clear();

  for (const auto &[injector_id, injector] : injectors_) {
    injector->clear_faults();
  }

  scenario_ = std::move(new_scenario);

  register_faults();
  schedule_faults();

  RCLCPP_INFO(node_.get_logger(), "Scenario reloaded successfully from file '%s'",
                scenario_file_.c_str());
  return ReloadScenarioResult{
    true,
    "Scenario reloaded successfully",
  };
}
ReloadScenarioResult FaultController::validate_reload_compatible(
  const ScenarioConfig & new_scenario) const
{
  if (new_scenario.injectors.size() != scenario_.injectors.size()) {
    return {false, "Reload cannot change the number of injectors"};
  }

  for (const auto & current : scenario_.injectors) {
    const auto *updated = find_injector(new_scenario, current.id);

    if (updated == nullptr) {
      return {false, "Reload cannot remove injector '" + current.id + "'"};
    }

    if (updated->type != current.type) {
      return {false, "Reload cannot change type for injector '" + current.id + "'"};
    }

      // Topic injector endpoint compatibility.
    if (current.topic.has_value() != updated->topic.has_value()) {
      return {false, "Reload cannot change endpoint kind for injector '" + current.id + "'"};
    }

    if (current.topic && updated->topic) {
      if (updated->topic->input_topic != current.topic->input_topic ||
        updated->topic->output_topic != current.topic->output_topic ||
        updated->topic->qos_depth != current.topic->qos_depth)
      {
        return {false, "Reload cannot change topic endpoints for injector '" + current.id + "'"};
      }
    }

      // Trigger service endpoint compatibility.
    if (current.trigger_service.has_value() != updated->trigger_service.has_value()) {
      return {false, "Reload cannot change endpoint kind for injector '" + current.id + "'"};
    }

    if (current.trigger_service && updated->trigger_service) {
      if (updated->trigger_service->proxy_service != current.trigger_service->proxy_service ||
        updated->trigger_service->target_service != current.trigger_service->target_service)
      {
        return {false, "Reload cannot change service endpoints for injector '" + current.id + "'"};
      }
    }
  }

  return {true, "Scenario is compatible with running injectors"};
}

std::string FaultController::scenario_file() const
{
  return scenario_file_;
}

const std::optional<std::string> FaultController::read_scenario_file() const
{
  if (scenario_file_.empty()) {
    RCLCPP_ERROR(node_.get_logger(), "No scenario file configured for controller");
    return std::nullopt;
  }

  std::ifstream file(scenario_file_);

  if (!file.is_open()) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to open scenario file '%s'", scenario_file_.c_str());
    return std::nullopt;
  }

  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

core::ScenarioReport FaultController::create_report() const
{
  core::ReportCreator report_creator(node_);
  const auto assertion_results = assertion_runner_ ? assertion_runner_->results() : std::vector<assertions::AssertionResult>{};

  return report_creator.create_report(scenario_file_, injectors_, assertion_results);
}

std::string FaultController::create_report_markdown() const
{
  core::ReportCreator report_creator(node_);
  return report_creator.to_markdown(create_report());
}
} // namespace ros2_fault_injection
