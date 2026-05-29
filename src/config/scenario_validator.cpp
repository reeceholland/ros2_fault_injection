// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/scenario_validator.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

#include "ros2_fault_injection/config/fault_config_schema.hpp"
#include "ros2_fault_injection/injectors/imu_fault_injector.hpp"
#include "ros2_fault_injection/injectors/joint_state_fault_injector.hpp"
#include "ros2_fault_injection/injectors/odom_fault_injector.hpp"
#include "ros2_fault_injection/injectors/scan_fault_injector.hpp"
#include "ros2_fault_injection/injectors/tf_fault_injector.hpp"
#include "ros2_fault_injection/injectors/trigger_service_fault_injector.hpp"

namespace ros2_fault_injection
{
namespace
{

bool is_known_injector_type(const std::string & type)
{
  return type == "odom" || type == "scan" || type == "joint_state" || type == "imu" ||
         type == "trigger_service" || type == "tf";
}

std::vector<FaultConfigField> schema_for_builtin_injector_type(const std::string & type)
{
  if (type == "odom") {
    return OdomFaultInjector::static_config_schema();
  }

  if (type == "scan") {
    return ScanFaultInjector::static_config_schema();
  }

  if (type == "joint_state") {
    return JointStateFaultInjector::static_config_schema();
  }

  if (type == "imu") {
    return ImuFaultInjector::static_config_schema();
  }

  if (type == "trigger_service") {
    return TriggerServiceFaultInjector::static_config_schema();
  }

  if (type == "tf") {
    return TfFaultInjector::static_config_schema();
  }

  return {};
}

const FaultConfigField * find_schema_field(
  const std::vector<FaultConfigField> & schema,
  const std::string & key)
{
  const auto it = std::find_if(
    schema.begin(), schema.end(),
    [&key](const FaultConfigField & field) {
      return field.key == key;
    });

  if (it == schema.end()) {
    return nullptr;
  }

  return &(*it);
}

void validate_fault_config_against_schema(
  const FaultConfig & fault,
  const std::vector<FaultConfigField> & schema,
  ValidationResult & result)
{
  for (const auto & [key, value] : fault.config) {
    const auto * field = find_schema_field(schema, key);

    if (field == nullptr) {
      result.warnings.push_back("fault '" + fault.id + "' has unknown config key '" + key + "'");
      continue;
    }

    const auto error = validate_config_value(*field, value);
    if (error.has_value()) {
      result.errors.push_back("fault '" + fault.id + "' " + error.value());
    }
  }
}

void validate_required_schema_fields(
  const FaultConfig & fault,
  const std::vector<FaultConfigField> & schema,
  ValidationResult & result)
{
  for (const auto & field : schema) {
    if (field.type != "non_empty_string") {
      continue;
    }

    const auto it = fault.config.find(field.key);
    if (it == fault.config.end()) {
      result.errors.push_back("fault '" + fault.id + "' config '" + field.key + "' is required");
    }
  }
}

void validate_injector(const InjectorConfig & injector, ValidationResult & result)
{
  if (injector.id.empty()) {
    result.errors.push_back("injector.id must not be empty");
  }

  if (!is_known_injector_type(injector.type)) {
    result.warnings.push_back(
      "injector.type '" + injector.type + "' is not built in; assuming external plugin");
  }

  if (!injector.topic && !injector.trigger_service) {
    result.errors.push_back("injector must have either topic or trigger_service configuration");
    return;
  }

  if (injector.topic && injector.trigger_service) {
    result.errors.push_back("injector cannot have both topic and trigger_service configuration");
    return;
  }

  if (injector.topic) {
    if (injector.topic->input_topic.empty()) {
      result.errors.push_back("injector.topic.input_topic must not be empty");
    }

    if (injector.topic->output_topic.empty()) {
      result.errors.push_back("injector.topic.output_topic must not be empty");
    }

    if (injector.topic->qos_depth == 0) {
      result.errors.push_back("injector.topic.qos_depth must be greater than 0");
    }
  }

  if (injector.trigger_service) {
    if (injector.trigger_service->proxy_service.empty()) {
      result.errors.push_back("injector.trigger_service.proxy_service must not be empty");
    }

    if (injector.trigger_service->target_service.empty()) {
      result.errors.push_back("injector.trigger_service.target_service must not be empty");
    }
  }
}

bool is_initially_active(const ScenarioConfig & scenario, const FaultConfig & fault)
{
  return std::find(scenario.initially_active_faults.begin(), scenario.initially_active_faults.end(),
                   fault.id) != scenario.initially_active_faults.end();
}

void validate_fault(
  const ScenarioConfig & scenario,
  const FaultConfig & fault,
  const InjectorConfig & injector,
  ValidationResult & result)
{
  const bool active_on_startup = is_initially_active(scenario, fault);

  if (fault.id.empty()) {
    result.errors.push_back("fault.id must not be empty");
  }

  if (fault.injector_id.empty()) {
    result.errors.push_back("fault '" + fault.id + "' injector_id must not be empty");
  }

  if (fault.injector_id != injector.id) {
    result.errors.push_back("fault '" + fault.id + "' targets injector_id '" + fault.injector_id +
                            "' but scenario injector is '" + injector.id + "'");
  }

  if (active_on_startup && fault.start) {
    result.warnings.push_back("fault '" + fault.id +
                              "' has active_on_startup=true and start; start will be ignored");
  }

  if (!active_on_startup && fault.duration && !fault.start) {
    result.warnings.push_back("fault '" + fault.id +
                              "' has duration but no start; duration will be ignored");
  }

  if (fault.start && *fault.start < std::chrono::milliseconds(0)) {
    result.errors.push_back("fault '" + fault.id + "' has negative start time");
  }

  if (fault.duration && *fault.duration < std::chrono::milliseconds(0)) {
    result.errors.push_back("fault '" + fault.id + "' has negative duration");
  }

  if (is_known_injector_type(injector.type)) {
    const auto schema = schema_for_builtin_injector_type(injector.type);
    validate_fault_config_against_schema(fault, schema, result);
    validate_required_schema_fields(fault, schema, result);
  }
}

}  // namespace

ValidationResult validate_scenario(const ScenarioConfig & scenario)
{
  ValidationResult result;

  std::unordered_set<std::string> injector_ids;
  for (const auto & injector : scenario.injectors) {
    if (!injector.id.empty() && !injector_ids.insert(injector.id).second) {
      result.errors.push_back("duplicate injector id: '" + injector.id + "'");
    }

    validate_injector(injector, result);
  }

  for (const auto & fault : scenario.faults) {
    const auto * injector = find_injector(scenario, fault.injector_id);

    if (injector == nullptr) {
      result.errors.push_back("fault '" + fault.id + "' references unknown injector_id '" +
                              fault.injector_id + "'");
      continue;
    }

    validate_fault(scenario, fault, *injector, result);
  }

  return result;
}

}  // namespace ros2_fault_injection
