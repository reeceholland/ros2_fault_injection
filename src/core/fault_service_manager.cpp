// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/fault_service_manager.hpp"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "ros2_fault_injection/config/fault_config_schema.hpp"
#include "ros2_fault_injection/utils/fault_descriptions.hpp"
#include "ros2_fault_injection/msg/fault_status.hpp"

namespace ros2_fault_injection
{
namespace
{

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

} // namespace

FaultServiceManager::FaultServiceManager(
  rclcpp::Node & node, const InjectorMap & injectors,
  FaultEventPublisher & events, ReloadScenarioCallback reload_scenario_callback)
: node_(node), injectors_(injectors), events_(events),
  reload_scenario_callback_(reload_scenario_callback)
{
  set_fault_state_service_ = node_.create_service<srv::SetFaultState>(
        "fault_injection/set_fault_state",
    [this](const std::shared_ptr<srv::SetFaultState::Request> request,
    std::shared_ptr<srv::SetFaultState::Response> response)
    {
      handle_set_fault_state(request, response);
        });

  list_faults_service_ = node_.create_service<srv::ListFaults>(
        "fault_injection/list_faults",
    [this](const std::shared_ptr<srv::ListFaults::Request> request,
    std::shared_ptr<srv::ListFaults::Response> response)
    {handle_list_faults(request, response);});

  get_fault_status_service_ = node_.create_service<srv::GetFaultStatus>(
        "fault_injection/get_fault_status",
    [this](const std::shared_ptr<srv::GetFaultStatus::Request> request,
    std::shared_ptr<srv::GetFaultStatus::Response> response)
    {
      handle_get_fault_status(request, response);
        });
  set_fault_config_service_ = node_.create_service<srv::SetFaultConfig>(
        "fault_injection/set_fault_config",
    [this](const std::shared_ptr<srv::SetFaultConfig::Request> request,
    std::shared_ptr<srv::SetFaultConfig::Response> response)
    {
      handle_set_fault_config(request, response);
        });
  reload_scenario_service_ = node_.create_service<srv::ReloadScenario>(
        "fault_injection/reload_scenario",
    [this](const std::shared_ptr<srv::ReloadScenario::Request> request,
    std::shared_ptr<srv::ReloadScenario::Response> response)
    {handle_reload_scenario(request, response);});

  get_fault_schema_service_ = node_.create_service<srv::GetFaultSchema>(
        "fault_injection/get_fault_schema",
    [this](const std::shared_ptr<srv::GetFaultSchema::Request> request,
    std::shared_ptr<srv::GetFaultSchema::Response> response)
    {handle_get_fault_schema(request, response);});

  get_fault_config_service_ = node_.create_service<srv::GetFaultConfig>(
        "fault_injection/get_fault_config",
    [this](const std::shared_ptr<srv::GetFaultConfig::Request> request,
    std::shared_ptr<srv::GetFaultConfig::Response> response)
    {handle_get_fault_config(request, response);});
}

std::shared_ptr<FaultInjector> FaultServiceManager::find_injector_for_fault(
  const std::string & fault_id) const
{
  for (const auto &[injector_id, injector] : injectors_) {
    (void)injector_id;
    if (injector->has_fault(fault_id)) {
      return injector;
    }
  }

  return nullptr;
}

void FaultServiceManager::handle_reload_scenario(
  const std::shared_ptr<srv::ReloadScenario::Request> request,
  std::shared_ptr<srv::ReloadScenario::Response> response)
{
  (void)request;

  const auto result = reload_scenario_callback_();

  response->success = result.success;
  response->message = result.message;
}

void FaultServiceManager::handle_get_fault_status(
  const std::shared_ptr<srv::GetFaultStatus::Request> request,
  std::shared_ptr<srv::GetFaultStatus::Response> response)
{
  (void)request;

  response->faults.clear();

  for (const auto &[injector_id, injector] : injectors_) {
    const auto fault_ids = injector->fault_ids();
    const auto active_fault_ids = injector->active_fault_ids();
    const std::unordered_set<std::string> active_faults(active_fault_ids.begin(),
      active_fault_ids.end());

    for (const auto & fault_id : fault_ids) {
      ros2_fault_injection::msg::FaultStatus status;
      status.fault_id = fault_id;
      status.injector_id = injector_id;
      status.state = active_faults.count(fault_id) > 0 ? "active" : "inactive";

      const auto fault_config = injector->get_fault_config(fault_id);
      status.details =
        fault_config.has_value() ? describe_fault(fault_config.value()) : "config unavailable";

      response->faults.push_back(status);
    }
  }
}

void FaultServiceManager::handle_set_fault_state(
  const std::shared_ptr<srv::SetFaultState::Request> request,
  std::shared_ptr<srv::SetFaultState::Response> response)
{
  if (request->fault_id.empty()) {
    response->success = false;
    response->message = "fault_id cannot be empty";
    RCLCPP_WARN(node_.get_logger(), "Received request with empty fault_id");
    return;
  }

  const auto injector = find_injector_for_fault(request->fault_id);
  if (!injector) {
    response->success = false;
    response->message = "unknown fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(), "Received request with unknown fault_id: %s",
                  request->fault_id.c_str());
    return;
  }

  const auto fault_config = injector->get_fault_config(request->fault_id);
  if (!fault_config.has_value()) {
    response->success = false;
    response->message = "unable to retrieve config for fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(),
                  "Unable to retrieve config for fault_id: %s when handling request",
                  request->fault_id.c_str());
    return;
  }

  if (request->active) {
    injector->activate_fault(request->fault_id);
    response->message = "activated fault: " + request->fault_id;
    FaultEvent event;
    event.fault_id = request->fault_id;
    event.injector_id = injector->id();
    event.state = "active";
    event.source = "manual";
    event.details = describe_fault(fault_config.value());
    events_.publish(event);
    RCLCPP_INFO(node_.get_logger(), "Activated fault: %s", request->fault_id.c_str());
  } else {
    injector->deactivate_fault(request->fault_id);
    response->message = "deactivated fault: " + request->fault_id;
    FaultEvent event;
    event.fault_id = request->fault_id;
    event.injector_id = injector->id();
    event.state = "inactive";
    event.source = "manual";
    event.details = describe_fault(fault_config.value());
    events_.publish(event);
    RCLCPP_INFO(node_.get_logger(), "Deactivated fault: %s", request->fault_id.c_str());
  }

  response->success = true;
}

void FaultServiceManager::handle_list_faults(
  const std::shared_ptr<srv::ListFaults::Request> request,
  std::shared_ptr<srv::ListFaults::Response> response)
{
  (void)request;

  for (const auto &[injector_id, injector] : injectors_) {
    (void)injector_id;
    const auto fault_ids = injector->fault_ids();
    response->fault_ids.insert(response->fault_ids.end(), fault_ids.begin(), fault_ids.end());

    const auto active_fault_ids = injector->active_fault_ids();
    response->active_fault_ids.insert(response->active_fault_ids.end(), active_fault_ids.begin(),
                                        active_fault_ids.end());
  }
}

void FaultServiceManager::handle_set_fault_config(
  const std::shared_ptr<srv::SetFaultConfig::Request> request,
  std::shared_ptr<srv::SetFaultConfig::Response> response)
{
  if (request->fault_id.empty()) {
    response->success = false;
    response->message = "fault_id cannot be empty";
    RCLCPP_WARN(node_.get_logger(), "Received SetFaultConfig request with empty fault_id");
    return;
  }

  const auto injector = find_injector_for_fault(request->fault_id);
  if (!injector) {
    response->success = false;
    response->message = "unknown fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(), "Received SetFaultConfig request with unknown fault_id: %s",
                  request->fault_id.c_str());
    return;
  }

  const auto fault_config = injector->get_fault_config(request->fault_id);
  if (!fault_config.has_value()) {
    response->success = false;
    response->message = "unable to retrieve config for fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(),
                  "Unable to retrieve config for fault_id: %s when handling SetFaultConfig request",
                  request->fault_id.c_str());
    return;
  }

  const auto schema = injector->config_schema();
  const auto * field = find_schema_field(schema, request->key);
  if (field == nullptr) {
    response->success = false;
    response->message =
      "key '" + request->key + "' is not valid for injector type '" + injector->type() + "'";
    RCLCPP_WARN(
      node_.get_logger(),
      "Rejected config update for fault '%s': invalid key '%s'",
      request->fault_id.c_str(), request->key.c_str());
    return;
  }

  const auto validation_error = validate_config_value(*field, request->value);
  if (validation_error.has_value()) {
    response->success = false;
    response->message = validation_error.value();
    RCLCPP_WARN(
      node_.get_logger(),
      "Rejected config update for fault '%s': %s",
      request->fault_id.c_str(), validation_error->c_str());
    return;
  }

  if (!injector->set_fault_config_value(request->fault_id, request->key, request->value)) {
    response->success = false;
    response->message =
      "failed to set config key '" + request->key + "' for fault_id '" + request->fault_id + "'";
    RCLCPP_ERROR(node_.get_logger(), "Failed to set config key '%s' for fault '%s'",
                   request->key.c_str(), request->fault_id.c_str());
    return;
  }

  response->success = true;
  response->message = "updated fault '" + request->fault_id + "' config '" + request->key +
    "' to '" + request->value + "'";
  FaultEvent event;
  event.fault_id = request->fault_id;
  event.injector_id = injector->id();
  event.state = "config_updated";
  event.source = "manual";
  event.details = describe_config_update(request->key, request->value);
  events_.publish(event);

  RCLCPP_INFO(node_.get_logger(), "Updated fault '%s' config '%s' to '%s'",
                request->fault_id.c_str(), request->key.c_str(), request->value.c_str());
}

void FaultServiceManager::handle_get_fault_schema(
  const std::shared_ptr<srv::GetFaultSchema::Request> request,
  std::shared_ptr<srv::GetFaultSchema::Response> response)
{
  const auto injector = find_injector_for_fault(request->fault_id);
  if (!injector) {
    response->success = false;
    response->message = "unknown fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(), "Received GetFaultSchema request with unknown fault_id: %s",
                  request->fault_id.c_str());
    return;
  }

  const auto injector_type = injector->type();

  response->success = true;
  response->message = "retrieved schema for fault_id: " + request->fault_id;
  response->injector_id = injector->id();
  response->injector_type = injector_type;

  const auto schema = injector->config_schema();

  response->keys.reserve(schema.size());
  for (const auto & field : schema) {
    response->keys.push_back(field.key);
  }

  std::sort(response->keys.begin(), response->keys.end());
}

void FaultServiceManager::handle_get_fault_config(
  const std::shared_ptr<srv::GetFaultConfig::Request> request,
  std::shared_ptr<srv::GetFaultConfig::Response> response)
{
  const auto injector = find_injector_for_fault(request->fault_id);
  if (!injector) {
    response->success = false;
    response->message = "unknown fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(), "Received GetFaultConfig request with unknown fault_id: %s",
                  request->fault_id.c_str());
    return;
  }

  const auto fault_config = injector->get_fault_config(request->fault_id);
  if (!fault_config.has_value()) {
    response->success = false;
    response->message = "unable to retrieve config for fault_id: " + request->fault_id;
    RCLCPP_WARN(node_.get_logger(),
                  "Unable to retrieve config for fault_id: %s when handling GetFaultConfig request",
                  request->fault_id.c_str());
    return;
  }

  response->success = true;
  response->message = "retrieved config for fault_id: " + request->fault_id;
  response->injector_id = injector->id();
  response->injector_type = injector->type();

  std::vector<std::string> keys;
  keys.reserve(fault_config->config.size());
  for (const auto &[key, _] : fault_config->config) {
    keys.push_back(key);
  }
  std::sort(keys.begin(), keys.end());

  response->keys.clear();
  response->values.clear();
  for (const auto & key : keys) {
    response->keys.push_back(key);
    response->values.push_back(fault_config->config.at(key));
  }
}
} // namespace ros2_fault_injection
