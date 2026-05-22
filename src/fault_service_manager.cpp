#include "ros2_fault_injection/fault_service_manager.hpp"

#include <string>
#include <unordered_set>

#include "ros2_fault_injection/fault_config_schema.hpp"
#include "ros2_fault_injection/fault_descriptions.hpp"
#include "ros2_fault_injection/msg/fault_status.hpp"

namespace ros2_fault_injection {

FaultServiceManager::FaultServiceManager(rclcpp::Node& node, const InjectorMap& injectors,
                                         FaultEventPublisher& events)
    : node_(node), injectors_(injectors), events_(events) {
  set_fault_state_service_ = node_.create_service<srv::SetFaultState>(
      "fault_injection/set_fault_state",
      [this](const std::shared_ptr<srv::SetFaultState::Request> request,
             std::shared_ptr<srv::SetFaultState::Response> response) {
        handle_set_fault_state(request, response);
      });

  list_faults_service_ = node_.create_service<srv::ListFaults>(
      "fault_injection/list_faults", [this](const std::shared_ptr<srv::ListFaults::Request> request,
                                            std::shared_ptr<srv::ListFaults::Response> response) {
        handle_list_faults(request, response);
      });

  get_fault_status_service_ = node_.create_service<srv::GetFaultStatus>(
      "fault_injection/get_fault_status",
      [this](const std::shared_ptr<srv::GetFaultStatus::Request> request,
             std::shared_ptr<srv::GetFaultStatus::Response> response) {
        handle_get_fault_status(request, response);
      });
  set_fault_config_service_ = node_.create_service<srv::SetFaultConfig>(
      "fault_injection/set_fault_config",
      [this](const std::shared_ptr<srv::SetFaultConfig::Request> request,
             std::shared_ptr<srv::SetFaultConfig::Response> response) {
        handle_set_fault_config(request, response);
      });
}

std::shared_ptr<FaultInjector> FaultServiceManager::find_injector_for_fault(
    const std::string& fault_id) const {
  for (const auto& [injector_id, injector] : injectors_) {
    (void)injector_id;
    if (injector->has_fault(fault_id)) {
      return injector;
    }
  }

  return nullptr;
}

void FaultServiceManager::handle_get_fault_status(
    const std::shared_ptr<srv::GetFaultStatus::Request> request,
    std::shared_ptr<srv::GetFaultStatus::Response> response) {
  (void)request;

  response->faults.clear();

  for (const auto& [injector_id, injector] : injectors_) {
    const auto fault_ids = injector->fault_ids();
    const auto active_fault_ids = injector->active_fault_ids();
    const std::unordered_set<std::string> active_faults(active_fault_ids.begin(),
                                                        active_fault_ids.end());

    for (const auto& fault_id : fault_ids) {
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
    std::shared_ptr<srv::SetFaultState::Response> response) {
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
    std::shared_ptr<srv::ListFaults::Response> response) {
  (void)request;

  for (const auto& [injector_id, injector] : injectors_) {
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
    std::shared_ptr<srv::SetFaultConfig::Response> response) {
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

  if (!is_allowed_config_key(fault_config->injector_id, request->key)) {
    response->success = false;
    response->message = "key '" + request->key + "' is not valid for injector_id '" +
                        fault_config->injector_id + "'";
    RCLCPP_WARN(node_.get_logger(), "Rejected config update for fault '%s': invalid key '%s'",
                request->fault_id.c_str(), request->key.c_str());
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

}  // namespace ros2_fault_injection
