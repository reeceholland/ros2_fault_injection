#include "ros2_fault_injection/fault_service_manager.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "ros2_fault_injection/msg/fault_status.hpp"

namespace ros2_fault_injection {

namespace {
std::string join_strings(const std::vector<std::string>& parts, const std::string& separator) {
  std::ostringstream out;

  for (size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) {
      out << separator;
    }

    out << parts[i];
  }

  return out.str();
}
}  // namespace

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

  if (request->active) {
    injector->activate_fault(request->fault_id);
    response->message = "activated fault: " + request->fault_id;
    events_.publish(request->fault_id, "active");
    RCLCPP_INFO(node_.get_logger(), "Activated fault: %s", request->fault_id.c_str());
  } else {
    injector->deactivate_fault(request->fault_id);
    response->message = "deactivated fault: " + request->fault_id;
    events_.publish(request->fault_id, "inactive");
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

std::string FaultServiceManager::describe_fault(const FaultConfig& fault) {
  // This function can be expanded to provide more detailed descriptions based on the fault
  // configuration
  std::vector<std::string> details;
  if (fault.start.has_value()) {
    details.push_back("scheduled");
    details.push_back("start: " + std::to_string(fault.start->count()) + "ms");
  } else {
    details.push_back("manual-only");
  }
  if (fault.duration.has_value()) {
    details.push_back("duration: " + std::to_string(fault.duration->count()) + "ms");
  }
  if (!fault.config.empty()) {
    std::vector<std::string> config_parts;
    for (const auto& [key, value] : fault.config) {
      config_parts.push_back(key + "=" + value);
    }
    details.push_back("config_keys={" + join_strings(config_parts, ", ") + "}");
  }

  return join_strings(details, ", ");
}

}  // namespace ros2_fault_injection
