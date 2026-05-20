#include "ros2_fault_injection/fault_service_manager.hpp"

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

  for (const auto& [injector_id, injector] : injectors_) {
    (void)injector_id;
    const auto fault_ids = injector->fault_ids();
    const auto active_fault_ids = injector->active_fault_ids();

    for (const auto& fault_id : fault_ids) {
      response->fault_ids.push_back(fault_id);
      response->injector_ids.push_back(injector_id);
      if (std::find(active_fault_ids.begin(), active_fault_ids.end(), fault_id) !=
          active_fault_ids.end()) {
        response->states.push_back("active");
        response->details.push_back("Fault is currently active");
      } else {
        response->states.push_back("inactive");
        response->details.push_back("Fault is currently inactive");
      }
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

}  // namespace ros2_fault_injection
