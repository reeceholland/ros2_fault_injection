#ifndef ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include "ros2_fault_injection/fault_event_publisher.hpp"
#include "ros2_fault_injection/fault_injector.hpp"
#include "ros2_fault_injection/srv/list_faults.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"

namespace ros2_fault_injection {

class FaultServiceManager {
public:
  using InjectorMap = std::unordered_map<std::string, std::shared_ptr<FaultInjector>>;

  FaultServiceManager(rclcpp::Node& node, const InjectorMap& injectors,
                      FaultEventPublisher& events);

private:
  std::shared_ptr<FaultInjector> find_injector_for_fault(const std::string& fault_id) const;
  void handle_set_fault_state(const std::shared_ptr<srv::SetFaultState::Request> request,
                              std::shared_ptr<srv::SetFaultState::Response> response);
  void handle_list_faults(const std::shared_ptr<srv::ListFaults::Request> request,
                          std::shared_ptr<srv::ListFaults::Response> response);
  rclcpp::Node& node_;
  const InjectorMap& injectors_;
  FaultEventPublisher& events_;

  rclcpp::Service<srv::SetFaultState>::SharedPtr set_fault_state_service_;
  rclcpp::Service<srv::ListFaults>::SharedPtr list_faults_service_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_