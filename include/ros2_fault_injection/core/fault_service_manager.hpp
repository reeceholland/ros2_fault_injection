// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_controller.hpp"
#include "ros2_fault_injection/srv/get_fault_status.hpp"
#include "ros2_fault_injection/srv/list_faults.hpp"
#include "ros2_fault_injection/srv/set_fault_config.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"
#include "ros2_fault_injection/srv/reload_scenario.hpp"

namespace ros2_fault_injection
{

  /**
   * @brief Owns runtime control services for fault injection.
   *
   * The service manager exposes APIs for listing faults, reading status,
   * activating/deactivating faults, and changing runtime config values.
   */
class FaultServiceManager
{
public:
    /// Map of injector id to injector instance.
  using InjectorMap = std::unordered_map<std::string, std::shared_ptr<FaultInjector>>;
  using ReloadScenarioCallback = std::function<ReloadScenarioResult()>;

    /**
     * @brief Create all fault control services.
     *
     * @param node Node used to create services.
     * @param injectors Runtime injectors indexed by id.
     * @param events Publisher used for manual state/config events.
     */
  FaultServiceManager(
    rclcpp::Node & node, const InjectorMap & injectors,
    FaultEventPublisher & events, ReloadScenarioCallback reload_scenario_callback);

private:
  std::shared_ptr<FaultInjector> find_injector_for_fault(const std::string & fault_id) const;
  void handle_set_fault_state(
    const std::shared_ptr<srv::SetFaultState::Request> request,
    std::shared_ptr<srv::SetFaultState::Response> response);
  void handle_list_faults(
    const std::shared_ptr<srv::ListFaults::Request> request,
    std::shared_ptr<srv::ListFaults::Response> response);

  void handle_get_fault_status(
    const std::shared_ptr<srv::GetFaultStatus::Request> request,
    std::shared_ptr<srv::GetFaultStatus::Response> response);
  void handle_set_fault_config(
    const std::shared_ptr<srv::SetFaultConfig::Request> request,
    std::shared_ptr<srv::SetFaultConfig::Response> response);
  void handle_reload_scenario(
    const std::shared_ptr<srv::ReloadScenario::Request> request,
    std::shared_ptr<srv::ReloadScenario::Response> response);

  rclcpp::Node & node_;
  const InjectorMap & injectors_;
  FaultEventPublisher & events_;
  ReloadScenarioCallback reload_scenario_callback_;
  rclcpp::Service<srv::SetFaultState>::SharedPtr set_fault_state_service_;
  rclcpp::Service<srv::ListFaults>::SharedPtr list_faults_service_;
  rclcpp::Service<srv::GetFaultStatus>::SharedPtr get_fault_status_service_;
  rclcpp::Service<srv::SetFaultConfig>::SharedPtr set_fault_config_service_;
  rclcpp::Service<srv::ReloadScenario>::SharedPtr reload_scenario_service_;
};

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_
