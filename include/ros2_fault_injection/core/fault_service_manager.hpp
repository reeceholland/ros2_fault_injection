// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include "ros2_fault_injection/core/fault_controller.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_event_recorder.hpp"
#include "ros2_fault_injection/srv/get_fault_status.hpp"
#include "ros2_fault_injection/srv/list_faults.hpp"
#include "ros2_fault_injection/srv/reload_scenario.hpp"
#include "ros2_fault_injection/srv/set_fault_config.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"
#include "ros2_fault_injection/srv/get_fault_schema.hpp"
#include "ros2_fault_injection/srv/get_fault_config.hpp"
#include "ros2_fault_injection/srv/get_scenario.hpp"
#include "ros2_fault_injection/srv/request_report.hpp"

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
  struct ReportResult
  {
    bool success{false};
    std::string message;
    std::string scenario_file;
    std::string final_result;
    std::string report_markdown;
  };
    /// Map of injector id to injector instance.
  using InjectorMap = std::unordered_map<std::string, std::shared_ptr<FaultInjector>>;

    /// Callback used by the reload service to ask the controller to reload its scenario.
  using ReloadScenarioCallback = std::function<ReloadScenarioResult()>;
  using ScenarioFileProvider = std::function<std::string()>;
  using ScenarioContentProvider = std::function<std::optional<std::string>()>;
  using RequestReportCallback = std::function<ReportResult()>;

    /**
     * @brief Create all fault control services.
     *
     * @param node Node used to create services.
     * @param injectors Runtime injectors indexed by id.
     * @param events Publisher used for manual state/config events.
     * @param reload_scenario_callback Callback used by the reload service.
     * @param scenario_provider Callback used to provide the scenario file path.
     * @param scenario_content_provider Callback used to provide the scenario content.
     */
  FaultServiceManager(
    rclcpp::Node & node, const InjectorMap & injectors,
    FaultEventPublisher & events, core::FaultEventRecorder & fault_event_recorder,
    ReloadScenarioCallback reload_scenario_callback,
    ScenarioFileProvider scenario_provider, ScenarioContentProvider scenario_content_provider,
    RequestReportCallback request_report_callback);

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

  void handle_get_fault_schema(
    const std::shared_ptr<srv::GetFaultSchema::Request> request,
    std::shared_ptr<srv::GetFaultSchema::Response> response);

  void handle_get_fault_config(
    const std::shared_ptr<srv::GetFaultConfig::Request> request,
    std::shared_ptr<srv::GetFaultConfig::Response> response);

  void handle_get_scenario(
    const std::shared_ptr<srv::GetScenario::Request> request,
    std::shared_ptr<srv::GetScenario::Response> response);

  void handle_request_report(
    const std::shared_ptr<srv::RequestReport::Request> request,
    std::shared_ptr<srv::RequestReport::Response> response);

  rclcpp::Node & node_;
  const InjectorMap & injectors_;
  FaultEventPublisher & events_;
  ReloadScenarioCallback reload_scenario_callback_;
  ScenarioFileProvider scenario_provider_;
  ScenarioContentProvider scenario_content_provider_;
  RequestReportCallback request_report_callback_;
  rclcpp::Service<srv::SetFaultState>::SharedPtr set_fault_state_service_;
  rclcpp::Service<srv::ListFaults>::SharedPtr list_faults_service_;
  rclcpp::Service<srv::GetFaultStatus>::SharedPtr get_fault_status_service_;
  rclcpp::Service<srv::SetFaultConfig>::SharedPtr set_fault_config_service_;
  rclcpp::Service<srv::ReloadScenario>::SharedPtr reload_scenario_service_;
  rclcpp::Service<srv::GetFaultSchema>::SharedPtr get_fault_schema_service_;
  rclcpp::Service<srv::GetFaultConfig>::SharedPtr get_fault_config_service_;
  rclcpp::Service<srv::GetScenario>::SharedPtr get_scenario_service_;
  rclcpp::Service<srv::RequestReport>::SharedPtr request_report_service_;

  core::FaultEventRecorder & fault_event_recorder_;
};

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_SERVICE_MANAGER_HPP_
