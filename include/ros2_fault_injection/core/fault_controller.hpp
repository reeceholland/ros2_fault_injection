// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_CONTROLLER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_injector_factory.hpp"
#include "ros2_fault_injection/core/fault_scheduler.hpp"
#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/assertions/fault_assertion_runner.hpp"

namespace ros2_fault_injection
{

struct ReloadScenarioResult
{
  bool success{false};
  std::string message;
};

  /**
   * @brief Owns the runtime injector graph for one scenario.
   *
   * The controller creates typed injectors, registers each fault with the
   * injector named by `FaultConfig::injector_id`, and starts scheduled fault
   * activation.
   */
class FaultController
{
public:
    /**
     * @brief Construct and start the controller.
     *
     * @param node ROS node used to create injector publishers, subscriptions, and timers.
     * @param scenario_file Scenario YAML file path used for runtime reloads.
     * @param scenario Parsed and validated scenario configuration.
     * @param events Shared event publisher used by scheduled faults.
     */
  FaultController(
    rclcpp::Node & node, std::string scenario_file, ScenarioConfig scenario,
    FaultEventPublisher & events);

    /**
     * @brief Access the configured injectors by injector id.
     *
     * @return Map of injector id to injector instance.
     */
  const InjectorMap & injectors() const;

  ReloadScenarioResult reload_scenario();

private:
  void create_injectors();
  void register_faults();
  void schedule_faults();
  ReloadScenarioResult validate_reload_compatible(const ScenarioConfig & new_scenario) const;

  rclcpp::Node & node_;
  std::string scenario_file_;
  ScenarioConfig scenario_;
  FaultEventPublisher & events_;
  FaultInjectorFactory factory_;
  FaultScheduler scheduler_;
  InjectorMap injectors_;
  std::unique_ptr<FaultAssertionRunner> assertion_runner_;
};

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_CONTROLLER_HPP_
