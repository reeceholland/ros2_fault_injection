// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__SCENARIO_CONFIG_HPP_
#define ROS2_FAULT_INJECTION__SCENARIO_CONFIG_HPP_

#include <string>
#include <vector>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/assertions/assertion_config.hpp"

namespace ros2_fault_injection::config
{

  /**
   * @brief Parsed YAML scenario.
   */
struct ScenarioConfig
{
    /// All injectors configured by the scenario.
  std::vector<InjectorConfig> injectors;

    /// Legacy single-injector field kept for compatibility with older examples.
  InjectorConfig injector;

    /// All faults configured by the scenario.
  std::vector<FaultConfig> faults;

    /// Fault ids requested active at startup.
  std::vector<std::string> initially_active_faults;

    /// Assertions configured by the scenario.
  std::vector<assertions::AssertionConfig> assertions;
};

  /**
   * @brief Load a scenario YAML file.
   *
   * @param path Filesystem path to a scenario YAML file.
   * @return Parsed scenario configuration.
   * @throws std::exception when the file cannot be parsed.
   */
ScenarioConfig load_scenario_config(const std::string & path);

  /**
   * @brief Find an injector config by id.
   *
   * @param scenario Scenario to search.
   * @param injector_id Injector id to find.
   * @return Pointer to the injector config, or nullptr when absent.
   */
const InjectorConfig * find_injector(
  const ScenarioConfig & scenario,
  const std::string & injector_id);

} // namespace ros2_fault_injection::config


namespace ros2_fault_injection
{
using config::ScenarioConfig;
using config::find_injector;
using config::load_scenario_config;
}  // namespace ros2_fault_injection
#endif // ROS2_FAULT_INJECTION__SCENARIO_CONFIG_HPP_
