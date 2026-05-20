#ifndef ROS2_FAULT_INJECTION__SCENARIO_CONFIG_HPP_
#define ROS2_FAULT_INJECTION__SCENARIO_CONFIG_HPP_

#include <string>
#include <vector>

#include "ros2_fault_injection/fault_config.hpp"

namespace ros2_fault_injection {

struct ScenarioConfig {
  std::vector<InjectorConfig> injectors;
  InjectorConfig injector;
  std::vector<FaultConfig> faults;
  std::vector<std::string> initially_active_faults;
};

ScenarioConfig load_scenario_config(const std::string& path);

const InjectorConfig* find_injector(const ScenarioConfig& scenario, const std::string& injector_id);

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__SCENARIO_CONFIG_HPP_