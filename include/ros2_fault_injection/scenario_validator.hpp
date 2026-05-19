#ifndef ROS2_FAULT_INJECTION__SCENARIO_VALIDATOR_HPP_
#define ROS2_FAULT_INJECTION__SCENARIO_VALIDATOR_HPP_

#include <string>
#include <vector>

#include "ros2_fault_injection/scenario_config.hpp"

namespace ros2_fault_injection {

struct ValidationResult {
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  bool ok() const {
    return errors.empty();
  }
};

ValidationResult validate_scenario(const ScenarioConfig& scenario);

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__SCENARIO_VALIDATOR_HPP_