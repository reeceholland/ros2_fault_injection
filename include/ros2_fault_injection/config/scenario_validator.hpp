// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__SCENARIO_VALIDATOR_HPP_
#define ROS2_FAULT_INJECTION__SCENARIO_VALIDATOR_HPP_

#include <string>
#include <vector>

#include "ros2_fault_injection/config/scenario_config.hpp"

namespace ros2_fault_injection
{

/**
 * @brief Result of scenario validation.
 */
struct ValidationResult
{
  /// Validation failures that should prevent startup.
  std::vector<std::string> errors;

  /// Non-fatal validation findings.
  std::vector<std::string> warnings;

  /**
   * @brief Check whether validation found any errors.
   *
   * @return true when `errors` is empty.
   */
  bool ok() const
  {
    return errors.empty();
  }
};

/**
 * @brief Validate a parsed scenario before starting injectors.
 *
 * @param scenario Scenario to validate.
 * @return Validation errors and warnings.
 */
ValidationResult validate_scenario(const ScenarioConfig & scenario);

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__SCENARIO_VALIDATOR_HPP_
