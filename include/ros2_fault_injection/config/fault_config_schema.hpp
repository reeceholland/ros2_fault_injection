// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_

#include <optional>
#include <string>
#include <unordered_set>

namespace ros2_fault_injection
{

/**
 * @brief Get the allowed config keys for an injector type.
 *
 * @param injector_type Type name from an InjectorConfig, such as `odom` or `scan`.
 * @return Set of accepted fault config keys. Unknown injector types return an empty set.
 */
const std::unordered_set<std::string> & allowed_config_keys_for_injector_type(
  const std::string & injector_type);

/**
 * @brief Check whether a config key is valid for an injector type.
 *
 * @param injector_type Type name from an InjectorConfig.
 * @param key Fault config key to validate.
 * @return true when the key is accepted by the injector type.
 */
bool is_allowed_config_key(const std::string & injector_type, const std::string & key);


/**
 * @brief Validate one runtime config value for an injector type.
 *
 * This is used by runtime services before mutating a stored fault config. It
 * keeps service updates consistent with scenario validation.
 *
 * @param injector_type Type name from an InjectorConfig.
 * @param key Fault config key to validate.
 * @param value Proposed value stored as text.
 * @return std::nullopt when valid, otherwise a human-readable validation error.
 */
std::optional<std::string> validate_config_value(
  const std::string & injector_type,
  const std::string & key,
  const std::string & value);

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_
