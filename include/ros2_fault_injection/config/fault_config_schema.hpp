#ifndef ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_

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

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_
