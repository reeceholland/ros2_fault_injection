// include/ros2_fault_injection/fault_config_schema.hpp

#ifndef ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_

#include <string>
#include <unordered_set>

namespace ros2_fault_injection {

const std::unordered_set<std::string>& allowed_config_keys_for_injector_type(
    const std::string& injector_type);

bool is_allowed_config_key(const std::string& injector_type, const std::string& key);

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_CONFIG_SCHEMA_HPP_