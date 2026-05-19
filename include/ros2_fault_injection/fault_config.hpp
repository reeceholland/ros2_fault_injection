#ifndef ROS2_FAULT_INJECTION__FAULT_CONFIG_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONFIG_HPP_

#include <chrono>
#include <cstddef>
#include <string>
#include <optional>
#include <unordered_map>

namespace ros2_fault_injection
{

  struct InjectorConfig
  {
    std::string id;
    std::string type;
    std::string input_topic;
    std::string output_topic;
    size_t qos_depth{10};
  };

  struct FaultConfig
  {
    std::string id;
    std::string injector_id;
    std::optional<std::chrono::milliseconds> start;
    std::optional<std::chrono::milliseconds> duration;
    std::unordered_map<std::string, std::string> config;
  };
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_CONFIG_HPP_