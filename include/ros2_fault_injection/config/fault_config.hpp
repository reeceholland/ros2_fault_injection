#ifndef ROS2_FAULT_INJECTION__FAULT_CONFIG_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONFIG_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>

namespace ros2_fault_injection
{

struct TopicEndpointConfig
{
  std::string input_topic;
  std::string output_topic;
  size_t qos_depth{10};
};

struct TriggerServiceEndpointConfig
{
  std::string proxy_service;
  std::string target_service;
};

/**
 * @brief Runtime configuration for one topic injector.
 *
 * An injector acts as a proxy between an input topic and an output topic. The
 * `type` selects the message-specific injector implementation, while `id` is
 * used by faults to target that injector.
 */
struct InjectorConfig
{
  /// Unique injector identifier used by fault definitions.
  std::string id;

  /// Injector implementation type, for example `odom`, `scan`, `joint_state`, or `imu`.
  std::string type;

  std::optional<TopicEndpointConfig> topic;

  std::optional<TriggerServiceEndpointConfig> trigger_service;
};

/**
 * @brief Configuration for one injectable fault.
 *
 * The top-level fields describe scheduling and ownership. The `config` map
 * stores injector-specific fault parameters such as `x_bias`,
 * `range_noise_stddev`, or `delay_ms`.
 */
struct FaultConfig
{
  /// Unique fault identifier used by services, events, and schedules.
  std::string id;

  /// Identifier of the injector that owns this fault.
  std::string injector_id;

  /// Optional activation time relative to node startup.
  std::optional<std::chrono::milliseconds> start;

  /// Optional active window duration.
  std::optional<std::chrono::milliseconds> duration;

  /// Injector-specific fault parameters parsed from YAML.
  std::unordered_map<std::string, std::string> config;
};
}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_CONFIG_HPP_
