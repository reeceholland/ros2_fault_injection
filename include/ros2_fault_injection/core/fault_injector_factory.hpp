// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_injector_plugin.hpp"

namespace ros2_fault_injection::core
{

/**
 * @brief Creates typed injectors from scenario configuration.
 */
class FaultInjectorFactory {
public:
  /**
   * @brief Construct a factory.
   *
   * @param node Node passed to created injectors.
   */
  explicit FaultInjectorFactory(rclcpp::Node & node);

  /**
   * @brief Create the injector implementation selected by `config.type`.
   *
   * @param config Injector configuration.
   * @return Shared injector instance, or nullptr when the type is unsupported.
   */
  std::shared_ptr<FaultInjector> create(const InjectorConfig & config);

private:
  void load_plugins();

  rclcpp::Node & node_;
  pluginlib::ClassLoader<FaultInjectorPlugin> loader_;
  std::unordered_map<std::string, std::string> plugin_names_by_type_;
};

}  // namespace ros2_fault_injection::core

namespace ros2_fault_injection
{
using core::FaultInjectorFactory;
}  // namespace ros2_fault_injection
#endif  // ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_
