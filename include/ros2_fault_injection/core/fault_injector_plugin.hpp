// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_PLUGIN_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_PLUGIN_HPP_

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"

namespace ros2_fault_injection
{

/**
 * @brief Plugin wrapper interface for creating typed fault injectors.
 *
 * Fault injectors need a ROS node and scenario configuration when they are
 * constructed, so pluginlib loads small default-constructible factory plugins
 * instead of loading the injectors directly.
 */
class FaultInjectorPlugin
{
public:
  virtual ~FaultInjectorPlugin() = default;

  /**
   * @brief Injector type handled by this plugin.
   *
   * @return Type string used by `InjectorConfig::type`.
   */
  virtual std::string type() const = 0;

  /**
   * @brief Create a typed injector instance.
   *
   * @param node ROS node used by the injector.
   * @param config Injector configuration from the scenario.
   * @return Created fault injector.
   */
  virtual std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) = 0;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_INJECTOR_PLUGIN_HPP_
