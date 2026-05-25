// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/fault_injector_factory.hpp"

#include <memory>
#include <string>

#include <pluginlib/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_fault_injection
{

FaultInjectorFactory::FaultInjectorFactory(rclcpp::Node & node)
: node_(node),
  loader_("ros2_fault_injection", "ros2_fault_injection::FaultInjectorPlugin")
{
  load_plugins();
}

void FaultInjectorFactory::load_plugins()
{
  for (const auto & plugin_name : loader_.getDeclaredClasses()) {
    try {
      const auto plugin = loader_.createSharedInstance(plugin_name);
      plugin_names_by_type_[plugin->type()] = plugin_name;
      RCLCPP_DEBUG(
        node_.get_logger(), "Loaded fault injector plugin '%s' for type '%s'",
        plugin_name.c_str(), plugin->type().c_str());
    } catch (const pluginlib::PluginlibException & error) {
      RCLCPP_WARN(
        node_.get_logger(), "Failed to load fault injector plugin '%s': %s",
        plugin_name.c_str(), error.what());
    }
  }
}

std::shared_ptr<FaultInjector> FaultInjectorFactory::create(const InjectorConfig & config)
{
  const auto plugin_it = plugin_names_by_type_.find(config.type);
  if (plugin_it == plugin_names_by_type_.end()) {
    return nullptr;
  }

  try {
    const auto plugin = loader_.createSharedInstance(plugin_it->second);
    return plugin->create(node_, config);
  } catch (const pluginlib::PluginlibException & error) {
    RCLCPP_ERROR(
      node_.get_logger(), "Failed to create fault injector plugin for type '%s': %s",
      config.type.c_str(), error.what());
  }

  return nullptr;
}

}  // namespace ros2_fault_injection
