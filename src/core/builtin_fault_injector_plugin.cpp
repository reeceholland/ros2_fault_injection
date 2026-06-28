// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/builtin_fault_injector_plugin.hpp"

#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "ros2_fault_injection/injectors/imu_fault_injector.hpp"
#include "ros2_fault_injection/injectors/joint_state_fault_injector.hpp"
#include "ros2_fault_injection/injectors/odom_fault_injector.hpp"
#include "ros2_fault_injection/injectors/scan_fault_injector.hpp"
#include "ros2_fault_injection/injectors/tf_fault_injector.hpp"
#include "ros2_fault_injection/injectors/trigger_service_fault_injector.hpp"

namespace ros2_fault_injection::core
{

std::string OdomFaultInjectorPlugin::type() const
{
  return "odom";
}

std::shared_ptr<FaultInjector> OdomFaultInjectorPlugin::create(
  rclcpp::Node & node, const InjectorConfig & config)
{
  return std::make_shared<injectors::OdomFaultInjector>(node, config);
}

std::string ScanFaultInjectorPlugin::type() const
{
  return "scan";
}

std::shared_ptr<FaultInjector> ScanFaultInjectorPlugin::create(
  rclcpp::Node & node, const InjectorConfig & config)
{
  return std::make_shared<injectors::ScanFaultInjector>(node, config);
}

std::string JointStateFaultInjectorPlugin::type() const
{
  return "joint_state";
}

std::shared_ptr<FaultInjector> JointStateFaultInjectorPlugin::create(
  rclcpp::Node & node, const InjectorConfig & config)
{
  return std::make_shared<injectors::JointStateFaultInjector>(node, config);
}

std::string ImuFaultInjectorPlugin::type() const
{
  return "imu";
}

std::shared_ptr<FaultInjector> ImuFaultInjectorPlugin::create(
  rclcpp::Node & node, const InjectorConfig & config)
{
  return std::make_shared<injectors::ImuFaultInjector>(node, config);
}

std::string TfFaultInjectorPlugin::type() const
{
  return "tf";
}

std::shared_ptr<FaultInjector> TfFaultInjectorPlugin::create(
  rclcpp::Node & node, const InjectorConfig & config)
{
  return std::make_shared<injectors::TfFaultInjector>(node, config);
}

std::string TriggerServiceFaultInjectorPlugin::type() const
{
  return "trigger_service";
}

std::shared_ptr<FaultInjector> TriggerServiceFaultInjectorPlugin::create(
  rclcpp::Node & node, const InjectorConfig & config)
{
  return std::make_shared<injectors::TriggerServiceFaultInjector>(node, config);
}

}  // namespace ros2_fault_injection::core

PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::core::OdomFaultInjectorPlugin,
  ros2_fault_injection::core::FaultInjectorPlugin)
PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::core::ScanFaultInjectorPlugin,
  ros2_fault_injection::core::FaultInjectorPlugin)
PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::core::JointStateFaultInjectorPlugin,
  ros2_fault_injection::core::FaultInjectorPlugin)
PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::core::ImuFaultInjectorPlugin,
  ros2_fault_injection::core::FaultInjectorPlugin)
PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::core::TfFaultInjectorPlugin,
  ros2_fault_injection::core::FaultInjectorPlugin)
PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::core::TriggerServiceFaultInjectorPlugin,
  ros2_fault_injection::core::FaultInjectorPlugin)
