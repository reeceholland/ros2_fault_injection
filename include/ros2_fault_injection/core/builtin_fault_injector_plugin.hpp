// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__BUILTIN_FAULT_INJECTOR_PLUGIN_HPP_
#define ROS2_FAULT_INJECTION__BUILTIN_FAULT_INJECTOR_PLUGIN_HPP_

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_injector_plugin.hpp"

namespace ros2_fault_injection::core
{

class OdomFaultInjectorPlugin : public FaultInjectorPlugin
{
public:
  std::string type() const override;
  std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) override;
};

class ScanFaultInjectorPlugin : public FaultInjectorPlugin
{
public:
  std::string type() const override;
  std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) override;
};

class JointStateFaultInjectorPlugin : public FaultInjectorPlugin
{
public:
  std::string type() const override;
  std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) override;
};

class ImuFaultInjectorPlugin : public FaultInjectorPlugin
{
public:
  std::string type() const override;
  std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) override;
};

class TfFaultInjectorPlugin : public FaultInjectorPlugin
{
public:
  std::string type() const override;
  std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) override;
};

class TriggerServiceFaultInjectorPlugin : public FaultInjectorPlugin
{
public:
  std::string type() const override;
  std::shared_ptr<FaultInjector> create(
    rclcpp::Node & node, const InjectorConfig & config) override;
};

}  // namespace ros2_fault_injection::core

namespace ros2_fault_injection
{
using core::ImuFaultInjectorPlugin;
using core::JointStateFaultInjectorPlugin;
using core::OdomFaultInjectorPlugin;
using core::ScanFaultInjectorPlugin;
using core::TfFaultInjectorPlugin;
using core::TriggerServiceFaultInjectorPlugin;
}  // namespace ros2_fault_injection
#endif  // ROS2_FAULT_INJECTION__BUILTIN_FAULT_INJECTOR_PLUGIN_HPP_
