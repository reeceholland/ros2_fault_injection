// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__TRIGGER_SERVICE_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__TRIGGER_SERVICE_FAULT_INJECTOR_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection::injectors
{
class TriggerServiceFaultInjector : public FaultInjectorBase {
public:
  explicit TriggerServiceFaultInjector(rclcpp::Node & node, const InjectorConfig & config);

  static std::vector<FaultConfigField> static_config_schema();

  std::vector<FaultConfigField> config_schema() const override;

private:
  void on_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};
}  // namespace ros2_fault_injection::injectors

namespace ros2_fault_injection
{
using injectors::TriggerServiceFaultInjector;
}  // namespace ros2_fault_injection
#endif  // ROS2_FAULT_INJECTION__TRIGGER_SERVICE_FAULT_INJECTOR_HPP_
