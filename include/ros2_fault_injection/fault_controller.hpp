#ifndef ROS2_FAULT_INJECTION__FAULT_CONTROLLER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/fault_event_publisher.hpp"
#include "ros2_fault_injection/fault_injector.hpp"
#include "ros2_fault_injection/fault_injector_factory.hpp"
#include "ros2_fault_injection/fault_scheduler.hpp"
#include "ros2_fault_injection/scenario_config.hpp"

namespace ros2_fault_injection {
class FaultController {
public:
  FaultController(rclcpp::Node& node, const ScenarioConfig& scenario, FaultEventPublisher& events);

  const InjectorMap& injectors() const;

private:
  void create_injectors();
  void register_faults();
  void schedule_faults();

  rclcpp::Node& node_;
  const ScenarioConfig& scenario_;
  FaultEventPublisher& events_;
  FaultInjectorFactory factory_;
  FaultScheduler scheduler_;
  InjectorMap injectors_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_CONTROLLER_HPP_