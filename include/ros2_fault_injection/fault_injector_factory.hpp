#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_

#include <memory>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/fault_injector.hpp"

namespace ros2_fault_injection {

class FaultInjectorFactory {
public:
  explicit FaultInjectorFactory(rclcpp::Node& node);

  std::shared_ptr<FaultInjector> create(const InjectorConfig& config) const;

private:
  rclcpp::Node& node_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_