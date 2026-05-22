#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_

#include <memory>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/fault_injector.hpp"

namespace ros2_fault_injection {

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
  explicit FaultInjectorFactory(rclcpp::Node& node);

  /**
   * @brief Create the injector implementation selected by `config.type`.
   *
   * @param config Injector configuration.
   * @return Shared injector instance, or nullptr when the type is unsupported.
   */
  std::shared_ptr<FaultInjector> create(const InjectorConfig& config) const;

private:
  rclcpp::Node& node_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_INJECTOR_FACTORY_HPP_
