#include "ros2_fault_injection/core/fault_injector_factory.hpp"

#include "ros2_fault_injection/injectors/imu_fault_injector.hpp"
#include "ros2_fault_injection/injectors/joint_state_fault_injector.hpp"
#include "ros2_fault_injection/injectors/odom_fault_injector.hpp"
#include "ros2_fault_injection/injectors/scan_fault_injector.hpp"
#include "ros2_fault_injection/injectors/trigger_service_fault_injector.hpp"

namespace ros2_fault_injection {

FaultInjectorFactory::FaultInjectorFactory(rclcpp::Node& node) : node_(node) {}

std::shared_ptr<FaultInjector> FaultInjectorFactory::create(const InjectorConfig& config) const {
  if (config.type == "odom") {
    return std::make_shared<OdomFaultInjector>(node_, config);
  }

  if (config.type == "scan") {
    return std::make_shared<ScanFaultInjector>(node_, config);
  }

  if (config.type == "joint_state") {
    return std::make_shared<JointStateFaultInjector>(node_, config);
  }

  if (config.type == "imu") {
    return std::make_shared<ImuFaultInjector>(node_, config);
  }

  if (config.type == "trigger_service") {
    return std::make_shared<TriggerServiceFaultInjector>(node_, config);
  }

  return nullptr;
}

}  // namespace ros2_fault_injection