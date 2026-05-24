#include "ros2_fault_injection/core/fault_controller.hpp"

namespace ros2_fault_injection
{

FaultController::FaultController(
  rclcpp::Node & node, const ScenarioConfig & scenario,
  FaultEventPublisher & events)
: node_(node), scenario_(scenario), events_(events), factory_(node), scheduler_(node, events)
{
  create_injectors();
  register_faults();
  schedule_faults();
}

const InjectorMap & FaultController::injectors() const
{
  return injectors_;
}

void FaultController::create_injectors()
{
  for (const auto & injector_config : scenario_.injectors) {
    auto injector = factory_.create(injector_config);

    if (!injector) {
      RCLCPP_ERROR(node_.get_logger(), "Unknown injector type '%s' for injector '%s'",
                   injector_config.type.c_str(), injector_config.id.c_str());

      continue;
    }

    injectors_[injector_config.id] = injector;
  }
}

void FaultController::register_faults()
{
  for (const auto & fault : scenario_.faults) {
    const auto injector_it = injectors_.find(fault.injector_id);

    if (injector_it == injectors_.end()) {
      RCLCPP_WARN(node_.get_logger(), "Fault '%s' targets unknown injector_id '%s', skipping",
                  fault.id.c_str(), fault.injector_id.c_str());

      continue;
    }

    injector_it->second->add_fault(fault);
  }
}

void FaultController::schedule_faults()
{
  for (const auto & [injector_id, injector] : injectors_) {
    std::vector<FaultConfig> targeted_faults;

    for (const auto & fault : scenario_.faults) {
      if (fault.injector_id == injector_id) {
        targeted_faults.push_back(fault);
      }
    }

    scheduler_.schedule(targeted_faults, *injector, scenario_.initially_active_faults);
  }
}

}  // namespace ros2_fault_injection
