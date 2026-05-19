#ifndef ROS2_FAULT_INJECTION__FAULT_SCHEDULER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_SCHEDULER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/fault_injector.hpp"

namespace ros2_fault_injection
{

  class FaultScheduler
  {
  public:
    explicit FaultScheduler(rclcpp::Node &node);

    void schedule(
        const std::vector<FaultConfig> &faults,
        FaultInjector &injector,
        const std::vector<std::string> &initially_active_faults);

  private:
    bool is_initially_active(
        const FaultConfig &fault,
        const std::vector<std::string> &initially_active_faults) const;

    void schedule_start(FaultInjector &injector, const FaultConfig &fault);
    void schedule_stop(FaultInjector &injector, const FaultConfig &fault);

    rclcpp::Node &node_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  };

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_SCHEDULER_HPP_
