#ifndef ROS2_FAULT_INJECTION__FAULT_SCHEDULER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_SCHEDULER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"

namespace ros2_fault_injection {

/**
 * @brief Schedules startup and timed fault activation.
 *
 * The scheduler is message-type agnostic. It activates/deactivates faults
 * through the FaultInjector interface and publishes lifecycle events.
 */
class FaultScheduler {
public:
  /**
   * @brief Construct a scheduler.
   *
   * @param node Node used to create wall timers.
   * @param event_pub Publisher used for scheduled fault events.
   */
  explicit FaultScheduler(rclcpp::Node& node, FaultEventPublisher& event_pub);

  /**
   * @brief Schedule all faults belonging to one injector.
   *
   * @param faults Faults assigned to the injector.
   * @param injector Injector to activate and deactivate.
   * @param initially_active_faults Fault ids that should start active.
   */
  void schedule(const std::vector<FaultConfig>& faults, FaultInjector& injector,
                const std::vector<std::string>& initially_active_faults);

private:
  bool is_initially_active(const FaultConfig& fault,
                           const std::vector<std::string>& initially_active_faults) const;

  void schedule_start(FaultInjector& injector, const FaultConfig& fault);
  void schedule_stop(FaultInjector& injector, const FaultConfig& fault);
  void schedule_stop_after(FaultInjector& injector, const FaultConfig& fault,
                           std::chrono::milliseconds delay);

  void publish_event(const FaultEvent& event);

  rclcpp::Node& node_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  FaultEventPublisher& event_pub_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_SCHEDULER_HPP_
