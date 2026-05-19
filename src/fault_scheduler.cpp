#include "ros2_fault_injection/fault_scheduler.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

namespace ros2_fault_injection
{

  FaultScheduler::FaultScheduler(rclcpp::Node &node, FaultEventPublisher &event_pub)
      : node_(node), event_pub_(event_pub)
  {
  }

  void FaultScheduler::schedule(
      const std::vector<FaultConfig> &faults,
      FaultInjector &injector,
      const std::vector<std::string> &initially_active_faults)
  {
    for (const auto &fault : faults)
    {
      if (is_initially_active(fault, initially_active_faults))
      {
        injector.activate_fault(fault.id);
        publish_event(fault.id, "active");

        RCLCPP_INFO(
            node_.get_logger(),
            "Activated fault '%s' immediately",
            fault.id.c_str());

        continue;
      }

      schedule_start(injector, fault);

      if (fault.duration)
      {
        schedule_stop(injector, fault);
      }
    }
  }

  bool FaultScheduler::is_initially_active(
      const FaultConfig &fault,
      const std::vector<std::string> &initially_active_faults) const
  {
    return std::find(
               initially_active_faults.begin(),
               initially_active_faults.end(),
               fault.id) != initially_active_faults.end();
  }

  void FaultScheduler::schedule_start(
      FaultInjector &injector,
      const FaultConfig &fault)
  {
    const auto start_delay =
        std::max(fault.start, std::chrono::milliseconds{1});

    auto timer_holder =
        std::make_shared<rclcpp::TimerBase::SharedPtr>();

    *timer_holder = node_.create_wall_timer(
        start_delay,
        [this, &injector, fault, timer_holder]()
        {
          (*timer_holder)->cancel();

          injector.activate_fault(fault.id);

          publish_event(fault.id, "active");

          RCLCPP_INFO(
              node_.get_logger(),
              "Activated scheduled fault '%s'",
              fault.id.c_str());
        });

    timers_.push_back(*timer_holder);
  }

  void FaultScheduler::schedule_stop(
      FaultInjector &injector,
      const FaultConfig &fault)
  {
    const auto stop_delay =
        std::max(fault.start + *fault.duration, std::chrono::milliseconds{1});

    auto timer_holder =
        std::make_shared<rclcpp::TimerBase::SharedPtr>();

    *timer_holder = node_.create_wall_timer(
        stop_delay,
        [this, &injector, fault, timer_holder]()
        {
          (*timer_holder)->cancel();

          injector.deactivate_fault(fault.id);

          publish_event(fault.id, "inactive");

          RCLCPP_INFO(
              node_.get_logger(),
              "Deactivated scheduled fault '%s'",
              fault.id.c_str());
        });

    timers_.push_back(*timer_holder);
  }

  void FaultScheduler::publish_event(
      const std::string &fault_id,
      const std::string &state)
  {
    event_pub_.publish(fault_id, state);
  }

} // namespace ros2_fault_injection
