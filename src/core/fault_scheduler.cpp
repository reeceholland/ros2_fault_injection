// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/fault_scheduler.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>

#include "ros2_fault_injection/utils/fault_descriptions.hpp"

namespace ros2_fault_injection
{

FaultScheduler::FaultScheduler(rclcpp::Node & node, FaultEventPublisher & event_pub)
: node_(node), event_pub_(event_pub) {}

void FaultScheduler::schedule(
  const std::vector<FaultConfig> & faults, FaultInjector & injector,
  const std::vector<std::string> & initially_active_faults)
{
  for (const auto & fault : faults) {
    if (is_initially_active(fault, initially_active_faults)) {
      injector.activate_fault(fault.id);
      FaultEvent event;
      event.fault_id = fault.id;
      event.state = "active";
      event.injector_id = injector.id();
      event.source = "startup";
      event.details = describe_config(fault.config);
      publish_event(event);

      RCLCPP_INFO(node_.get_logger(), "Activated fault '%s' immediately", fault.id.c_str());

      if (fault.duration) {
        schedule_stop_after(injector, fault, *fault.duration);
      }

      continue;
    }

    if (!fault.start) {
      RCLCPP_INFO(node_.get_logger(),
                  "Fault '%s' has no start time; leaving it inactive for manual control",
                  fault.id.c_str());
      continue;
    }

    schedule_start(injector, fault);

    if (fault.duration) {
      schedule_stop(injector, fault);
    }
  }
}

bool FaultScheduler::is_initially_active(
  const FaultConfig & fault, const std::vector<std::string> & initially_active_faults) const
{
  return std::find(initially_active_faults.begin(), initially_active_faults.end(), fault.id) !=
         initially_active_faults.end();
}

void FaultScheduler::schedule_start(FaultInjector & injector, const FaultConfig & fault)
{
  const auto start_delay = std::max(*fault.start, std::chrono::milliseconds{1});

  auto timer_holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();

  *timer_holder = node_.create_wall_timer(start_delay, [this, &injector, fault, timer_holder]() {
        (*timer_holder)->cancel();

        injector.activate_fault(fault.id);

        FaultEvent event;
        event.fault_id = fault.id;
        event.state = "active";
        event.injector_id = injector.id();
        event.source = "scheduled";
        event.details = describe_config(fault.config);
        publish_event(event);

        RCLCPP_INFO(node_.get_logger(), "Activated scheduled fault '%s'", fault.id.c_str());
  });

  timers_.push_back(*timer_holder);
}

void FaultScheduler::schedule_stop(FaultInjector & injector, const FaultConfig & fault)
{
  schedule_stop_after(injector, fault, *fault.start + *fault.duration);
}

void FaultScheduler::schedule_stop_after(
  FaultInjector & injector, const FaultConfig & fault,
  std::chrono::milliseconds delay)
{
  const auto stop_delay = std::max(delay, std::chrono::milliseconds{1});

  auto timer_holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();

  *timer_holder = node_.create_wall_timer(stop_delay, [this, &injector, fault, timer_holder]() {
        (*timer_holder)->cancel();

        injector.deactivate_fault(fault.id);

        FaultEvent event;
        event.fault_id = fault.id;
        event.state = "inactive";
        event.injector_id = injector.id();
        event.source = "scheduled";
        event.details = describe_config(fault.config);
        publish_event(event);

        RCLCPP_INFO(node_.get_logger(), "Deactivated scheduled fault '%s'", fault.id.c_str());
  });

  timers_.push_back(*timer_holder);
}

void FaultScheduler::publish_event(const FaultEvent & event)
{
  event_pub_.publish(event);
}

}  // namespace ros2_fault_injection
