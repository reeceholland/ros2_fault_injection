// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTIONS__FAULT_ASSERTION_RUNNER_HPP_
#define ROS2_FAULT_INJECTION__ASSERTIONS__FAULT_ASSERTION_RUNNER_HPP_

#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/generic_subscription.hpp"

#include "ros2_fault_injection/assertions/topic_hz_assertion.hpp"
#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/fault_event_assertion.hpp"
#include "ros2_fault_injection/msg/assertion_event.hpp"

namespace ros2_fault_injection::assertions
{
class FaultAssertionRunner
{
public:
  FaultAssertionRunner(rclcpp::Node & node);

  void start(const std::vector<AssertionConfig> & assertions);
  std::vector<AssertionResult> results() const;

private:
  void on_fault_event(const msg::FaultEvent & event);
  void update();
  void publish_state_change();
  void publish_assertion_event();

  rclcpp::Node & node_;
  rclcpp::Time start_time_;

  std::vector<TopicHzAssertion> topic_hz_assertions_;
  std::vector<FaultEventAssertion> fault_event_assertions_;
  std::unordered_map<std::string, AssertionState> last_published_states_;

  rclcpp::Subscription<msg::FaultEvent>::SharedPtr fault_event_subscription_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> topic_hz_subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<msg::AssertionEvent>::SharedPtr assertion_event_publisher_;
};
} // namespace ros2_fault_injection::assertions

#endif // ROS2_FAULT_INJECTION__ASSERTIONS__FAULT_ASSERTION_RUNNER_HPP_
