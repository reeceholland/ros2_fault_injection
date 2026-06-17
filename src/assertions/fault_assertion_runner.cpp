// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/assertions/fault_assertion_runner.hpp"

#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/fault_event_assertion.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"
#include "ros2_fault_injection/msg/assertion_event.hpp"

namespace ros2_fault_injection::assertions
{
namespace
{
std::string assertion_state_to_string(AssertionState state)
{
  switch (state) {
    case AssertionState::Pending:
      return "pending";
    case AssertionState::Passed:
      return "passed";
    case AssertionState::Failed:
      return "failed";
  }

  return "unknown";
}
}   // namespace
FaultAssertionRunner::FaultAssertionRunner(rclcpp::Node & node)
: node_(node) {}

void FaultAssertionRunner::start(const std::vector<AssertionConfig> & assertions)
{
  fault_event_assertions_.clear();
  topic_hz_assertions_.clear();
  topic_hz_subscriptions_.clear();
  last_published_states_.clear();

  for (const auto & config : assertions) {
    if (config.type == "fault_event") {
      fault_event_assertions_.emplace_back(config);
    } else if (config.type == "topic_hz") {
      const auto assertion_index = topic_hz_assertions_.size();
      topic_hz_assertions_.emplace_back(config);

      topic_hz_subscriptions_.push_back(node_.create_generic_subscription(
            config.topic, config.message_type, rclcpp::QoS(10),
          [this, assertion_index](std::shared_ptr<rclcpp::SerializedMessage>)
          {
            topic_hz_assertions_.at(assertion_index).observe_message(node_.now());
            }));
    } else {
      RCLCPP_ERROR(node_.get_logger(), "Unknown assertion type '%s' for assertion '%s'",
                     config.type.c_str(), config.id.c_str());
    }
  }

  start_time_ = node_.now();

  fault_event_subscription_ = node_.create_subscription<msg::FaultEvent>(
        "/fault_injection/events", 10, [this](const msg::FaultEvent::SharedPtr event)
    {on_fault_event(*event);});

  assertion_event_publisher_ = node_.create_publisher<msg::AssertionEvent>(
        "/fault_injection/assertion_events", 10);

  timer_ = node_.create_wall_timer(
        std::chrono::milliseconds(100), [this]()
    {update();});

  RCLCPP_INFO(node_.get_logger(), "Started %zu assertions",
                fault_event_assertions_.size() + topic_hz_assertions_.size());
}

std::vector<AssertionResult> FaultAssertionRunner::results() const
{
  std::vector<AssertionResult> results;
  results.reserve(fault_event_assertions_.size() + topic_hz_assertions_.size());

  for (const auto & assertion : fault_event_assertions_) {
    results.push_back(assertion.result());
  }

  for (const auto & assertion : topic_hz_assertions_) {
    results.push_back(assertion.result());
  }
  return results;
}

void FaultAssertionRunner::on_fault_event(const msg::FaultEvent & event)
{
  for (auto & assertion : fault_event_assertions_) {
    assertion.observe(event);
  }

  publish_assertion_event();
}

void FaultAssertionRunner::update()
{
  const auto now = node_.now();
  const double elapsed_seconds = (now - start_time_).seconds();

  for (auto & assertion : fault_event_assertions_) {
    assertion.update(elapsed_seconds);
  }

  for (auto & assertion : topic_hz_assertions_) {
    assertion.update(elapsed_seconds, now);
  }

  publish_assertion_event();
}

void FaultAssertionRunner::publish_assertion_event()
{
  for (const auto & result : results()) {
    if (result.state == AssertionState::Pending) {
      continue;   // Don't publish pending assertions
    }

    const auto it = last_published_states_.find(result.id);
    if (it != last_published_states_.end() && it->second == result.state) {
      continue;   // State hasn't changed since last publish
    }

    msg::AssertionEvent event;
    event.stamp = node_.now();
    event.assertion_id = result.id;
    event.assertion_type = result.type;
    event.state = assertion_state_to_string(result.state);
    event.message = result.message;

    assertion_event_publisher_->publish(event);
    last_published_states_[result.id] = result.state;
  }
}

}
