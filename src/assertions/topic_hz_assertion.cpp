// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/assertions/topic_hz_assertion.hpp"

#include <algorithm>
#include <string>

namespace ros2_fault_injection::assertions
{
TopicHzAssertion::TopicHzAssertion(const AssertionConfig & config)
: config_(config)
{
  result_.id = config.id;
  result_.type = config.type;
  result_.state = AssertionState::Pending;
  result_.message = "Waiting for topic " + config.topic + " to reach " +
    std::to_string(config.min_hz.value_or(0.0)) + " Hz";
}

const std::string & TopicHzAssertion::topic() const
{
  return config_.topic;
}

void TopicHzAssertion::observe_message(rclcpp::Time stamp)
{
  message_times_.push_back(stamp);
}

void TopicHzAssertion::update(double elapsed_seconds, const rclcpp::Time & now)
{
  if (result_.state != AssertionState::Pending) {
    return;
  }

  const double window = config_.window.value_or(1.0);
  const double min_hz = config_.min_hz.value_or(0.0);

  message_times_.erase(
    std::remove_if(
      message_times_.begin(),
      message_times_.end(),
      [&](const rclcpp::Time & stamp)
      {
        return (now - stamp).seconds() > window;
      }),
    message_times_.end());

  const double observed_hz = static_cast<double>(message_times_.size()) / window;

  if (observed_hz >= min_hz) {
    result_.state = AssertionState::Passed;
    result_.message =
      "Observed topic " + config_.topic + " at " + std::to_string(observed_hz) + " Hz";
    return;
  }

  if (config_.within && elapsed_seconds > config_.within.value()) {
    result_.state = AssertionState::Failed;
    result_.message =
      "Timed out waiting for topic " + config_.topic + " to reach " +
      std::to_string(min_hz) + " Hz";
  }
}

AssertionResult TopicHzAssertion::result() const
{
  return result_;
}
} // namespace ros2_fault_injection::assertions
