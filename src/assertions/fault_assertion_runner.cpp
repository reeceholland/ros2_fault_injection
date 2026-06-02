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

namespace ros2_fault_injection
{
FaultAssertionRunner::FaultAssertionRunner(rclcpp::Node & node)
: node_(node) {}

void FaultAssertionRunner::start(const std::vector<AssertionConfig> & assertions)
{
  fault_event_assertions_.clear();

  for (const auto & config : assertions) {
    if (config.type == "fault_event") {
      fault_event_assertions_.emplace_back(config);
    } else {
      RCLCPP_WARN(node_.get_logger(), "Unsupported assertion type: %s", config.type.c_str());
    }
  }

  start_time_ = node_.now();

  fault_event_subscription_ = node_.create_subscription<msg::FaultEvent>(
        "/fault_injection/events", 10, [this](const msg::FaultEvent::SharedPtr event)
    {on_fault_event(*event);});

  timer_ = node_.create_wall_timer(
        std::chrono::milliseconds(100), [this]()
    {update();});

  RCLCPP_INFO(node_.get_logger(), "Started %zu assertions", fault_event_assertions_.size());
}

std::vector<AssertionResult> FaultAssertionRunner::results() const
{
  std::vector<AssertionResult> results;
  results.reserve(fault_event_assertions_.size());

  for (const auto & assertion : fault_event_assertions_) {
    results.push_back(assertion.result());
  }

  return results;
}

void FaultAssertionRunner::on_fault_event(const msg::FaultEvent & event)
{
  for (auto & assertion : fault_event_assertions_) {
    assertion.observe(event);
  }
}

void FaultAssertionRunner::update()
{
  double elapsed_seconds = (node_.now() - start_time_).seconds();

  for (auto & assertion : fault_event_assertions_) {
    assertion.update(elapsed_seconds);
  }
}

}
