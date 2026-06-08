// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/assertions/fault_event_assertion.hpp"

namespace ros2_fault_injection
{
FaultEventAssertion::FaultEventAssertion(const AssertionConfig & config)
: config_(config)
{
  result_.id = config.id;
  result_.type = config.type;
  result_.state = AssertionState::Pending;
  result_.message = "Waiting for fault event " + config.fault_id + " to reach state " +
    config.state;
}

void FaultEventAssertion::observe(const msg::FaultEvent & event)
{
  if (result_.state != AssertionState::Pending) {
    return;
  }

  if (event.fault_id != config_.fault_id) {
    return;
  }

  if (event.state != config_.state) {
    return;
  }

  result_.state = AssertionState::Passed;
  result_.message = "Observed fault event " + config_.fault_id + " reach state " + config_.state;
}

void FaultEventAssertion::update(double elapsed_seconds)
{
  if (result_.state != AssertionState::Pending) {
    return;   // Already passed or failed
  }

  if (!config_.within.has_value()) {
    return;   // No deadline configured
  }

  if (elapsed_seconds <= config_.within.value()) {
    return;   // Still within deadline
  }

  result_.state = AssertionState::Failed;
  result_.message = "Timed out waiting for fault event " + config_.fault_id + " to reach state " +
    config_.state;
}

AssertionResult FaultEventAssertion::result() const
{
  return result_;
}
} // namespace ros2_fault_injection
