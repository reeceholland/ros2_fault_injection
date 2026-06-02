// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/fault_event_assertion.hpp"

TEST(FaultEventAssertionTest, StartsPending)
{
  ros2_fault_injection::AssertionConfig config;
  config.id = "odom_bias_activates";
  config.type = "fault_event";
  config.fault_id = "odom_bias";
  config.state = "active";

  ros2_fault_injection::FaultEventAssertion assertion(config);
  auto result = assertion.result();

  EXPECT_EQ(result.id, "odom_bias_activates");
  EXPECT_EQ(result.state, ros2_fault_injection::AssertionState::Pending);
}

TEST(FaultEventAssertionTest, PassesOnMatchingEvent)
{
  ros2_fault_injection::AssertionConfig config;
  config.id = "odom_bias_activates";
  config.type = "fault_event";
  config.fault_id = "odom_bias";
  config.state = "active";

  ros2_fault_injection::FaultEventAssertion assertion(config);

  ros2_fault_injection::msg::FaultEvent event;
  event.fault_id = "odom_bias";
  event.state = "active";

  assertion.observe(event);
  auto result = assertion.result();

  EXPECT_EQ(result.id, "odom_bias_activates");
  EXPECT_EQ(result.state, ros2_fault_injection::AssertionState::Passed);
}

TEST(FaultEventAssertionTest, IgnoresDifferentFaultId)
{
  ros2_fault_injection::AssertionConfig config;
  config.id = "odom_bias_activates";
  config.type = "fault_event";
  config.fault_id = "odom_bias";
  config.state = "active";

  ros2_fault_injection::FaultEventAssertion assertion(config);

  ros2_fault_injection::msg::FaultEvent event;
  event.fault_id = "some_other_fault";
  event.state = "active";

  assertion.observe(event);
  auto result = assertion.result();

  EXPECT_EQ(result.id, "odom_bias_activates");
  EXPECT_EQ(result.state, ros2_fault_injection::AssertionState::Pending);
}

TEST(FaultEventAssertionTest, IgnoresDifferentState)
{
  ros2_fault_injection::AssertionConfig config;
  config.id = "odom_bias_activates";
  config.type = "fault_event";
  config.fault_id = "odom_bias";
  config.state = "active";

  ros2_fault_injection::FaultEventAssertion assertion(config);

  ros2_fault_injection::msg::FaultEvent event;
  event.fault_id = "odom_bias";
  event.state = "inactive";

  assertion.observe(event);
  auto result = assertion.result();

  EXPECT_EQ(result.id, "odom_bias_activates");
  EXPECT_EQ(result.state, ros2_fault_injection::AssertionState::Pending);
}

TEST(FaultEventAssertionTest, FailsWhenWithinDeadlineExpires)
{
  ros2_fault_injection::AssertionConfig config;
  config.id = "odom_bias_activates";
  config.type = "fault_event";
  config.fault_id = "odom_bias";
  config.state = "active";
  config.within = 5.0; // seconds

  ros2_fault_injection::FaultEventAssertion assertion(config);
  assertion.update(6.0); // Simulate 6 seconds elapsed

  auto result = assertion.result();
  EXPECT_EQ(result.id, "odom_bias_activates");
  EXPECT_EQ(result.state, ros2_fault_injection::AssertionState::Failed);
}

TEST(FaultEventAssertionTest, DoesNotFailBeforeDeadline)
{
  ros2_fault_injection::AssertionConfig config;
  config.id = "odom_bias_activates";
  config.type = "fault_event";
  config.fault_id = "odom_bias";
  config.state = "active";
  config.within = 5.0; // seconds

  ros2_fault_injection::FaultEventAssertion assertion(config);
  assertion.update(3.0); // Simulate 3 seconds elapsed

  auto result = assertion.result();
  EXPECT_EQ(result.id, "odom_bias_activates");
  EXPECT_EQ(result.state, ros2_fault_injection::AssertionState::Pending);
}
