// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstddef>
#include <initializer_list>
#include <limits>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/twist_stopped_assertion.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection::assertions
{
namespace
{
AssertionConfig make_twist_stopped_assertion()
{
  AssertionConfig config;
  config.id = "watchdog_stops_after_cmd_dropout";
  config.type = "twist_stopped";
  config.topic = "/cmd_vel";
  config.fault_id = "drop_cmd_vel";
  config.trigger_within = 2.0;
  config.within = 0.5;
  config.duration = 0.5;
  config.linear_tolerance = 0.01;
  config.angular_tolerance = 0.01;
  config.max_gap = 0.2;
  return config;
}

geometry_msgs::msg::Twist moving_command()
{
  geometry_msgs::msg::Twist message;
  message.linear.x = 0.5;
  return message;
}

geometry_msgs::msg::Twist stopped_command()
{
  return geometry_msgs::msg::Twist();
}

msg::FaultEvent activate_fault()
{
  msg::FaultEvent event;
  event.fault_id = "drop_cmd_vel";
  event.state = "active";
  return event;
}

void observe_hold_before_completion(TwistStoppedAssertion & assertion)
{
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 250000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 400000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 550000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 700000000));
}
}  // namespace

TEST(TwistStoppedAssertion, StartsPending)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));

  EXPECT_EQ(assertion.result().state, AssertionState::Pending);
}

TEST(TwistStoppedAssertion, PassesAfterZeroCommandIsHeldForDuration)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));

  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 400000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 600000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 700000000));
  assertion.update(rclcpp::Time(0, 710000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, AllowsMotionAfterHoldCompletesBeforeTimerUpdate)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  observe_hold_before_completion(assertion);
  ASSERT_EQ(assertion.result().state, AssertionState::Pending);

  assertion.observe_message(moving_command(), rclcpp::Time(0, 760000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, AllowsMotionExactlyWhenHoldCompletes)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  observe_hold_before_completion(assertion);

  assertion.observe_message(moving_command(), rclcpp::Time(0, 750000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, LateTimerIgnoresGapThatOnlyExceedsLimitAfterHold)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  observe_hold_before_completion(assertion);

  assertion.update(rclcpp::Time(1, 200000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, LateZeroCommandIgnoresGapThatOnlyExceedsLimitAfterHold)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  observe_hold_before_completion(assertion);

  assertion.observe_message(stopped_command(), rclcpp::Time(1, 200000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, RejectsMotionJustBeforeHoldCompletes)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  observe_hold_before_completion(assertion);

  assertion.observe_message(moving_command(), rclcpp::Time(0, 740000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, LateTimerStillRejectsGapExceededDuringHold)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 250000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 400000000));

  assertion.update(rclcpp::Time(1, 200000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, LateMotionStillRejectsGapExceededDuringHold)
{
  TwistStoppedAssertion assertion(make_twist_stopped_assertion(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 250000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 400000000));

  assertion.observe_message(moving_command(), rclcpp::Time(0, 760000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, ExactMaxGapRemainsValidDuringLongerHold)
{
  auto config = make_twist_stopped_assertion();
  config.duration = 1.0;
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 0));
  for (const auto stamp : {100000000, 300000000, 500000000, 700000000, 900000000}) {
    assertion.observe_message(stopped_command(), rclcpp::Time(0, stamp));
  }

  EXPECT_EQ(assertion.result().state, AssertionState::Pending);
}

TEST(TwistStoppedAssertion, ExactMaxGapRemainsValidAtDecimalHoldDeadline)
{
  auto config = make_twist_stopped_assertion();
  config.duration = 0.8;
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 0));
  for (const auto stamp : {100000000, 300000000, 500000000, 700000000}) {
    assertion.observe_message(stopped_command(), rclcpp::Time(0, stamp));
  }

  assertion.update(rclcpp::Time(0, 900000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, AppliesSeparateLinearAndAngularTolerancesToEveryAxis)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));

  geometry_msgs::msg::Twist within_tolerance;
  within_tolerance.linear.x = 0.005;
  within_tolerance.linear.y = 0.005;
  within_tolerance.linear.z = 0.005;
  within_tolerance.angular.x = 0.005;
  within_tolerance.angular.y = 0.005;
  within_tolerance.angular.z = 0.005;
  assertion.observe_message(within_tolerance, rclcpp::Time(0, 200000000));
  assertion.observe_message(within_tolerance, rclcpp::Time(0, 350000000));
  assertion.observe_message(within_tolerance, rclcpp::Time(0, 500000000));
  assertion.observe_message(within_tolerance, rclcpp::Time(0, 650000000));
  assertion.observe_message(within_tolerance, rclcpp::Time(0, 700000000));
  assertion.update(rclcpp::Time(0, 710000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, PassesWhenTheCommandStreamIsAlreadyZeroAtFaultActivation)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 350000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 500000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 650000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 700000000));
  assertion.update(rclcpp::Time(0, 710000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, DoesNotCountZeroCommandsObservedBeforeFaultActivation)
{
  auto config = make_twist_stopped_assertion();
  config.trigger_within = 0.5;
  config.within = 0.5;
  config.duration = 0.6;
  config.max_gap = 0.5;
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));

  assertion.observe_message(stopped_command(), rclcpp::Time(0, 0));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 350000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 500000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 650000000));
  assertion.update(rclcpp::Time(0, 700000000));

  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.update(rclcpp::Time(0, 700000000));
  EXPECT_EQ(assertion.result().state, AssertionState::Pending);

  assertion.update(rclcpp::Time(0, 810000000));
  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, FailsWhenNoZeroCommandArrivesBeforeDeadline)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.update(rclcpp::Time(0, 610000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsIfMotionResumesDuringHold)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 300000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsWhenZeroCommandsStopArrivingDuringHold)
{
  auto config = make_twist_stopped_assertion();
  config.duration = 1.0;
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000));
  assertion.update(rclcpp::Time(0, 410000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsOnNonFiniteVelocityAfterFaultActivation)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  auto invalid = stopped_command();
  invalid.linear.x = std::numeric_limits<double>::quiet_NaN();
  assertion.observe_message(invalid, rclcpp::Time(0, 200000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsWhenTheFaultDoesNotActivateBeforeDeadline)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.update(rclcpp::Time(2, 100000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Pending);

  assertion.update(rclcpp::Time(2, 600000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsWhenCombinedDeadlineIsNotFinite)
{
  auto config = make_twist_stopped_assertion();
  config.trigger_within = std::numeric_limits<double>::max();
  config.within = std::numeric_limits<double>::max();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));

  assertion.update(rclcpp::Time(1, 0));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
  EXPECT_NE(assertion.result().message.find("finite sum"), std::string::npos);
}

TEST(TwistStoppedAssertion, AcceptsTimelyActivationEventDeliveredAfterTriggerDeadline)
{
  auto config = make_twist_stopped_assertion();
  config.trigger_within = 0.5;
  config.within = 1.0;
  config.duration = 0.3;
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));

  // The event callback can be processed after trigger_within even though the
  // event's source timestamp proves the fault activated on time. Command
  // callbacks after activation can also be processed before the event callback.
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 300000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 450000000));
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 600000000));
  assertion.update(rclcpp::Time(0, 700000000));
  EXPECT_EQ(assertion.result().state, AssertionState::Pending);

  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 100000000));
  assertion.update(rclcpp::Time(0, 710000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, FailsClosedWhenPendingMessageLimitIsExceeded)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));

  for (std::size_t index = 0; index < 20000 &&
    assertion.result().state == AssertionState::Pending; ++index)
  {
    assertion.observe_message(moving_command(), rclcpp::Time(0, 0));
  }

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
  EXPECT_NE(assertion.result().message.find("cannot safely verify"), std::string::npos);
}

TEST(TwistStoppedAssertion, FailsWhenTheFaultEventStampIsAfterTriggerDeadline)
{
  auto config = make_twist_stopped_assertion();
  config.trigger_within = 0.5;
  TwistStoppedAssertion assertion(config, rclcpp::Time(0, 0));
  assertion.observe_fault_event(activate_fault(), rclcpp::Time(0, 600000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}
}  // namespace ros2_fault_injection::assertions
