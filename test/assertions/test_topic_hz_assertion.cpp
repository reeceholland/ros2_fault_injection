// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ros2_fault_injection/assertions/fault_assertion_runner.hpp"
#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"

namespace ros2_fault_injection::assertions
{
AssertionConfig make_topic_hz_assertion()
{
  AssertionConfig config;
  config.id = "odom_stays_above_10hz";
  config.type = "topic_hz";
  config.topic = "/odom";
  config.min_hz = 10.0;
  config.window = 1.0;
  config.within = 2.0;
  return config;
}

TEST(TopicHzAssertionTest, StartsPending)
  {
    auto const config = make_topic_hz_assertion();

    TopicHzAssertion assertion(config);
    auto result = assertion.result();

    EXPECT_EQ(result.id, "odom_stays_above_10hz");
    EXPECT_EQ(result.type, "topic_hz");
    EXPECT_EQ(result.state, AssertionState::Pending);
}

TEST(TopicHzAssertionTest, PassesWhenObservedRateMeetsMinimum)
  {
    auto const config = make_topic_hz_assertion();

    TopicHzAssertion assertion(config);

    for (int i = 0; i < 12; ++i) {
    assertion.observe_message(rclcpp::Time(0, i * 80000000));   // 80ms apart
    }

    assertion.update(1.0, rclcpp::Time(1, 0));

    const auto result = assertion.result();

    EXPECT_EQ(result.state, AssertionState::Passed);
}

TEST(TopicHzAssertion, StaysPendingBeforeDeadlineWhenRateTooLow)
  {
    auto config = make_topic_hz_assertion();
    config.min_hz = 20.0;
    config.window = 1.0;
    config.within = 5.0;

    TopicHzAssertion assertion(config);

    assertion.observe_message(rclcpp::Time(0, 0));
    assertion.observe_message(rclcpp::Time(0, 500000000));

    assertion.update(2.0, rclcpp::Time(2, 0));

    EXPECT_EQ(assertion.result().state, AssertionState::Pending);
}

TEST(TopicHzAssertion, FailsAfterDeadlineWhenRateTooLow)
  {
    auto config = make_topic_hz_assertion();
    config.min_hz = 10.0;
    config.window = 1.0;
    config.within = 2.0;

    TopicHzAssertion assertion(config);

    assertion.observe_message(rclcpp::Time(0, 0));
    assertion.observe_message(rclcpp::Time(1, 0));

    assertion.update(2.1, rclcpp::Time(2, 100000000));

    const auto result = assertion.result();
    EXPECT_EQ(result.state, AssertionState::Failed);
    EXPECT_NE(result.message.find("Timed out"), std::string::npos);
}

TEST(TopicHzAssertion, IgnoresOldMessagesOutsideWindow)
  {
    auto config = make_topic_hz_assertion();
    config.min_hz = 3.0;
    config.window = 1.0;
    config.within = 5.0;

    TopicHzAssertion assertion(config);

    assertion.observe_message(rclcpp::Time(0, 0));
    assertion.observe_message(rclcpp::Time(0, 100000000));
    assertion.observe_message(rclcpp::Time(3, 0));

    assertion.update(3.0, rclcpp::Time(3, 0));

    EXPECT_EQ(assertion.result().state, AssertionState::Pending);
}
} // namespace ros2_fault_injection::assertions
