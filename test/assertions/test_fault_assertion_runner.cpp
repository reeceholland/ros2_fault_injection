// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <thread>

#include "ros2_fault_injection/assertions/fault_assertion_runner.hpp"
#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/fault_event_assertion.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;

namespace ros2_fault_injection::assertions
{

class FaultAssertionRunnerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(FaultAssertionRunnerTest, StartsFaultEventAssertions)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_assertion_runner");
    FaultAssertionRunner runner(*node);

    AssertionConfig config;
    config.id = "test_assertion";
    config.type = "fault_event";
    config.fault_id = "test_fault";
    config.state = "active";

    runner.start({config});
    auto results = runner.results();

    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].id, "test_assertion");
    EXPECT_EQ(results[0].state, AssertionState::Pending);
}

TEST_F(FaultAssertionRunnerTest, PassesAssertionWhenEventPublished)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_assertion_runner");

    AssertionConfig config;
    config.id = "odom_bias_activates";
    config.type = "fault_event";
    config.fault_id = "odom_bias";
    config.state = "active";
    config.within = 2.0;

    FaultAssertionRunner runner(*node);
    runner.start({config});

    auto publisher = node->create_publisher<msg::FaultEvent>("/fault_injection/events", 10);

    msg::FaultEvent event;
    event.fault_id = "odom_bias";
    event.state = "active";
    event.source = "test";

    const auto start = std::chrono::steady_clock::now();
    while (publisher->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() - start < 500ms)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    }

    publisher->publish(event);

    const auto spin_start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - spin_start < 2s) {
    rclcpp::spin_some(node);
    const auto results = runner.results();
    ASSERT_EQ(results.size(), 1);

    if (results.front().state == AssertionState::Passed) {
      break;
    }
    std::this_thread::sleep_for(10ms);
    }
    const auto results = runner.results();
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results.front().state, AssertionState::Passed);
}

TEST_F(FaultAssertionRunnerTest, TopicHzAssertionPassesWhenOdomPublishesFastEnough)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_assertion_runner");

    AssertionConfig config;
    config.id = "odom_stays_above_10hz";
    config.type = "topic_hz";
    config.topic = "/odom";
    config.message_type = "nav_msgs/msg/Odometry";
    config.min_hz = 5.0;
    config.window = 0.5;
    config.within = 2.0;

    FaultAssertionRunner runner(*node);
    runner.start({config});

    auto publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    const auto start = std::chrono::steady_clock::now();
    while (publisher->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() - start < 500ms)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    }

    ASSERT_GT(publisher->get_subscription_count(), 0u);

    nav_msgs::msg::Odometry msg;

    const auto publish_start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - publish_start < 1s) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
    }

    const auto results = runner.results();
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results.front().id, "odom_stays_above_10hz");
    EXPECT_EQ(results.front().type, "topic_hz");
    EXPECT_EQ(results.front().state, AssertionState::Passed);
}

TEST_F(FaultAssertionRunnerTest, TopicHzAssertionFailsWhenOdomPublishesTooSlowly)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_assertion_runner");

    AssertionConfig config;
    config.id = "odom_stays_above_10hz";
    config.type = "topic_hz";
    config.topic = "/odom";
    config.message_type = "nav_msgs/msg/Odometry";
    config.min_hz = 5.0;
    config.window = 0.5;
    config.within = 2.0;

    FaultAssertionRunner runner(*node);
    runner.start({config});

    auto publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    const auto start = std::chrono::steady_clock::now();
    while (publisher->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() - start < 500ms)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    }

    ASSERT_GT(publisher->get_subscription_count(), 0u);

    nav_msgs::msg::Odometry msg;

    const auto publish_start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - publish_start < 3s) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(300ms);
    }

    const auto results = runner.results();
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results.front().id, "odom_stays_above_10hz");
    EXPECT_EQ(results.front().type, "topic_hz");
    EXPECT_EQ(results.front().state, AssertionState::Failed);
}

TEST_F(FaultAssertionRunnerTest, TwistStoppedAssertionObservesCommandsAndFaultEvents)
{
  auto node = std::make_shared<rclcpp::Node>("test_twist_stopped_assertion_runner");

  AssertionConfig config;
  config.id = "watchdog_stops_after_cmd_dropout";
  config.type = "twist_stopped";
  config.topic = "/cmd_vel_test";
  config.fault_id = "drop_cmd_vel";
  config.trigger_within = 2.0;
  config.within = 0.5;
  config.duration = 0.4;
  config.linear_tolerance = 0.01;
  config.angular_tolerance = 0.01;
  config.max_gap = 0.15;

  FaultAssertionRunner runner(*node);
  runner.start({config});

  auto command_publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_test", rclcpp::QoS(10));
  auto event_publisher = node->create_publisher<msg::FaultEvent>(
    "/fault_injection/events", rclcpp::QoS(10));

  const auto discovery_start = std::chrono::steady_clock::now();
  while ((command_publisher->get_subscription_count() == 0 ||
    event_publisher->get_subscription_count() == 0) &&
    std::chrono::steady_clock::now() - discovery_start < 1s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_GT(command_publisher->get_subscription_count(), 0u);
  ASSERT_GT(event_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.5;
  command_publisher->publish(moving);
  rclcpp::spin_some(node);

  msg::FaultEvent event;
  event.stamp = node->now();
  event.fault_id = "drop_cmd_vel";
  event.state = "active";
  event_publisher->publish(event);
  rclcpp::spin_some(node);

  geometry_msgs::msg::Twist stopped;
  const auto stop_start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - stop_start < 700ms) {
    command_publisher->publish(stopped);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  const auto results = runner.results();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.front().type, "twist_stopped");
  EXPECT_EQ(results.front().state, AssertionState::Passed);
}

TEST_F(FaultAssertionRunnerTest, UsesFaultEventStampForStopResponseDeadline)
{
  auto node = std::make_shared<rclcpp::Node>("test_twist_stopped_delayed_event");

  AssertionConfig config;
  config.id = "watchdog_stops_after_cmd_dropout";
  config.type = "twist_stopped";
  config.topic = "/cmd_vel_delayed_test";
  config.fault_id = "drop_cmd_vel";
  config.trigger_within = 2.0;
  config.within = 0.5;
  config.duration = 0.4;
  config.linear_tolerance = 0.01;
  config.angular_tolerance = 0.01;
  config.max_gap = 0.15;

  FaultAssertionRunner runner(*node);
  runner.start({config});

  auto event_publisher = node->create_publisher<msg::FaultEvent>(
    "/fault_injection/events", rclcpp::QoS(10));
  const auto discovery_start = std::chrono::steady_clock::now();
  while (event_publisher->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() - discovery_start < 1s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(event_publisher->get_subscription_count(), 0u);

  const auto assertion_start = node->now();
  std::this_thread::sleep_for(700ms);

  msg::FaultEvent event;
  event.stamp = assertion_start + rclcpp::Duration::from_seconds(0.1);
  event.fault_id = "drop_cmd_vel";
  event.state = "active";
  event_publisher->publish(event);

  const auto update_start = std::chrono::steady_clock::now();
  while (runner.results().front().state == AssertionState::Pending &&
    std::chrono::steady_clock::now() - update_start < 400ms)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  const auto results = runner.results();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.front().state, AssertionState::Failed);
}
}
