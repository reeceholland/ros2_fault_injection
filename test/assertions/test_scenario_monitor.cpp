// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "ros2_fault_injection/assertions/scenario_monitor.hpp"

using namespace std::chrono_literals;

namespace
{

class ScenarioMonitorTest : public ::testing::Test
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
}

namespace ros2_fault_injection::assertions
{
TEST_F(ScenarioMonitorTest, PublishesRunningWhenAssertionsPending)
  {
    auto node = std::make_shared<rclcpp::Node>("test_scenario_monitor_running");

    ScenarioMonitor monitor(*node);

    auto latest_status = std::make_shared<msg::ScenarioStatus>();
    auto received = std::make_shared<bool>(false);

    auto subscription = node->create_subscription<msg::ScenarioStatus>(
        "/fault_injection/scenario_status",
        10,
    [latest_status, received](const msg::ScenarioStatus & status)
    {
      *latest_status = status;
      *received = true;
        });

    AssertionResult result;
    result.id = "odom_stays_above_10hz";
    result.type = "topic_hz";
    result.state = AssertionState::Pending;
    result.message = "waiting";

    monitor.publish({result});

    const auto start = std::chrono::steady_clock::now();
    while (!*received && std::chrono::steady_clock::now() - start < 500ms) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(*received);
    EXPECT_EQ(latest_status->state, "running");
    EXPECT_EQ(latest_status->pending_count, 1u);
    EXPECT_EQ(latest_status->passed_count, 0u);
    EXPECT_EQ(latest_status->failed_count, 0u);

  (void)subscription;
}

TEST_F(ScenarioMonitorTest, PublishesPassedWhenAllAssertionsPassed)
  {
    auto node = std::make_shared<rclcpp::Node>("test_scenario_monitor_passed");

    ScenarioMonitor monitor(*node);

    auto latest_status = std::make_shared<msg::ScenarioStatus>();
    auto received = std::make_shared<bool>(false);

    auto subscription = node->create_subscription<msg::ScenarioStatus>(
        "/fault_injection/scenario_status",
        10,
    [latest_status, received](const msg::ScenarioStatus & status)
    {
      *latest_status = status;
      *received = true;
        });

    AssertionResult result;
    result.id = "odom_stays_above_10hz";
    result.type = "topic_hz";
    result.state = AssertionState::Passed;
    result.message = "passed";

    monitor.publish({result});

    const auto start = std::chrono::steady_clock::now();
    while (!*received && std::chrono::steady_clock::now() - start < 500ms) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(*received);
    EXPECT_EQ(latest_status->state, "passed");
    EXPECT_EQ(latest_status->pending_count, 0u);
    EXPECT_EQ(latest_status->passed_count, 1u);
    EXPECT_EQ(latest_status->failed_count, 0u);

  (void)subscription;
}

TEST_F(ScenarioMonitorTest, PublishesFailedWhenAnyAssertionFails)
  {
    auto node = std::make_shared<rclcpp::Node>("test_scenario_monitor_failed");

    ScenarioMonitor monitor(*node);

    auto latest_status = std::make_shared<msg::ScenarioStatus>();
    auto received = std::make_shared<bool>(false);

    auto subscription = node->create_subscription<msg::ScenarioStatus>(
        "/fault_injection/scenario_status",
        10,
    [latest_status, received](const msg::ScenarioStatus & status)
    {
      *latest_status = status;
      *received = true;
        });

    AssertionResult result;
    result.id = "odom_stays_above_10hz";
    result.type = "topic_hz";
    result.state = AssertionState::Failed;
    result.message = "failed";

    monitor.publish({result});

    const auto start = std::chrono::steady_clock::now();
    while (!*received && std::chrono::steady_clock::now() - start < 500ms) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(*received);
    EXPECT_EQ(latest_status->state, "failed");
    EXPECT_EQ(latest_status->pending_count, 0u);
    EXPECT_EQ(latest_status->passed_count, 0u);
    EXPECT_EQ(latest_status->failed_count, 1u);

  (void)subscription;
}

} // namespace ros2_fault_injection::assertions
