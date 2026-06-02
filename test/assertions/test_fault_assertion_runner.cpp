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
#include "ros2_fault_injection/assertions/fault_assertion_runner.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;

namespace ros2_fault_injection
{

TEST(FaultAssertionRunner, StartsFaultEventAssertions)
    {
        rclcpp::Node node("test_fault_assertion_runner");
        ros2_fault_injection::FaultAssertionRunner runner(node);

        ros2_fault_injection::AssertionConfig config;
        config.id = "test_assertion";
        config.type = "fault_event";
        config.fault_id = "test_fault";
        config.state = "active";

        runner.start({config});
        auto results = runner.results();

        ASSERT_EQ(results.size(), 1);
        EXPECT_EQ(results[0].id, "test_assertion");
        EXPECT_EQ(results[0].state, ros2_fault_injection::AssertionState::Pending);

        rclcpp::shutdown();
}

TEST(FaultAssertionRunner, PassesAssertionWhenEventPublished)
    {
        auto node = std::make_shared<rclcpp::Node>("test_fault_assertion_runner");

        ros2_fault_injection::AssertionConfig config;
        config.id = "odom_bias_activates";
        config.type = "fault_event";
        config.fault_id = "odom_bias";
        config.state = "active";
        config.within = 2.0;

        FaultAssertionRunner runner(*node);
        runner.start({config});

        auto publisher = node->create_publisher<msg::FaultEvent>("/fault_injection/events", 10);

        ros2_fault_injection::msg::FaultEvent event;
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

    if (results.front().state == ros2_fault_injection::AssertionState::Passed) {
      break;
    }
    std::this_thread::sleep_for(10ms);
        }
        const auto results = runner.results();
        ASSERT_EQ(results.size(), 1u);
        EXPECT_EQ(results.front().state, ros2_fault_injection::AssertionState::Passed);

        rclcpp::shutdown();
}

}
