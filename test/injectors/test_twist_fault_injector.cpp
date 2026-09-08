// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/injectors/twist_fault_injector.hpp"

namespace ros2_fault_injection::injectors
{
namespace
{

using namespace std::chrono_literals;

void spin_for(const rclcpp::Node::SharedPtr & node, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
}

std::optional<geometry_msgs::msg::Twist> publish_and_wait_for_twist(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr & pub,
  const std::shared_ptr<std::optional<geometry_msgs::msg::Twist>> & latest_msg,
  const geometry_msgs::msg::Twist & msg)
{
  latest_msg->reset();

  const auto deadline = std::chrono::steady_clock::now() + 500ms;

  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    rclcpp::spin_some(node);

    if (latest_msg->has_value()) {
      return latest_msg->value();
    }

    std::this_thread::sleep_for(10ms);
  }

  return std::nullopt;
}

InjectorConfig make_injector_config()
{
  InjectorConfig config;
  config.id = "cmd_vel";
  config.type = "twist";

  TopicEndpointConfig topic;
  topic.input_topic = "/test/cmd_vel_raw";
  topic.output_topic = "/test/cmd_vel";
  topic.qos_depth = 10;

  config.topic = topic;
  return config;
}

geometry_msgs::msg::Twist make_twist(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  return msg;
}

FaultConfig make_dropout_fault()
{
  FaultConfig fault;
  fault.id = "cmd_vel_dropout";
  fault.injector_id = "cmd_vel";
  fault.config["drop_probability"] = "1.0";
  return fault;
}

FaultConfig make_stale_replay_fault()
{
  FaultConfig fault;
  fault.id = "cmd_vel_stale_replay";
  fault.injector_id = "cmd_vel";
  fault.config["stale_replay_enabled"] = "true";
  fault.config["stale_replay_duration_ms"] = "1000";
  return fault;
}

FaultConfig make_scale_fault()
{
  FaultConfig fault;
  fault.id = "cmd_vel_scale";
  fault.injector_id = "cmd_vel";
  fault.config["linear_x_scale"] = "0.5";
  fault.config["angular_z_scale"] = "2.0";
  return fault;
}

FaultConfig make_clamp_fault()
{
  FaultConfig fault;
  fault.id = "cmd_vel_clamp";
  fault.injector_id = "cmd_vel";
  fault.config["max_linear_x"] = "0.3";
  fault.config["max_angular_z"] = "0.4";
  return fault;
}

FaultConfig make_force_stop_fault()
{
  FaultConfig fault;
  fault.id = "cmd_vel_force_stop";
  fault.injector_id = "cmd_vel";
  fault.config["force_stop"] = "true";
  return fault;
}

}  // namespace

TEST(TwistFaultInjector, PassesThroughCommandsWithoutActiveFault)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_twist_fault_injector_passthrough");

  TwistFaultInjector injector(*node, make_injector_config());

  auto raw_pub = node->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel_raw", 10);

  auto latest_msg = std::make_shared<std::optional<geometry_msgs::msg::Twist>>();
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/test/cmd_vel", 10,
    [latest_msg](const geometry_msgs::msg::Twist & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_twist(0.4, 0.2);
  const auto output = publish_and_wait_for_twist(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->linear.x, 0.4);
  EXPECT_DOUBLE_EQ(output->angular.z, 0.2);

  (void)sub;
  rclcpp::shutdown();
}

TEST(TwistFaultInjector, DropsCommandsWhenDropProbabilityIsOne)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_twist_fault_injector_dropout");

  TwistFaultInjector injector(*node, make_injector_config());

  auto fault = make_dropout_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel_raw", 10);

  auto latest_msg = std::make_shared<std::optional<geometry_msgs::msg::Twist>>();
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/test/cmd_vel", 10,
    [latest_msg](const geometry_msgs::msg::Twist & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < deadline) {
    raw_pub->publish(make_twist(0.4, 0.0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_FALSE(latest_msg->has_value());

  (void)sub;
  rclcpp::shutdown();
}

TEST(TwistFaultInjector, ReplaysPreviousCommandWhenStaleReplayIsActive)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_twist_fault_injector_stale_replay");

  TwistFaultInjector injector(*node, make_injector_config());

  auto fault = make_stale_replay_fault();
  injector.add_fault(fault);

  auto raw_pub = node->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel_raw", 10);

  auto latest_msg = std::make_shared<std::optional<geometry_msgs::msg::Twist>>();
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/test/cmd_vel", 10,
    [latest_msg](const geometry_msgs::msg::Twist & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto first_command = make_twist(0.5, 0.0);
  const auto first_output =
    publish_and_wait_for_twist(node, raw_pub, latest_msg, first_command);

  ASSERT_TRUE(first_output.has_value());
  EXPECT_DOUBLE_EQ(first_output->linear.x, 0.5);

  injector.activate_fault(fault.id);

  const auto stopped_command = make_twist(0.0, 0.0);
  const auto stale_output =
    publish_and_wait_for_twist(node, raw_pub, latest_msg, stopped_command);

  ASSERT_TRUE(stale_output.has_value());
  EXPECT_DOUBLE_EQ(stale_output->linear.x, 0.5);
  EXPECT_DOUBLE_EQ(stale_output->angular.z, 0.0);

  (void)sub;
  rclcpp::shutdown();
}

TEST(TwistFaultInjector, ScalesCommandValues)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_twist_fault_injector_scale");

  TwistFaultInjector injector(*node, make_injector_config());

  auto fault = make_scale_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel_raw", 10);

  auto latest_msg = std::make_shared<std::optional<geometry_msgs::msg::Twist>>();
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/test/cmd_vel", 10,
    [latest_msg](const geometry_msgs::msg::Twist & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto output = publish_and_wait_for_twist(node, raw_pub, latest_msg, make_twist(0.8, 0.3));

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->linear.x, 0.4);
  EXPECT_DOUBLE_EQ(output->angular.z, 0.6);

  (void)sub;
  rclcpp::shutdown();
}

TEST(TwistFaultInjector, ClampsCommandValuesSymmetrically)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_twist_fault_injector_clamp");

  TwistFaultInjector injector(*node, make_injector_config());

  auto fault = make_clamp_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel_raw", 10);

  auto latest_msg = std::make_shared<std::optional<geometry_msgs::msg::Twist>>();
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/test/cmd_vel", 10,
    [latest_msg](const geometry_msgs::msg::Twist & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto positive_output =
    publish_and_wait_for_twist(node, raw_pub, latest_msg, make_twist(0.8, 0.9));

  ASSERT_TRUE(positive_output.has_value());
  EXPECT_DOUBLE_EQ(positive_output->linear.x, 0.3);
  EXPECT_DOUBLE_EQ(positive_output->angular.z, 0.4);

  spin_for(node, 100ms);

  const auto negative_output =
    publish_and_wait_for_twist(node, raw_pub, latest_msg, make_twist(-0.8, -0.9));

  ASSERT_TRUE(negative_output.has_value());
  EXPECT_DOUBLE_EQ(negative_output->linear.x, -0.3);
  EXPECT_DOUBLE_EQ(negative_output->angular.z, -0.4);

  (void)sub;
  rclcpp::shutdown();
}

TEST(TwistFaultInjector, ForceStopPublishesZeroCommand)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_twist_fault_injector_force_stop");

  TwistFaultInjector injector(*node, make_injector_config());

  auto fault = make_force_stop_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel_raw", 10);

  auto latest_msg = std::make_shared<std::optional<geometry_msgs::msg::Twist>>();
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/test/cmd_vel", 10,
    [latest_msg](const geometry_msgs::msg::Twist & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto output = publish_and_wait_for_twist(node, raw_pub, latest_msg, make_twist(0.8, 0.9));

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(output->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(output->linear.z, 0.0);
  EXPECT_DOUBLE_EQ(output->angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output->angular.y, 0.0);
  EXPECT_DOUBLE_EQ(output->angular.z, 0.0);

  (void)sub;
  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection::injectors
