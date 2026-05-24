// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/injectors/odom_fault_injector.hpp"

namespace ros2_fault_injection
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

std::optional<nav_msgs::msg::Odometry> publish_and_wait_for_odom(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
  const std::shared_ptr<std::optional<nav_msgs::msg::Odometry>> & latest_msg,
  const nav_msgs::msg::Odometry & msg)
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
  config.id = "odom";
  config.type = "odom";

  TopicEndpointConfig topic;
  topic.input_topic = "/test/odom_raw";
  topic.output_topic = "/test/odom";
  topic.qos_depth = 10;

  config.topic = topic;
  return config;
}

FaultConfig make_covariance_fault()
{
  FaultConfig fault;
  fault.id = "odom_covariance";
  fault.injector_id = "odom";
  fault.config["pose_covariance_scale"] = "10.0";
  fault.config["pose_covariance_floor"] = "0.05";
  fault.config["twist_covariance_scale"] = "20.0";
  fault.config["twist_covariance_floor"] = "0.10";
  return fault;
}

nav_msgs::msg::Odometry make_odom_with_covariance(double pose_value, double twist_value)
{
  nav_msgs::msg::Odometry msg;

  for (auto & value : msg.pose.covariance) {
    value = pose_value;
  }

  for (auto & value : msg.twist.covariance) {
    value = twist_value;
  }

  return msg;
}

}  // namespace

TEST(OdomFaultInjector, CovarianceFloorAppliesWhenRawCovarianceIsZero) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_odom_fault_injector_covariance_floor");

  OdomFaultInjector injector(*node, make_injector_config());

  auto fault = make_covariance_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<nav_msgs::msg::Odometry>("/test/odom_raw", 10);

  auto latest_msg = std::make_shared<std::optional<nav_msgs::msg::Odometry>>();
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/test/odom", 10, [latest_msg](const nav_msgs::msg::Odometry & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_odom_with_covariance(0.0, 0.0);
  const auto output = publish_and_wait_for_odom(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());

  for (const auto value : output->pose.covariance) {
    EXPECT_DOUBLE_EQ(value, 0.05);
  }

  for (const auto value : output->twist.covariance) {
    EXPECT_DOUBLE_EQ(value, 0.10);
  }

  (void)sub;
  rclcpp::shutdown();
}

TEST(OdomFaultInjector, CovarianceScaleAppliesWhenRawCovarianceIsNonZero) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_odom_fault_injector_covariance_scale");

  OdomFaultInjector injector(*node, make_injector_config());

  auto fault = make_covariance_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<nav_msgs::msg::Odometry>("/test/odom_raw", 10);

  auto latest_msg = std::make_shared<std::optional<nav_msgs::msg::Odometry>>();
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/test/odom", 10, [latest_msg](const nav_msgs::msg::Odometry & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_odom_with_covariance(0.02, 0.02);
  const auto output = publish_and_wait_for_odom(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());

  for (const auto value : output->pose.covariance) {
    EXPECT_DOUBLE_EQ(value, 0.20);
  }

  for (const auto value : output->twist.covariance) {
    EXPECT_DOUBLE_EQ(value, 0.40);
  }

  (void)sub;
  rclcpp::shutdown();
}

TEST(OdomFaultInjector, CovarianceUnchangedWithoutActiveFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_odom_fault_injector_covariance_unchanged");

  OdomFaultInjector injector(*node, make_injector_config());

  auto fault = make_covariance_fault();
  injector.add_fault(fault);

  auto raw_pub = node->create_publisher<nav_msgs::msg::Odometry>("/test/odom_raw", 10);

  auto latest_msg = std::make_shared<std::optional<nav_msgs::msg::Odometry>>();
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/test/odom", 10, [latest_msg](const nav_msgs::msg::Odometry & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_odom_with_covariance(0.02, 0.03);
  const auto output = publish_and_wait_for_odom(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());

  for (const auto value : output->pose.covariance) {
    EXPECT_DOUBLE_EQ(value, 0.02);
  }

  for (const auto value : output->twist.covariance) {
    EXPECT_DOUBLE_EQ(value, 0.03);
  }

  (void)sub;
  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection
