// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "ros2_fault_injection/injectors/point_cloud_fault_injector.hpp"

namespace ros2_fault_injection::injectors
{
namespace
{

using namespace std::chrono_literals;

struct Point
{
  float x;
  float y;
  float z;
  float intensity;
};

void spin_for(const rclcpp::Node::SharedPtr & node, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
}

InjectorConfig make_injector_config()
{
  InjectorConfig config;
  config.id = "point_cloud";
  config.type = "point_cloud";

  TopicEndpointConfig topic;
  topic.input_topic = "/test/points_raw";
  topic.output_topic = "/test/points";
  topic.qos_depth = 10;

  config.topic = topic;
  return config;
}

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<Point> & points)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "lidar";
  msg.height = 1;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(msg, "intensity");

  for (const auto & point : points) {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    *intensity = point.intensity;
    ++x;
    ++y;
    ++z;
    ++intensity;
  }

  return msg;
}

std::vector<Point> read_points(sensor_msgs::msg::PointCloud2 msg)
{
  std::vector<Point> points;

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(msg, "intensity");

  for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
    points.push_back(Point{*x, *y, *z, *intensity});
  }

  return points;
}

std::optional<sensor_msgs::msg::PointCloud2> publish_and_wait_for_cloud(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<std::optional<sensor_msgs::msg::PointCloud2>> & latest_msg,
  const sensor_msgs::msg::PointCloud2 & msg)
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

FaultConfig make_fault(const std::string & id)
{
  FaultConfig fault;
  fault.id = id;
  fault.injector_id = "point_cloud";
  return fault;
}

}  // namespace

TEST(PointCloudFaultInjector, PassesThroughPointCloudWithoutActiveFault)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_point_cloud_fault_injector_passthrough");
  PointCloudFaultInjector injector(*node, make_injector_config());

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points_raw", 10);

  auto latest_msg = std::make_shared<std::optional<sensor_msgs::msg::PointCloud2>>();
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/test/points", 10,
    [latest_msg](const sensor_msgs::msg::PointCloud2 & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_cloud({Point{1.0F, 2.0F, 3.0F, 4.0F}});
  const auto output = publish_and_wait_for_cloud(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  const auto points = read_points(output.value());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().x, 1.0F);
  EXPECT_FLOAT_EQ(points.front().y, 2.0F);
  EXPECT_FLOAT_EQ(points.front().z, 3.0F);
  EXPECT_FLOAT_EQ(points.front().intensity, 4.0F);

  (void)sub;
  rclcpp::shutdown();
}

TEST(PointCloudFaultInjector, InvalidatesPointsWhenPointDropoutProbabilityIsOne)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_point_cloud_fault_injector_point_dropout");
  PointCloudFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("point_dropout");
  fault.config["point_dropout_probability"] = "1.0";
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points_raw", 10);

  auto latest_msg = std::make_shared<std::optional<sensor_msgs::msg::PointCloud2>>();
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/test/points", 10,
    [latest_msg](const sensor_msgs::msg::PointCloud2 & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_cloud({Point{1.0F, 0.0F, 0.0F, 4.0F}});
  const auto output = publish_and_wait_for_cloud(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  const auto points = read_points(output.value());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_TRUE(std::isnan(points.front().x));
  EXPECT_TRUE(std::isnan(points.front().y));
  EXPECT_TRUE(std::isnan(points.front().z));
  EXPECT_FLOAT_EQ(points.front().intensity, 4.0F);

  (void)sub;
  rclcpp::shutdown();
}

TEST(PointCloudFaultInjector, ConvertsPointsToNearDustReturns)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_point_cloud_fault_injector_dust_returns");
  PointCloudFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("dust_returns");
  fault.config["dust_return_probability"] = "1.0";
  fault.config["dust_min_range"] = "1.0";
  fault.config["dust_max_range"] = "1.0";
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points_raw", 10);

  auto latest_msg = std::make_shared<std::optional<sensor_msgs::msg::PointCloud2>>();
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/test/points", 10,
    [latest_msg](const sensor_msgs::msg::PointCloud2 & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_cloud({Point{10.0F, 0.0F, 0.0F, 7.0F}});
  const auto output = publish_and_wait_for_cloud(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  const auto points = read_points(output.value());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().x, 1.0F);
  EXPECT_FLOAT_EQ(points.front().y, 0.0F);
  EXPECT_FLOAT_EQ(points.front().z, 0.0F);
  EXPECT_FLOAT_EQ(points.front().intensity, 7.0F);

  (void)sub;
  rclcpp::shutdown();
}

TEST(PointCloudFaultInjector, ScalesIntensityWhenIntensityFieldExists)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_point_cloud_fault_injector_intensity");
  PointCloudFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("intensity_scale");
  fault.config["intensity_scale"] = "0.5";
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points_raw", 10);

  auto latest_msg = std::make_shared<std::optional<sensor_msgs::msg::PointCloud2>>();
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/test/points", 10,
    [latest_msg](const sensor_msgs::msg::PointCloud2 & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_cloud({Point{1.0F, 0.0F, 0.0F, 8.0F}});
  const auto output = publish_and_wait_for_cloud(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  const auto points = read_points(output.value());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().intensity, 4.0F);

  (void)sub;
  rclcpp::shutdown();
}

TEST(PointCloudFaultInjector, AppliesRangeNoiseAlongPointRay)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_point_cloud_fault_injector_range_noise");
  PointCloudFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("range_noise");
  fault.config["range_noise_stddev"] = "1.0";
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points_raw", 10);

  auto latest_msg = std::make_shared<std::optional<sensor_msgs::msg::PointCloud2>>();
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/test/points", 10,
    [latest_msg](const sensor_msgs::msg::PointCloud2 & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_cloud({Point{3.0F, 0.0F, 0.0F, 1.0F}});
  const auto output = publish_and_wait_for_cloud(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  const auto points = read_points(output.value());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().y, 0.0F);
  EXPECT_FLOAT_EQ(points.front().z, 0.0F);
  EXPECT_GE(points.front().x, 0.0F);

  (void)sub;
  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection::injectors
