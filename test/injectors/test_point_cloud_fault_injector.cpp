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
  EXPECT_FLOAT_EQ(points.front().intensity, 1.4F);

  (void)sub;
  rclcpp::shutdown();
}

TEST(PointCloudFaultInjector, ScalesDustReturnIntensity)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_point_cloud_fault_injector_dust_intensity");
  PointCloudFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("dust_returns");
  fault.config["dust_return_probability"] = "1.0";
  fault.config["dust_min_range"] = "1.0";
  fault.config["dust_max_range"] = "1.0";
  fault.config["dust_intensity_scale"] = "0.25";
  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points_raw", 10);

  auto latest_msg = std::make_shared<std::optional<sensor_msgs::msg::PointCloud2>>();
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/test/points", 10,
    [latest_msg](const sensor_msgs::msg::PointCloud2 & msg) {*latest_msg = msg;});

  spin_for(node, 100ms);

  const auto input = make_cloud({Point{10.0F, 0.0F, 0.0F, 8.0F}});
  const auto output = publish_and_wait_for_cloud(node, raw_pub, latest_msg, input);

  ASSERT_TRUE(output.has_value());
  const auto points = read_points(output.value());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().x, 1.0F);
  EXPECT_FLOAT_EQ(points.front().intensity, 2.0F);

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

class SeededDustTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}

  void compare_cloud_sequences(std::uint32_t second_seed, bool expect_equal)
  {
    auto node = std::make_shared<rclcpp::Node>("seeded_dust_test");
    auto first_config = make_injector_config();
    first_config.seed = 12345u;
    first_config.topic->input_topic = "/seeded_dust/first_raw";
    first_config.topic->output_topic = "/seeded_dust/first";
    auto second_config = first_config;
    second_config.id = "second";
    second_config.seed = second_seed;
    second_config.topic->input_topic = "/seeded_dust/second_raw";
    second_config.topic->output_topic = "/seeded_dust/second";
    PointCloudFaultInjector first(*node, first_config);
    PointCloudFaultInjector second(*node, second_config);
    auto fault = make_fault("dust");
    fault.config["dust_return_probability"] = "1.0";
    fault.config["dust_min_range"] = "0.2";
    fault.config["dust_max_range"] = "1.5";
    first.add_fault(fault);
    fault.injector_id = second_config.id;
    second.add_fault(fault);
    first.activate_fault(fault.id);
    second.activate_fault(fault.id);

    using Cloud = sensor_msgs::msg::PointCloud2;
    auto first_pub = node->create_publisher<Cloud>(first_config.topic->input_topic, 10);
    auto second_pub = node->create_publisher<Cloud>(second_config.topic->input_topic, 10);
    std::vector<Cloud> first_outputs, second_outputs;
    auto first_sub = node->create_subscription<Cloud>(first_config.topic->output_topic, 10,
        [&first_outputs](const Cloud & msg) {first_outputs.push_back(msg);});
    auto second_sub = node->create_subscription<Cloud>(second_config.topic->output_topic, 10,
        [&second_outputs](const Cloud & msg) {second_outputs.push_back(msg);});
    const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
    while (first_pub->get_subscription_count() == 0 || second_pub->get_subscription_count() == 0 ||
      first_sub->get_publisher_count() == 0 || second_sub->get_publisher_count() == 0)
    {
      ASSERT_LT(std::chrono::steady_clock::now(), discovery_deadline);
      spin_for(node, 5ms);
    }

    std::vector<Point> points;
    for (int i = 0; i < 32; ++i) {
      points.push_back(Point{10.0F + i, 2.0F, 1.0F, 8.0F});
    }
    auto input = make_cloud(points);
    bool sequences_differ = false;
    for (std::size_t sequence = 0; sequence < 5; ++sequence) {
      SCOPED_TRACE(sequence);
      input.header.stamp.sec = static_cast<int32_t>(sequence + 1);
      // Exactly one publish per input: retrying publication would consume extra RNG draws.
      first_pub->publish(input);
      second_pub->publish(input);
      const auto deadline = std::chrono::steady_clock::now() + 3s;
      while (first_outputs.size() <= sequence || second_outputs.size() <= sequence) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline);
        spin_for(node, 5ms);
      }
      ASSERT_EQ(first_outputs.size(), sequence + 1);
      ASSERT_EQ(second_outputs.size(), sequence + 1);
      const auto & a = first_outputs.back();
      const auto & b = second_outputs.back();
      EXPECT_EQ(a.header, input.header);
      EXPECT_EQ(b.header, input.header);
      EXPECT_NE(a.data, input.data);
      EXPECT_NE(b.data, input.data);
      if (expect_equal) {
        EXPECT_EQ(a, b);
      }
      sequences_differ |= a.data != b.data;
      if (sequence > 0) {
        EXPECT_NE(a.data, first_outputs[sequence - 1].data);
      }
      for (const auto & cloud : {a, b}) {
        for (const auto & point : read_points(cloud)) {
          const auto range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
          EXPECT_GE(range, 0.2F - 1e-5F);
          EXPECT_LE(range, 1.5F + 1e-5F);
        }
      }
    }
    EXPECT_EQ(sequences_differ, !expect_equal);
  }
};

TEST_F(SeededDustTest, SameSeedProducesIdenticalDustOutput)
{
  compare_cloud_sequences(12345u, true);
}

TEST_F(SeededDustTest, DifferentSeedsProduceDifferentDustOutput)
{
  compare_cloud_sequences(54321u, false);
}

}  // namespace ros2_fault_injection::injectors
