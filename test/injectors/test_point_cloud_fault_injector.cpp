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

class PlumeDustTest : public ::testing::Test
{
protected:
  using Cloud = sensor_msgs::msg::PointCloud2;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("plume_dust_test");

    auto config = make_injector_config();
    config.seed = 12345u;
    config.topic->input_topic = "/plume_test/points_raw";
    config.topic->output_topic = "/plume_test/points";

    injector_ = std::make_unique<PointCloudFaultInjector>(*node_, config);

    publisher_ = node_->create_publisher<Cloud>(config.topic->input_topic, rclcpp::QoS(10));
    subscription_ = node_->create_subscription<Cloud>(
        config.topic->output_topic,
        rclcpp::QoS(10),
      [this](const Cloud & cloud) {
        received_ = cloud;
        });
  }

  void TearDown() override
  {
    subscription_.reset();
    publisher_.reset();
    injector_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  FaultConfig plume_fault(const std::string & coefficient)
  {
    FaultConfig fault;
    fault.id = "test_plume";
    fault.injector_id = "point_cloud";
    fault.config = {
      {"dust_model", "plume"},
      {"plume_center_x", "2.0"},
      {"plume_center_y", "0.0"},
      {"plume_center_z", "0.0"},
      {"plume_sigma_x", "0.5"},
      {"plume_sigma_y", "0.5"},
      {"plume_sigma_z", "0.3"},
      {"plume_interaction_coefficient", coefficient},
      {"plume_step_size", "0.1"}
    };
    return fault;
  }

  bool wait_for_connections()
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (publisher_->get_subscription_count() > 0 &&
        subscription_->get_publisher_count() > 0)
      {
        return true;
      }
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  bool publish_once_and_wait(const Cloud & input)
  {
    received_.reset();
    publisher_->publish(input);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
        // Match the stamp so an earlier phase cannot satisfy this wait.
      if (received_ && received_->header.stamp == input.header.stamp) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<PointCloudFaultInjector> injector_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr subscription_;
  std::optional<Cloud> received_;

};

TEST_F(PlumeDustTest, PlumeDustMovesPointsAlongOriginalRays)
{
  FaultConfig fault;
  fault.id = "test_plume";
  fault.injector_id = "point_cloud";  // ID from make_injector_config().
  fault.config = {
    {"dust_model", "plume"},
    {"plume_center_x", "2.0"},
    {"plume_center_y", "0.0"},
    {"plume_center_z", "0.0"},
    {"plume_sigma_x", "0.5"},
    {"plume_sigma_y", "0.5"},
    {"plume_sigma_z", "0.3"},
    {"plume_interaction_coefficient", "10.0"},
    {"plume_step_size", "0.1"}
  };

  injector_->add_fault(fault);
  injector_->activate_fault(fault.id);

  // Wait for both sides of the proxy to discover their connections.
  const auto discovery_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(5);

  while ((publisher_->get_subscription_count() == 0 ||
    subscription_->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_GT(publisher_->get_subscription_count(), 0u);
  ASSERT_GT(subscription_->get_publisher_count(), 0u);

  const auto input = make_cloud(
    std::vector<Point>(32, Point{5.0F, 0.0F, 0.0F, 100.0F}));

  received_.reset();
  publisher_->publish(input);  // Publish once to preserve the RNG sequence.

  const auto response_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(3);

  while (!received_.has_value() &&
    std::chrono::steady_clock::now() < response_deadline)
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(received_.has_value()) << "No output cloud received";

  const auto points = read_points(received_.value());
  ASSERT_EQ(points.size(), 32u);
  EXPECT_EQ(received_->header.frame_id, input.header.frame_id);

  std::size_t moved_points = 0;

  for (const auto & point : points) {
    ASSERT_TRUE(std::isfinite(point.x));
    ASSERT_TRUE(std::isfinite(point.y));
    ASSERT_TRUE(std::isfinite(point.z));

    EXPECT_GE(point.x, 0.0F);
    EXPECT_LE(point.x, 5.0F);
    EXPECT_FLOAT_EQ(point.y, 0.0F);
    EXPECT_FLOAT_EQ(point.z, 0.0F);

    if (point.x < 5.0F) {
      ++moved_points;
    }
  }

  EXPECT_GT(moved_points, 0u)
    << "The active plume did not modify any points";
}


TEST_F(PlumeDustTest, ZeroCoefficientForwardsCloudUnchanged)
{
  const auto fault = plume_fault("0.0");
  injector_->add_fault(fault);
  injector_->activate_fault(fault.id);
  ASSERT_TRUE(wait_for_connections()) << "ROS discovery timed out";

  auto input = make_cloud({
      {5.0F, 0.0F, 0.0F, 100.0F},
      {3.0F, 1.0F, -0.5F, 25.0F},
      {-2.0F, 0.5F, 1.0F, 7.0F}
  });
  input.header.stamp.sec = 1;

  ASSERT_TRUE(publish_once_and_wait(input)) << "No matching output cloud";
  EXPECT_EQ(received_.value(), input);
}

TEST_F(PlumeDustTest, DeactivatingPlumeRestoresUnchangedForwarding)
{
  const auto fault = plume_fault("10.0");
  injector_->add_fault(fault);
  injector_->activate_fault(fault.id);
  ASSERT_TRUE(wait_for_connections()) << "ROS discovery timed out";

  auto input = make_cloud(
    std::vector<Point>(32, Point{5.0F, 0.0F, 0.0F, 100.0F}));
  input.header.stamp.sec = 1;
  ASSERT_TRUE(publish_once_and_wait(input)) << "No active-phase output";
  ASSERT_NE(received_->data, input.data) << "Active plume must modify the input";

  injector_->deactivate_fault(fault.id);
  // Check multiple subsequent messages to catch retained mutation/state.
  for (int sequence = 2; sequence <= 4; ++sequence) {
    SCOPED_TRACE(sequence);
    input.header.stamp.sec = sequence;
    ASSERT_TRUE(publish_once_and_wait(input)) << "No recovery output";
    EXPECT_EQ(received_.value(), input);
  }
}


TEST_F(PlumeDustTest, PlumeDustLeavesDistantRaysUnchanged)
{
  const auto fault = plume_fault("10.0");
  injector_->add_fault(fault);
  injector_->activate_fault(fault.id);
  ASSERT_TRUE(wait_for_connections());

  std::vector<Point> inputs(32, Point{5.0F, 0.0F, 0.0F, 100.0F});
  inputs.insert(inputs.end(), 32, Point{0.0F, 5.0F, 0.0F, 25.0F});
  auto input = make_cloud(inputs);
  input.header.stamp.sec = 1;
  ASSERT_TRUE(publish_once_and_wait(input));

  const auto points = read_points(received_.value());
  ASSERT_EQ(points.size(), inputs.size());
  std::size_t moved = 0;
  for (std::size_t i = 0; i < 32; ++i) {
    ASSERT_TRUE(std::isfinite(points[i].x));
    EXPECT_GE(points[i].x, 0.0F);
    EXPECT_LE(points[i].x, 5.0F);
    EXPECT_FLOAT_EQ(points[i].y, 0.0F);
    EXPECT_FLOAT_EQ(points[i].z, 0.0F);
    if (points[i].x < inputs[i].x) {
      ++moved;
    }
  }
  EXPECT_GT(moved, 0u);
  // Gaussian tails are nonzero: unchanged distant rays are a fixed-seed case,
  // not a claim that interactions outside a hard boundary are impossible.
  for (std::size_t i = 32; i < points.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_FLOAT_EQ(points[i].x, inputs[i].x);
    EXPECT_FLOAT_EQ(points[i].y, inputs[i].y);
    EXPECT_FLOAT_EQ(points[i].z, inputs[i].z);
    EXPECT_FLOAT_EQ(points[i].intensity, inputs[i].intensity);
  }
}

TEST_F(PlumeDustTest, SameSeedReproducesPlumeCloudSequence)
{
  auto second_config = make_injector_config();
  second_config.id = "second_plume";
  second_config.seed = 12345u;
  second_config.topic->input_topic = "/plume_test/second_raw";
  second_config.topic->output_topic = "/plume_test/second";

  PointCloudFaultInjector second(*node_, second_config);
  auto fault = plume_fault("10.0");
  injector_->add_fault(fault);
  injector_->activate_fault(fault.id);
  fault.injector_id = second_config.id;
  second.add_fault(fault);
  second.activate_fault(fault.id);

  std::optional<Cloud> second_received;
  auto second_pub = node_->create_publisher<Cloud>(second_config.topic->input_topic, 10);
  auto second_sub = node_->create_subscription<Cloud>(
    second_config.topic->output_topic, 10,
    [&second_received](const Cloud & cloud) {second_received = cloud;});

  ASSERT_TRUE(wait_for_connections());
  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((second_pub->get_subscription_count() == 0 ||
    second_sub->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(second_pub->get_subscription_count(), 0u);
  ASSERT_GT(second_sub->get_publisher_count(), 0u);

  std::optional<Cloud> previous;
  for (int sequence = 1; sequence <= 5; ++sequence) {
    SCOPED_TRACE(sequence);
    auto input = make_cloud(
      std::vector<Point>(32, Point{5.0F, 0.0F, 0.0F, 100.0F}));
    input.header.stamp.sec = sequence;
    received_.reset();
    second_received.reset();
    // Exactly one input per injector per sequence element.
    publisher_->publish(input);
    second_pub->publish(input);

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    const auto complete = [&]() {
        return received_ && second_received &&
               received_->header.stamp == input.header.stamp &&
               second_received->header.stamp == input.header.stamp;
      };
    while (!complete() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_TRUE(complete()) << "Missing matching cloud from one of the injectors";
    EXPECT_EQ(received_.value(), second_received.value());
    EXPECT_NE(received_->data, input.data) << "Must compare actual plume mutations";
    if (previous) {
      EXPECT_NE(received_->data, previous->data) << "RNG must advance across clouds";
    }
    previous = received_;
  }
}

}  // namespace ros2_fault_injection::injectors
