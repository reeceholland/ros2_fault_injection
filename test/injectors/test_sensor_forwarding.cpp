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
#include "ros2_fault_injection/injectors/scan_fault_injector.hpp"
#include "ros2_fault_injection/injectors/imu_fault_injector.hpp"
#include "ros2_fault_injection/injectors/joint_state_fault_injector.hpp"

using namespace std::chrono_literals;
using namespace ros2_fault_injection;

class SensorForwarding : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

template<class Message, class Injector>
class Stream
{
public:
  Stream()
  {
    node = std::make_shared<rclcpp::Node>("sensor_forwarding_test");
    InjectorConfig config;
    config.id = "sensor";
    TopicEndpointConfig topic;
    topic.input_topic = "/sensor_forwarding_test/raw";
    topic.output_topic = "/sensor_forwarding_test/output";
    topic.qos_depth = 10;
    config.topic = topic;
    injector = std::make_unique<Injector>(*node, config);
    pub = node->create_publisher<Message>(topic.input_topic, 10);
    sub = node->create_subscription<Message>(topic.output_topic, 10,
        [this](typename Message::SharedPtr msg) {received = *msg; history.push_back(*msg);});
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while ((pub->get_subscription_count() == 0 || sub->get_publisher_count() == 0) &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
  }

  std::optional<Message> send(const Message & message, std::chrono::milliseconds wait = 1s)
  {
    received.reset();
    pub->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + wait;
    while (!received && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
    return received;
  }

  void fault(const std::string & id, const std::string & key, const std::string & value)
  {
    FaultConfig config;
    config.id = id;
    config.injector_id = "sensor";
    config.config[key] = value;
    injector->add_fault(config);
  }

  rclcpp::Node::SharedPtr node;
  std::unique_ptr<Injector> injector;
  typename rclcpp::Publisher<Message>::SharedPtr pub;
  typename rclcpp::Subscription<Message>::SharedPtr sub;
  std::optional<Message> received;
  std::vector<Message> history;
};

sensor_msgs::msg::LaserScan scan()
{
  sensor_msgs::msg::LaserScan m;
  m.header.frame_id = "laser";
  m.header.stamp.sec = 17;
  m.header.stamp.nanosec = 123;
  m.angle_min = -1.0f;
  m.angle_max = 1.0f;
  m.angle_increment = 0.5f;
  m.time_increment = 0.01f;
  m.scan_time = 0.1f;
  m.range_min = 0.1f;
  m.range_max = 10.0f;
  m.ranges = {0.2f, 1.0f, 9.9f};
  m.intensities = {3.0f, 5.0f, 7.0f};
  return m;
}

sensor_msgs::msg::Imu imu()
{
  sensor_msgs::msg::Imu m;
  m.header.frame_id = "imu";
  m.header.stamp.sec = 42;
  m.orientation.w = 1.0;
  m.angular_velocity.x = 0.2;
  m.angular_velocity.y = -0.4;
  m.angular_velocity.z = 0.8;
  m.linear_acceleration.x = 1.0;
  m.linear_acceleration.y = -2.0;
  m.linear_acceleration.z = 9.81;
  m.orientation_covariance.fill(0.1);
  m.angular_velocity_covariance.fill(0.2);
  m.linear_acceleration_covariance.fill(0.3);
  return m;
}

sensor_msgs::msg::JointState joints()
{
  sensor_msgs::msg::JointState m;
  m.header.frame_id = "base_link";
  m.header.stamp.sec = 73;
  m.name = {"right", "left"};
  m.position = {2.0, -1.0};
  m.velocity = {0.5, -0.5};
  m.effort = {4.0, 5.0};
  return m;
}

TEST_F(SensorForwarding, ScanInactiveFaultPreservesEveryField)
{
  Stream<sensor_msgs::msg::LaserScan, ScanFaultInjector> stream;
  stream.fault("bias", "range_bias", "5.0");
  const auto input = scan();
  const auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, ScanDropoutStopsAndDeactivationRestoresForwarding)
{
  Stream<sensor_msgs::msg::LaserScan, ScanFaultInjector> stream;
  const auto input = scan();
  ASSERT_TRUE(stream.send(input).has_value());
  stream.fault("drop", "drop_probability", "1.0");
  stream.injector->activate_fault("drop");
  EXPECT_FALSE(stream.send(input, 150ms).has_value());
  stream.injector->deactivate_fault("drop");
  const auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, ImuInactiveFaultPreservesEveryField)
{
  Stream<sensor_msgs::msg::Imu, ImuFaultInjector> stream;
  stream.fault("bias", "angular_velocity_z_bias", "5.0");
  const auto input = imu();
  const auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, ImuDropoutStopsAndDeactivationRestoresForwarding)
{
  Stream<sensor_msgs::msg::Imu, ImuFaultInjector> stream;
  const auto input = imu();
  ASSERT_TRUE(stream.send(input).has_value());
  stream.fault("drop", "drop_probability", "1.0");
  stream.injector->activate_fault("drop");
  EXPECT_FALSE(stream.send(input, 150ms).has_value());
  stream.injector->deactivate_fault("drop");
  const auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, JointStateInactiveFaultPreservesEveryField)
{
  Stream<sensor_msgs::msg::JointState, JointStateFaultInjector> stream;
  stream.fault("bias", "velocity_bias", "5.0");
  const auto input = joints();
  const auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, JointStateDropoutStopsAndDeactivationRestoresForwarding)
{
  Stream<sensor_msgs::msg::JointState, JointStateFaultInjector> stream;
  const auto input = joints();
  ASSERT_TRUE(stream.send(input).has_value());
  stream.fault("drop", "drop_probability", "1.0");
  stream.injector->activate_fault("drop");
  EXPECT_FALSE(stream.send(input, 150ms).has_value());
  stream.injector->deactivate_fault("drop");
  const auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, ScanBiasClampsFiniteRangesAndPreservesInvalidReturns)
{
  Stream<sensor_msgs::msg::LaserScan, ScanFaultInjector> stream;
  stream.fault("bias", "range_bias", "1.0");
  stream.injector->activate_fault("bias");
  auto input = scan();
  input.ranges = {1.0f, 9.9f, std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN()};
  auto output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  ASSERT_EQ(output->ranges.size(), 4u);
  EXPECT_FLOAT_EQ(output->ranges[0], 2.0f);
  EXPECT_FLOAT_EQ(output->ranges[1], 10.0f);
  EXPECT_TRUE(std::isinf(output->ranges[2]));
  EXPECT_TRUE(std::isnan(output->ranges[3]));
  output->ranges = input.ranges = {};
  EXPECT_EQ(*output, input);
}

TEST_F(SensorForwarding, ImuBiasesComposeAndDeactivateIndependently)
{
  Stream<sensor_msgs::msg::Imu, ImuFaultInjector> stream;
  stream.fault("angular", "angular_velocity_z_bias", "0.5");
  stream.fault("linear", "linear_acceleration_x_bias", "2.0");
  stream.injector->activate_fault("angular");
  stream.injector->activate_fault("linear");
  auto expected = imu();
  expected.angular_velocity.z += 0.5;
  expected.linear_acceleration.x += 2.0;
  auto output = stream.send(imu());
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, expected);
  stream.injector->deactivate_fault("angular");
  expected.angular_velocity.z = imu().angular_velocity.z;
  output = stream.send(imu());
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, expected);
}

TEST_F(SensorForwarding, JointBiasPreservesNamesPositionsEffortsAndHandlesEmptyVelocity)
{
  Stream<sensor_msgs::msg::JointState, JointStateFaultInjector> stream;
  stream.fault("bias", "velocity_bias", "0.25");
  stream.injector->activate_fault("bias");
  auto expected = joints();
  expected.velocity = {0.75, -0.25};
  auto output = stream.send(joints());
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, expected);
  auto input = joints();
  input.velocity.clear();
  output = stream.send(input);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(*output, input);
}

// Distinct stamps distinguish recovered messages from stale queued messages.
// Keep spinning after recovery to catch delayed replay as well as the first output.
template<class Message, class Injector>
void check_repeated_dropout_recovery(Message input)
{
  Stream<Message, Injector> stream;
  ASSERT_GT(stream.pub->get_subscription_count(), 0u);
  ASSERT_GT(stream.sub->get_publisher_count(), 0u);
  stream.fault("drop", "drop_probability", "1.0");

  for (int cycle = 0; cycle < 3; ++cycle) {
    SCOPED_TRACE(cycle);
    input.header.stamp.sec = cycle * 10 + 1;
    const auto baseline = stream.send(input);
    ASSERT_TRUE(baseline.has_value());
    EXPECT_EQ(*baseline, input);

    stream.injector->activate_fault("drop");
    const auto before_drop = stream.history.size();
    for (int message = 0; message < 3; ++message) {
      input.header.stamp.sec = cycle * 10 + 2 + message;
      EXPECT_FALSE(stream.send(input, 100ms).has_value());
    }
    EXPECT_EQ(stream.history.size(), before_drop);

    stream.injector->deactivate_fault("drop");
    const auto quiet_deadline = std::chrono::steady_clock::now() + 150ms;
    while (std::chrono::steady_clock::now() < quiet_deadline) {
      rclcpp::spin_some(stream.node);
      std::this_thread::sleep_for(5ms);
    }
    // Deactivation itself must not flush the dropped messages.
    EXPECT_EQ(stream.history.size(), before_drop);

    input.header.stamp.sec = cycle * 10 + 5;
    const auto recovered = stream.send(input);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(*recovered, input);
    const auto recovery_deadline = std::chrono::steady_clock::now() + 150ms;
    while (std::chrono::steady_clock::now() < recovery_deadline) {
      rclcpp::spin_some(stream.node);
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_EQ(stream.history.size(), before_drop + 1);
    EXPECT_EQ(stream.history.back(), input);
  }
}

TEST_F(SensorForwarding, ScanRepeatedRecoveryDoesNotReplayDroppedMessages)
{
  check_repeated_dropout_recovery<sensor_msgs::msg::LaserScan, ScanFaultInjector>(scan());
}

TEST_F(SensorForwarding, ImuRepeatedRecoveryDoesNotReplayDroppedMessages)
{
  check_repeated_dropout_recovery<sensor_msgs::msg::Imu, ImuFaultInjector>(imu());
}

TEST_F(SensorForwarding, JointStateRepeatedRecoveryDoesNotReplayDroppedMessages)
{
  check_repeated_dropout_recovery<sensor_msgs::msg::JointState, JointStateFaultInjector>(joints());
}
