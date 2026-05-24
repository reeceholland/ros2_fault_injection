// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/injectors/tf_fault_injector.hpp"

using namespace std::chrono_literals;

namespace ros2_fault_injection
{
namespace
{

InjectorConfig make_injector_config()
{
  InjectorConfig config;
  config.id = "tf";
  config.type = "tf";

  TopicEndpointConfig topic;
  topic.input_topic = "/test/tf_raw";
  topic.output_topic = "/test/tf";
  topic.qos_depth = 10;

  config.topic = topic;
  return config;
}

geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent_frame = "odom",
  const std::string & child_frame = "base_link")
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform.transform.rotation = tf2::toMsg(q);

  return transform;
}

tf2_msgs::msg::TFMessage make_tf_message(
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  tf2_msgs::msg::TFMessage msg;
  msg.transforms = transforms;
  return msg;
}

void spin_for(const std::shared_ptr<rclcpp::Node> & node, std::chrono::milliseconds duration)
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
}

bool wait_for_message(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::shared_ptr<tf2_msgs::msg::TFMessage> & latest_msg,
  std::chrono::milliseconds timeout = 500ms)
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < timeout) {
    rclcpp::spin_some(node);

    if (!latest_msg->transforms.empty()) {
      return true;
    }

    std::this_thread::sleep_for(5ms);
  }

  return false;
}

FaultConfig make_fault(const std::string & id = "tf_fault")
{
  FaultConfig fault;
  fault.id = id;
  fault.injector_id = "tf";
  fault.config["parent_frame"] = "odom";
  fault.config["child_frame"] = "base_link";
  return fault;
}

}  // namespace

TEST(TfFaultInjector, TranslationBiasAppliesToMatchingTransform) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_tf_translation_bias");
  TfFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("tf_translation_bias");
  fault.config["x_bias"] = "0.25";
  fault.config["y_bias"] = "-0.5";
  fault.config["z_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto latest_msg = std::make_shared<tf2_msgs::msg::TFMessage>();

  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/test/tf", 10, [latest_msg](const tf2_msgs::msg::TFMessage & msg) {*latest_msg = msg;});

  auto publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/test/tf_raw", 10);

  spin_for(node, 100ms);

  publisher->publish(make_tf_message({make_transform()}));

  ASSERT_TRUE(wait_for_message(node, latest_msg));
  ASSERT_EQ(latest_msg->transforms.size(), 1u);

  const auto & transform = latest_msg->transforms[0];
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.25);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, 1.5);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 4.0);

  (void)subscription;
  rclcpp::shutdown();
}

TEST(TfFaultInjector, BiasDoesNotApplyToNonMatchingTransform) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_tf_non_matching_transform");
  TfFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("tf_translation_bias");
  fault.config["x_bias"] = "0.25";

  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto latest_msg = std::make_shared<tf2_msgs::msg::TFMessage>();

  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/test/tf", 10, [latest_msg](const tf2_msgs::msg::TFMessage & msg) {*latest_msg = msg;});

  auto publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/test/tf_raw", 10);

  spin_for(node, 100ms);

  publisher->publish(make_tf_message({make_transform("map", "odom")}));

  ASSERT_TRUE(wait_for_message(node, latest_msg));
  ASSERT_EQ(latest_msg->transforms.size(), 1u);

  const auto & transform = latest_msg->transforms[0];
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 3.0);

  (void)subscription;
  rclcpp::shutdown();
}

TEST(TfFaultInjector, YawBiasAppliesToMatchingTransform) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_tf_yaw_bias");
  TfFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("tf_yaw_bias");
  fault.config["yaw_bias_deg"] = "90.0";

  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto latest_msg = std::make_shared<tf2_msgs::msg::TFMessage>();

  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/test/tf", 10, [latest_msg](const tf2_msgs::msg::TFMessage & msg) {*latest_msg = msg;});

  auto publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/test/tf_raw", 10);

  spin_for(node, 100ms);

  publisher->publish(make_tf_message({make_transform()}));

  ASSERT_TRUE(wait_for_message(node, latest_msg));
  ASSERT_EQ(latest_msg->transforms.size(), 1u);

  tf2::Quaternion q;
  tf2::fromMsg(latest_msg->transforms[0].transform.rotation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  EXPECT_NEAR(roll, 0.0, 1e-6);
  EXPECT_NEAR(pitch, 0.0, 1e-6);
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-6);

  (void)subscription;
  rclcpp::shutdown();
}

TEST(TfFaultInjector, DropProbabilityOneDropsMatchingTransform) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_tf_drop");
  TfFaultInjector injector(*node, make_injector_config());

  auto fault = make_fault("tf_dropout");
  fault.config["drop_probability"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault(fault.id);

  auto latest_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
  bool received = false;

  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/test/tf", 10, [latest_msg, &received](const tf2_msgs::msg::TFMessage & msg) {
      *latest_msg = msg;
      received = true;
      });

  auto publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/test/tf_raw", 10);

  spin_for(node, 100ms);

  publisher->publish(make_tf_message({make_transform()}));

  spin_for(node, 200ms);

  ASSERT_TRUE(received);
  EXPECT_TRUE(latest_msg->transforms.empty());

  (void)subscription;
  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection
