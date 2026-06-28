// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/odom_fault_injector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <random>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ros2_fault_injection::injectors
{

OdomFaultInjector::OdomFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

    // The injector sits in the odom data path: input odom is read, optionally
    // modified or delayed, then republished on the configured output topic.
  pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.topic->output_topic, qos);

  sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.topic->input_topic, qos,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {on_odom(msg);});

    // Delayed odom samples are released by a small periodic flush instead of
    // creating one timer per message.
  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]()
      {flush_delayed();});
}

void OdomFaultInjector::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

    // Copy first so the original incoming message is never mutated in place.
  auto out = *msg;

  apply_bias(out);
  apply_noise(out);
  apply_yaw_bias(out);
  apply_yaw_noise(out);
  apply_covariance_scale(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedOdom{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void OdomFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void OdomFaultInjector::apply_bias(nav_msgs::msg::Odometry & msg)
{
    // Biases stack additively, so separate faults can represent separate error
    // sources while still targeting the same odom stream.
  msg.pose.pose.position.x += active_sum_double("x_bias", 0.0);
  msg.pose.pose.position.y += active_sum_double("y_bias", 0.0);
}

void OdomFaultInjector::apply_noise(nav_msgs::msg::Odometry & msg)
{
  const double x_noise_stddev = active_max_double("x_noise_stddev", 0.0);
  const double y_noise_stddev = active_max_double("y_noise_stddev", 0.0);

  if (x_noise_stddev > 0.0) {
    std::normal_distribution<double> distribution(0.0, x_noise_stddev);
    msg.pose.pose.position.x += distribution(rng_);
  }

  if (y_noise_stddev > 0.0) {
    std::normal_distribution<double> distribution(0.0, y_noise_stddev);
    msg.pose.pose.position.y += distribution(rng_);
  }
}

void OdomFaultInjector::apply_yaw_bias(nav_msgs::msg::Odometry & msg)
{
    // Biases stack additively, so separate faults can represent separate error
    // sources while still targeting the same odom stream.
  const double yaw_bias_deg = active_sum_double("yaw_bias_deg", 0.0);
  if (yaw_bias_deg == 0.0) {
    return;
  }

    // Convert ROS geometry message to tf2 type for easier manipulation.
  tf2::Quaternion q;
  tf2::fromMsg(msg.pose.pose.orientation, q);

    // Extract roll, pitch, and yaw from the quaternion so we can apply bias to just the yaw
    // component.
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Apply bias to just the yaw component. Convert bias from degrees to radians since that's the
    // unit used in the odom message.
  yaw += yaw_bias_deg * M_PI / 180.0;

    // Convert back to quaternion.
  q.setRPY(roll, pitch, yaw);

    // Normalize the quaternion to prevent it from becoming invalid after multiple biased updates.
  q.normalize();

    // Convert back to ROS geometry message and update the odom message.
  msg.pose.pose.orientation = tf2::toMsg(q);
}

void OdomFaultInjector::apply_yaw_noise(nav_msgs::msg::Odometry & msg)
{
    // Get the maximum yaw noise stddev across all active faults since noise from multiple faults
    // won't stack but will compete to inject noise on the same messages.
  const double yaw_noise_stddev_deg = active_max_double("yaw_noise_stddev_deg", 0.0);
  if (yaw_noise_stddev_deg == 0.0) {
    return;
  }

    // Noise is applied independently per message, so multiple faults targeting the same odom stream
    // will compete to inject noise but won't stack.
  std::normal_distribution<double> distribution(0.0, yaw_noise_stddev_deg);

    // Convert noise from degrees to radians since that's the unit used in the odom message.
  const double yaw_noise_rad = distribution(rng_) * M_PI / 180.0;

    // Convert ROS geometry message to tf2 type for easier manipulation.
  tf2::Quaternion q;
  tf2::fromMsg(msg.pose.pose.orientation, q);

    // Extract roll, pitch, and yaw from the quaternion so we can apply noise to just the yaw
    // component.
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Apply noise to just the yaw component.
  yaw += yaw_noise_rad;

    // Convert back to quaternion.
  q.setRPY(roll, pitch, yaw);

    // Normalize the quaternion to prevent it from becoming invalid after multiple noisy updates.
  q.normalize();

    // Convert back to ROS geometry message and update the odom message.
  msg.pose.pose.orientation = tf2::toMsg(q);
}

void OdomFaultInjector::apply_covariance_scale(nav_msgs::msg::Odometry & msg)
{
  const double pose_scale = active_max_double("pose_covariance_scale", 1.0);
  const double twist_scale = active_max_double("twist_covariance_scale", 1.0);
  const double pose_floor = active_max_double("pose_covariance_floor", 0.0);
  const double twist_floor = active_max_double("twist_covariance_floor", 0.0);

  if (pose_scale != 1.0 || pose_floor > 0.0) {
    for (auto & value : msg.pose.covariance) {
      value = std::max(value * pose_scale, pose_floor);
    }
  }

  if (twist_scale != 1.0 || twist_floor > 0.0) {
    for (auto & value : msg.twist.covariance) {
      value = std::max(value * twist_scale, twist_floor);
    }
  }
}

std::vector<FaultConfigField> OdomFaultInjector::static_config_schema()
{
  std::vector<FaultConfigField> schema;

  const auto add_field = [&schema](
    const std::string & key,
    const std::string & type,
    const std::string & description,
    std::optional<double> min_value = std::nullopt,
    std::optional<double> max_value = std::nullopt,
    std::optional<std::string> default_value = std::nullopt) {
      FaultConfigField field;
      field.key = key;
      field.type = type;
      field.description = description;
      field.min_value = min_value;
      field.max_value = max_value;
      field.default_value = default_value;
      schema.push_back(field);
    };

  add_field("drop_probability", "double", "Probability that an incoming message is dropped.", 0.0,
      1.0, "0.0");
  add_field("delay_ms", "int", "Delay applied before publishing the message, in milliseconds.", 0.0,
      std::nullopt, "0");
  add_field("x_bias", "double", "Additive bias applied to odometry pose x.", std::nullopt,
      std::nullopt, "0.0");
  add_field("y_bias", "double", "Additive bias applied to odometry pose y.", std::nullopt,
      std::nullopt, "0.0");
  add_field("yaw_bias_deg", "double",
      "Additive yaw bias applied to odometry orientation, in degrees.", std::nullopt, std::nullopt,
      "0.0");
  add_field("yaw_noise_stddev_deg", "double",
      "Standard deviation of Gaussian noise applied to odometry yaw, in degrees.", 0.0,
      std::nullopt, "0.0");
  add_field("x_noise_stddev", "double",
      "Standard deviation of Gaussian noise applied to odometry pose x.", 0.0, std::nullopt, "0.0");
  add_field("y_noise_stddev", "double",
      "Standard deviation of Gaussian noise applied to odometry pose y.", 0.0, std::nullopt, "0.0");
  add_field("pose_covariance_scale", "double",
      "Multiplier applied to every odometry pose covariance entry.", 0.0, std::nullopt, "1.0");
  add_field("twist_covariance_scale", "double",
      "Multiplier applied to every odometry twist covariance entry.", 0.0, std::nullopt, "1.0");
  add_field("pose_covariance_floor", "double",
      "Minimum value applied to every odometry pose covariance entry.", 0.0, std::nullopt, "0.0");
  add_field("twist_covariance_floor", "double",
      "Minimum value applied to every odometry twist covariance entry.", 0.0, std::nullopt, "0.0");

  return schema;
}

std::vector<FaultConfigField> OdomFaultInjector::config_schema() const
{
  return static_config_schema();
}

} // namespace ros2_fault_injection::injectors
