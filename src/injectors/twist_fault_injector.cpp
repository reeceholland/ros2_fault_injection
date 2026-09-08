// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/twist_fault_injector.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ros2_fault_injection::injectors
{

TwistFaultInjector::TwistFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

  pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(config_.topic->output_topic, qos);

  sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        config_.topic->input_topic, qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg)
    {on_twist(msg);});

  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]()
      {flush_delayed();});

  RCLCPP_INFO(node_.get_logger(), "Twist fault injector running: %s -> %s",
                config_.topic->input_topic.c_str(), config_.topic->output_topic.c_str());
}

bool TwistFaultInjector::stale_replay_enabled() const
{
  return active_bool("stale_replay_enabled", false);
}

std::chrono::milliseconds TwistFaultInjector::stale_replay_duration() const
{
  return std::chrono::milliseconds{active_max_int("stale_replay_duration_ms", 500)};
}

double TwistFaultInjector::active_product_double(const std::string & key, double fallback) const
{
  double value = fallback;

  for (const auto &[fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto & fault = faults_.at(fault_id);
    const auto it = fault.config.find(key);
    if (it != fault.config.end()) {
      value *= std::stod(it->second);
    }
  }

  return value;
}

void TwistFaultInjector::apply_scale(geometry_msgs::msg::Twist & msg) const
{
  msg.linear.x *= active_product_double("linear_x_scale", 1.0);
  msg.angular.z *= active_product_double("angular_z_scale", 1.0);
}

void TwistFaultInjector::apply_clamp(geometry_msgs::msg::Twist & msg) const
{
  const double max_linear_x = active_max_double("max_linear_x", -1.0);
  if (max_linear_x >= 0.0) {
    msg.linear.x = std::clamp(msg.linear.x, -max_linear_x, max_linear_x);
  }

  const double max_angular_z = active_max_double("max_angular_z", -1.0);
  if (max_angular_z >= 0.0) {
    msg.angular.z = std::clamp(msg.angular.z, -max_angular_z, max_angular_z);
  }
}

void TwistFaultInjector::apply_force_stop(geometry_msgs::msg::Twist & msg) const
{
  if (!active_bool("force_stop", false)) {
    return;
  }

  msg = geometry_msgs::msg::Twist{};
}

void TwistFaultInjector::on_twist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  const auto now = node_.now();
  auto out = *msg;

  if (stale_replay_enabled() && last_command_.has_value()) {
    const auto max_age = rclcpp::Duration(stale_replay_duration());
    const auto age = now - last_command_time_;

    if (age <= max_age) {
      out = last_command_.value();
    } else {
      last_command_ = *msg;
      last_command_time_ = now;
    }
  } else {
    last_command_ = *msg;
    last_command_time_ = now;
  }

  apply_scale(out);
  apply_clamp(out);
  apply_force_stop(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedTwist{out, now + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void TwistFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

std::vector<FaultConfigField> TwistFaultInjector::static_config_schema()
{
  std::vector<FaultConfigField> schema;

  const auto add_field = [&schema](
    const std::string & key,
    const std::string & type,
    const std::string & description,
    std::optional<double> min_value = std::nullopt,
    std::optional<double> max_value = std::nullopt,
    std::optional<std::string> default_value = std::nullopt)
    {
      FaultConfigField field;
      field.key = key;
      field.type = type;
      field.description = description;
      field.min_value = min_value;
      field.max_value = max_value;
      field.default_value = default_value;
      schema.push_back(field);
    };

  add_field("drop_probability", "double", "Probability that an incoming command is dropped.",
              0.0, 1.0, "0.0");
  add_field("delay_ms", "int", "Delay applied before publishing the command, in milliseconds.",
              0.0, std::nullopt, "0");
  add_field("linear_x_scale", "double", "Scale factor applied to linear.x commands.",
              0.0, std::nullopt, "1.0");
  add_field("angular_z_scale", "double", "Scale factor applied to angular.z commands.",
              0.0, std::nullopt, "1.0");
  add_field("max_linear_x", "double", "Symmetric maximum absolute linear.x command.",
              0.0, std::nullopt, "");
  add_field("max_angular_z", "double", "Symmetric maximum absolute angular.z command.",
              0.0, std::nullopt, "");
  add_field("force_stop", "bool", "When true, publish a zero Twist command.",
              std::nullopt, std::nullopt, "false");
  add_field(
        "stale_replay_enabled",
        "bool",
        "When true, replay the last command instead of publishing the newest command.",
        std::nullopt,
        std::nullopt,
        "false");

  add_field(
        "stale_replay_duration_ms",
        "int",
        "Maximum age of a stored command that may be replayed, in milliseconds.",
        0.0,
        std::nullopt,
        "500");

  return schema;
}

std::vector<FaultConfigField> TwistFaultInjector::config_schema() const
{
  return static_config_schema();
}

} // namespace ros2_fault_injection::injectors
