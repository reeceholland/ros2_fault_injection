// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/tf_fault_injector.hpp"

#include <cmath>
#include <string>
#include <thread>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ros2_fault_injection::injectors
{
namespace
{

constexpr double kDegreesToRadians = M_PI / 180.0;

bool parse_double(const std::string & text, double & value)
{
  try {
    size_t parsed = 0;
    value = std::stod(text, &parsed);
    return parsed == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

double get_double(const FaultConfig & fault, const std::string & key, double fallback = 0.0)
{
  const auto it = fault.config.find(key);
  if (it == fault.config.end()) {
    return fallback;
  }

  double value = fallback;
  if (!parse_double(it->second, value)) {
    return fallback;
  }

  return value;
}

bool matches_frame(
  const FaultConfig & fault,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto parent_it = fault.config.find("parent_frame");
  const auto child_it = fault.config.find("child_frame");

  if (parent_it == fault.config.end() || child_it == fault.config.end()) {
    return false;
  }

  return parent_it->second == transform.header.frame_id &&
         child_it->second == transform.child_frame_id;
}

}  // namespace

TfFaultInjector::TfFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(config.topic->qos_depth);

  publisher_ = node_.create_publisher<tf2_msgs::msg::TFMessage>(config.topic->output_topic, qos);

  subscription_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
      config.topic->input_topic, qos,
    [this](const tf2_msgs::msg::TFMessage & msg) {on_message(msg);});

  RCLCPP_INFO(node_.get_logger(), "TF fault injector running: %s -> %s",
              config.topic->input_topic.c_str(), config.topic->output_topic.c_str());
}

void TfFaultInjector::on_message(const tf2_msgs::msg::TFMessage & msg)
{
  const auto delay = active_delay();
  if (delay.count() > 0) {
    std::this_thread::sleep_for(delay);
  }

  tf2_msgs::msg::TFMessage output;

  for (const auto & input_transform : msg.transforms) {
    auto transform = input_transform;

    if (should_drop_transform(transform)) {
      continue;
    }

    apply_translation_bias(transform);
    apply_rotation_bias(transform);

    output.transforms.push_back(transform);
  }

  publisher_->publish(output);
}

bool TfFaultInjector::should_drop_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  for (const auto & fault_id : active_fault_ids()) {
    const auto & fault = faults_.at(fault_id);

    if (!matches_frame(fault, transform)) {
      continue;
    }

    const double probability = get_double(fault, "drop_probability", 0.0);
    if (probability <= 0.0) {
      continue;
    }

    if (probability >= 1.0) {
      return true;
    }

    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    if (distribution(rng_) < probability) {
      return true;
    }
  }

  return false;
}

void TfFaultInjector::apply_translation_bias(geometry_msgs::msg::TransformStamped & transform)
{
  for (const auto & fault_id : active_fault_ids()) {
    const auto & fault = faults_.at(fault_id);

    if (!matches_frame(fault, transform)) {
      continue;
    }

    transform.transform.translation.x += get_double(fault, "x_bias", 0.0);
    transform.transform.translation.y += get_double(fault, "y_bias", 0.0);
    transform.transform.translation.z += get_double(fault, "z_bias", 0.0);
  }
}

void TfFaultInjector::apply_rotation_bias(geometry_msgs::msg::TransformStamped & transform)
{
  for (const auto & fault_id : active_fault_ids()) {
    const auto & fault = faults_.at(fault_id);

    if (!matches_frame(fault, transform)) {
      continue;
    }

    const double roll_bias = get_double(fault, "roll_bias_deg", 0.0) * kDegreesToRadians;
    const double pitch_bias = get_double(fault, "pitch_bias_deg", 0.0) * kDegreesToRadians;
    const double yaw_bias = get_double(fault, "yaw_bias_deg", 0.0) * kDegreesToRadians;

    if (roll_bias == 0.0 && pitch_bias == 0.0 && yaw_bias == 0.0) {
      continue;
    }

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    roll += roll_bias;
    pitch += pitch_bias;
    yaw += yaw_bias;

    q.setRPY(roll, pitch, yaw);
    q.normalize();

    transform.transform.rotation = tf2::toMsg(q);
  }
}

std::vector<FaultConfigField> TfFaultInjector::static_config_schema()
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

  add_field("parent_frame", "non_empty_string", "Parent frame id of the transform to affect.",
      std::nullopt, std::nullopt, std::nullopt);
  add_field("child_frame", "non_empty_string", "Child frame id of the transform to affect.",
      std::nullopt, std::nullopt, std::nullopt);
  add_field("drop_probability", "double", "Probability that an incoming message is dropped.", 0.0,
      1.0, "0.0");
  add_field("delay_ms", "int", "Delay applied before publishing the message, in milliseconds.", 0.0,
      std::nullopt, "0");
  add_field("x_bias", "double", "Additive translation bias applied to transform x.", std::nullopt,
      std::nullopt, "0.0");
  add_field("y_bias", "double", "Additive translation bias applied to transform y.", std::nullopt,
      std::nullopt, "0.0");
  add_field("z_bias", "double", "Additive translation bias applied to transform z.", std::nullopt,
      std::nullopt, "0.0");
  add_field("roll_bias_deg", "double",
      "Additive roll bias applied to transform rotation, in degrees.", std::nullopt, std::nullopt,
      "0.0");
  add_field("pitch_bias_deg", "double",
      "Additive pitch bias applied to transform rotation, in degrees.", std::nullopt, std::nullopt,
      "0.0");
  add_field("yaw_bias_deg", "double",
      "Additive yaw bias applied to transform rotation, in degrees.", std::nullopt, std::nullopt,
      "0.0");

  return schema;
}

std::vector<FaultConfigField> TfFaultInjector::config_schema() const
{
  return static_config_schema();
}

}  // namespace ros2_fault_injection::injectors
