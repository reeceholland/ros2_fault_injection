// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/point_cloud_fault_injector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ros2_fault_injection::injectors
{

PointCloudFaultInjector::PointCloudFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

  pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(config_.topic->output_topic, qos);

  sub_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.topic->input_topic, qos,
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_point_cloud(msg);});

  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]() {flush_delayed();});

  RCLCPP_INFO(node_.get_logger(), "PointCloud2 fault injector running: %s -> %s",
    config_.topic->input_topic.c_str(), config_.topic->output_topic.c_str());
}

std::vector<FaultConfigField> PointCloudFaultInjector::static_config_schema()
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

  add_field("drop_probability", "double",
      "Probability that an incoming point cloud message is dropped.",
    0.0, 1.0, "0.0");
  add_field("delay_ms", "int", "Delay applied before publishing the point cloud, in milliseconds.",
    0.0, std::nullopt, "0");
  add_field("point_dropout_probability", "double",
    "Probability that each individual point is invalidated with NaN coordinates.", 0.0, 1.0,
    "0.0");
  add_field("range_noise_stddev", "double",
    "Standard deviation of Gaussian noise applied along each point ray.", 0.0, std::nullopt,
    "0.0");
  add_field("dust_return_probability", "double",
    "Probability that each point is converted into a short-range dust return.", 0.0, 1.0, "0.0");
  add_field("dust_min_range", "double", "Minimum range for generated dust returns.", 0.0,
    std::nullopt, "0.2");
  add_field("dust_max_range", "double", "Maximum range for generated dust returns.", 0.0,
    std::nullopt, "2.0");
  add_field("dust_intensity_scale", "double",
    "Multiplier applied only to points converted into dust returns.", 0.0, std::nullopt, "0.2");
  add_field("intensity_scale", "double",
    "Multiplier applied to float32 intensity values when an intensity field exists.", 0.0,
    std::nullopt, "1.0");

  return schema;
}

std::vector<FaultConfigField> PointCloudFaultInjector::config_schema() const
{
  return static_config_schema();
}

void PointCloudFaultInjector::on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  auto out = *msg;
  apply_point_dropout(out);
  apply_range_noise(out);
  apply_dust_returns(out);
  apply_intensity_scale(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedPointCloud{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void PointCloudFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void PointCloudFaultInjector::apply_point_dropout(sensor_msgs::msg::PointCloud2 & msg)
{
  const double drop_probability = active_max_double("point_dropout_probability", 0.0);

  if (drop_probability <= 0.0) {
    return;
  }

  if (!has_float32_field(msg, "x") || !has_float32_field(msg, "y") || !has_float32_field(msg,
      "z"))
  {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 message does not have float32 x, y, z fields; skipping point dropout");
    return;
  }

  std::bernoulli_distribution should_drop_point(drop_probability);

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

  for (; x != x.end(); ++x, ++y, ++z) {
    if (!should_drop_point(rng_)) {
      continue;
    }

    *x = std::numeric_limits<float>::quiet_NaN();
    *y = std::numeric_limits<float>::quiet_NaN();
    *z = std::numeric_limits<float>::quiet_NaN();
  }
}

void PointCloudFaultInjector::apply_range_noise(sensor_msgs::msg::PointCloud2 & msg)
{
  const double range_noise_stddev = active_max_double("range_noise_stddev", 0.0);

  if (range_noise_stddev <= 0.0) {
    return;
  }

  if (!has_float32_field(msg, "x") || !has_float32_field(msg, "y") || !has_float32_field(msg,
      "z"))
  {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 message does not have float32 x, y, z fields; skipping range noise");
    return;
  }

  std::normal_distribution<float> noise_dist(0.0F, static_cast<float>(range_noise_stddev));

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }

    const float range = std::sqrt((*x * *x) + (*y * *y) + (*z * *z));
    if (range <= std::numeric_limits<float>::epsilon()) {
      continue;
    }

    const float noisy_range = std::max(0.0F, range + noise_dist(rng_));
    const float scale = noisy_range / range;
    *x *= scale;
    *y *= scale;
    *z *= scale;
  }
}

void PointCloudFaultInjector::apply_dust_returns(sensor_msgs::msg::PointCloud2 & msg)
{
  const double dust_return_probability = active_max_double("dust_return_probability", 0.0);

  if (dust_return_probability <= 0.0) {
    return;
  }

  if (!has_float32_field(msg, "x") || !has_float32_field(msg, "y") || !has_float32_field(msg,
      "z"))
  {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 message does not have float32 x, y, z fields; skipping dust returns");
    return;
  }

  const double dust_min_range = active_max_double("dust_min_range", 0.2);
  const double dust_max_range = active_min_double("dust_max_range", 2.0);
  if (dust_max_range < dust_min_range) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 dust_max_range is less than dust_min_range; skipping dust returns");
    return;
  }

  std::bernoulli_distribution should_add_dust_return(dust_return_probability);
  std::uniform_real_distribution<float> dust_range_dist(
    static_cast<float>(dust_min_range), static_cast<float>(dust_max_range));
  const auto dust_intensity_scale =
    static_cast<float>(active_product_double_or_default("dust_intensity_scale", 0.2));

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

  if (has_float32_field(msg, "intensity")) {
    sensor_msgs::PointCloud2Iterator<float> intensity(msg, "intensity");
    for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
      if (!should_add_dust_return(rng_)) {
        continue;
      }

      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }

      const float range = std::sqrt((*x * *x) + (*y * *y) + (*z * *z));
      if (range <= std::numeric_limits<float>::epsilon()) {
        continue;
      }

      const float dust_range = dust_range_dist(rng_);
      const float scale = dust_range / range;
      *x *= scale;
      *y *= scale;
      *z *= scale;
      *intensity *= dust_intensity_scale;
    }
    return;
  }

  for (; x != x.end(); ++x, ++y, ++z) {
    if (!should_add_dust_return(rng_)) {
      continue;
    }

    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }

    const float range = std::sqrt((*x * *x) + (*y * *y) + (*z * *z));
    if (range <= std::numeric_limits<float>::epsilon()) {
      continue;
    }

    const float dust_range = dust_range_dist(rng_);
    const float scale = dust_range / range;
    *x *= scale;
    *y *= scale;
    *z *= scale;
  }
}

void PointCloudFaultInjector::apply_intensity_scale(sensor_msgs::msg::PointCloud2 & msg)
{
  const double intensity_scale = active_product_double("intensity_scale", 1.0);
  if (intensity_scale == 1.0) {
    return;
  }

  if (!has_float32_field(msg, "intensity")) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 message does not have a float32 intensity field; skipping intensity scale");
    return;
  }

  sensor_msgs::PointCloud2Iterator<float> intensity(msg, "intensity");
  for (; intensity != intensity.end(); ++intensity) {
    *intensity = static_cast<float>(*intensity * intensity_scale);
  }
}

double PointCloudFaultInjector::active_min_double(
  const std::string & key,
  double fallback) const
{
  double value = fallback;

  for (const auto &[fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto & fault = faults_.at(fault_id);
    const auto it = fault.config.find(key);
    if (it != fault.config.end()) {
      value = std::min(value, std::stod(it->second));
    }
  }

  return value;
}

double PointCloudFaultInjector::active_product_double(
  const std::string & key,
  double fallback) const
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

double PointCloudFaultInjector::active_product_double_or_default(
  const std::string & key,
  double default_when_unconfigured) const
{
  double value = 1.0;
  bool has_configured_value = false;

  for (const auto &[fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto & fault = faults_.at(fault_id);
    const auto it = fault.config.find(key);
    if (it != fault.config.end()) {
      value *= std::stod(it->second);
      has_configured_value = true;
    }
  }

  if (!has_configured_value) {
    return default_when_unconfigured;
  }

  return value;
}

bool PointCloudFaultInjector::has_field(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & field_name) const
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field)
    {
      return field.name == field_name;
    });
}

bool PointCloudFaultInjector::has_float32_field(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & field_name) const
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field)
    {
      return field.name == field_name &&
             field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    });
}

}  // namespace ros2_fault_injection::injectors
