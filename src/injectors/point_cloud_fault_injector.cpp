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

#include "ros2_fault_injection/core/dust_plume.hpp"

namespace ros2_fault_injection::injectors
{

namespace
{
std::string dust_model_for(const FaultConfig & fault)
{
  const auto it = fault.config.find("dust_model");
  if(it == fault.config.end()) {
    return "random";
  }
  return it->second;
}

double fault_double(
  const FaultConfig & fault,
  const std::string & key,
  double fallback)
{
  const auto it = fault.config.find(key);

  if (it == fault.config.end()) {
    return fallback;
  }

  return std::stod(it->second);
}

core::DustPlume plume_for(const FaultConfig & fault)
{
  core::DustPlume plume;
  plume.center_x = fault_double(fault, "plume_center_x", 0.0);
  plume.center_y = fault_double(fault, "plume_center_y", 0.0);
  plume.center_z = fault_double(fault, "plume_center_z", 0.0);
  plume.sigma_x = fault_double(fault, "plume_sigma_x", 1.0);
  plume.sigma_y = fault_double(fault, "plume_sigma_y", 1.0);
  plume.sigma_z = fault_double(fault, "plume_sigma_z", 1.0);

  plume.validate();
  return plume;
}
}


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
  add_field("dust_model", "string", "Dust model: random or plume.", std::nullopt, std::nullopt,
      "random");

  add_field("plume_center_x", "double",
    "Plume centre X in metres in the sensor frame.", std::nullopt, std::nullopt, "0.0");
  add_field("plume_center_y", "double",
    "Plume centre Y in metres in the sensor frame.", std::nullopt, std::nullopt, "0.0");
  add_field("plume_center_z", "double",
    "Plume centre Z in metres in the sensor frame.", std::nullopt, std::nullopt, "0.0");
  add_field("plume_sigma_x", "double",
    "Gaussian width along X in metres; must be greater than zero.", 0.0, std::nullopt, "1.0");
  add_field("plume_sigma_y", "double",
    "Gaussian width along Y in metres; must be greater than zero.", 0.0, std::nullopt, "1.0");
  add_field("plume_sigma_z", "double",
    "Gaussian width along Z in metres; must be greater than zero.", 0.0, std::nullopt, "1.0");
  add_field("plume_interaction_coefficient", "double",
    "Peak interaction rate in inverse metres; zero disables plume interactions.",
    0.0, std::nullopt, "0.0");
  add_field("plume_step_size", "double",
    "Maximum ray integration step in metres; must be greater than zero.",
    0.0, std::nullopt, "0.1");

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
  apply_random_dust_returns(out);

  std::vector<std::string> plume_ids;

  for (const auto & [fault_id, is_active] : active_) {
    if (is_active && dust_model_for(faults_.at(fault_id)) == "plume") {
      plume_ids.push_back(fault_id);
    }
  }

  std::sort(plume_ids.begin(), plume_ids.end());

  for (const auto & fault_id : plume_ids) {
    try {
      apply_plume_dust_returns(out, faults_.at(fault_id));
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "Skipping plume fault '%s': %s", fault_id.c_str(), error.what());
    } catch (const std::out_of_range & error) {
      RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "Skipping plume fault '%s': %s", fault_id.c_str(), error.what());
    }
  }

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

void PointCloudFaultInjector::apply_random_dust_returns(sensor_msgs::msg::PointCloud2 & msg)
{
  const double dust_return_probability =
    active_random_dust_max("dust_return_probability", 0.0);

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

  const double dust_min_range = active_random_dust_max("dust_min_range", 0.2);
  const double dust_max_range = active_random_dust_min("dust_max_range", 2.0);

  if (dust_max_range < dust_min_range) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 dust_max_range is less than dust_min_range; skipping dust returns");
    return;
  }

  std::bernoulli_distribution should_add_dust_return(dust_return_probability);
  std::uniform_real_distribution<float> dust_range_dist(
    static_cast<float>(dust_min_range), static_cast<float>(dust_max_range));
  const auto dust_intensity_scale = static_cast<float>(
    active_random_dust_product_or_default("dust_intensity_scale", 0.2));

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

double PointCloudFaultInjector::active_random_dust_max(
  const std::string & key, double fallback) const
{
  double value = fallback;

  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto & fault = faults_.at(fault_id);

    if (dust_model_for(fault) != "random") {
      continue;
    }

    const auto it = fault.config.find(key);
    if (it != fault.config.end()) {
      value = std::max(value, std::stod(it->second));
    }
  }

  return value;
}

double PointCloudFaultInjector::active_random_dust_min(
  const std::string & key, double fallback) const
{
  double value = fallback;

  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto & fault = faults_.at(fault_id);

    if (dust_model_for(fault) != "random") {
      continue;
    }

    const auto it = fault.config.find(key);
    if (it != fault.config.end()) {
      value = std::min(value, std::stod(it->second));
    }
  }

  return value;
}

double PointCloudFaultInjector::active_random_dust_product_or_default(
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
    if (dust_model_for(fault) != "random") {
      continue;
    }
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

void PointCloudFaultInjector::apply_plume_dust_returns(
  sensor_msgs::msg::PointCloud2 & msg,
  const FaultConfig & fault)
{
  const auto plume = plume_for(fault);

  const double coefficient =
    fault_double(fault, "plume_interaction_coefficient", 0.0);

  const double step_size =
    fault_double(fault, "plume_step_size", 0.1);

  if (coefficient == 0.0) {
    return;
  }

  if (!has_float32_field(msg, "x") ||
    !has_float32_field(msg, "y") ||
    !has_float32_field(msg, "z"))
  {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PointCloud2 requires float32 x, y, z fields; skipping plume dust");
    return;
  }

  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) ||
      !std::isfinite(*y) ||
      !std::isfinite(*z))
    {
      continue;
    }

    const double range = std::hypot(
    static_cast<double>(*x),
    static_cast<double>(*y),
    static_cast<double>(*z));

    if (range <= std::numeric_limits<float>::epsilon()) {
      continue;
    }

    std::optional<double> distance;

    try {
      distance = plume.first_interaction_distance(
    *x, *y, *z, coefficient, step_size, uniform(rng_));
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 5000,
    "Skipping plume dust for a point: %s", error.what());
      continue;
    }

    if (!distance.has_value()) {
      continue;
    }

    const double scale = distance.value() / range;
    *x = static_cast<float>(*x * scale);
    *y = static_cast<float>(*y * scale);
    *z = static_cast<float>(*z * scale);
  }
}

}  // namespace ros2_fault_injection::injectors
