// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/scan_fault_injector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

namespace ros2_fault_injection::injectors
{

namespace
{
constexpr double kRadiansToDegrees = 180.0 / M_PI;

double normalize_degrees(double degrees)
{
  while (degrees > 180.0) {
    degrees -= 360.0;
  }
  while (degrees < -180.0) {
    degrees += 360.0;
  }
  return degrees;
}

bool parse_double(const std::string & text, double & value)
{
  try {
    value = std::stod(text);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool angle_in_sector(double angle_deg, double min_deg, double max_deg)
{
  angle_deg = normalize_degrees(angle_deg);
  min_deg = normalize_degrees(min_deg);
  max_deg = normalize_degrees(max_deg);

  if (min_deg <= max_deg) {
    return angle_deg >= min_deg && angle_deg <= max_deg;
  }

  return angle_deg >= min_deg || angle_deg <= max_deg;
}

double parse_sector_value(const std::string & value)
{
  if (value == "inf" || value == "+inf") {
    return std::numeric_limits<double>::infinity();
  }
  if (value == "-inf") {
    return -std::numeric_limits<double>::infinity();
  }
  if (value == "nan") {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return std::stod(value);
}
}  // namespace

ScanFaultInjector::ScanFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

  pub_ = node_.create_publisher<sensor_msgs::msg::LaserScan>(config_.topic->output_topic, qos);

  sub_ = node_.create_subscription<sensor_msgs::msg::LaserScan>(
      config_.topic->input_topic, qos,
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {on_scan(msg);});

  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]() {flush_delayed();});
}

void ScanFaultInjector::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  auto out = *msg;
  apply_range_bias(out);
  apply_range_noise(out);
  apply_sector_dropout(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedScan{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void ScanFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void ScanFaultInjector::apply_range_bias(sensor_msgs::msg::LaserScan & msg)
{
  const double bias = active_sum_double("range_bias", 0.0);
  if (bias == 0.0) {
    return;
  }

  for (auto & range : msg.ranges) {
    if (!std::isfinite(range)) {
      continue;
    }

    range = std::clamp(static_cast<double>(range) + bias, static_cast<double>(msg.range_min),
                       static_cast<double>(msg.range_max));
  }
}

void ScanFaultInjector::apply_range_noise(sensor_msgs::msg::LaserScan & msg)
{
  const double noise_stddev = active_max_double("range_noise_stddev", 0.0);
  if (noise_stddev <= 0.0) {
    return;
  }

  std::normal_distribution<double> distribution(0.0, noise_stddev);

  for (auto & range : msg.ranges) {
    if (!std::isfinite(range)) {
      continue;
    }

    range = std::clamp(static_cast<double>(range) + distribution(rng_),
                       static_cast<double>(msg.range_min), static_cast<double>(msg.range_max));
  }
}

void ScanFaultInjector::apply_sector_dropout(sensor_msgs::msg::LaserScan & msg)
{
  if (msg.angle_increment == 0.0F) {
    return;
  }

  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto & fault = faults_.at(fault_id);

    const auto min_it = fault.config.find("sector_min_deg");
    const auto max_it = fault.config.find("sector_max_deg");
    if (min_it == fault.config.end() || max_it == fault.config.end()) {
      continue;
    }

    double sector_min_deg = 0.0;
    double sector_max_deg = 0.0;
    if (!parse_double(min_it->second, sector_min_deg) ||
      !parse_double(max_it->second, sector_max_deg))
    {
      RCLCPP_WARN(node_.get_logger(), "Fault '%s' has invalid sector bounds", fault.id.c_str());
      continue;
    }

    double replacement = std::numeric_limits<double>::infinity();
    const auto value_it = fault.config.find("sector_value");
    if (value_it != fault.config.end()) {
      try {
        replacement = parse_sector_value(value_it->second);
      } catch (const std::exception &) {
        RCLCPP_WARN(node_.get_logger(), "Fault '%s' has invalid sector_value '%s'",
                    fault.id.c_str(), value_it->second.c_str());
        continue;
      }
    }

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const double angle_rad =
        static_cast<double>(msg.angle_min) + static_cast<double>(i) * msg.angle_increment;
      const double angle_deg = angle_rad * kRadiansToDegrees;

      if (angle_in_sector(angle_deg, sector_min_deg, sector_max_deg)) {
        msg.ranges[i] = static_cast<float>(replacement);
      }
    }
  }
}

std::vector<FaultConfigField> ScanFaultInjector::static_config_schema()
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
  add_field("range_bias", "double", "Additive bias applied to each laser scan range.", std::nullopt,
      std::nullopt, "0.0");
  add_field("range_noise_stddev", "double",
      "Standard deviation of Gaussian noise applied to laser scan ranges.", 0.0, std::nullopt,
      "0.0");
  add_field("sector_min_deg", "double",
      "Start angle of the affected laser scan sector, in degrees.", std::nullopt, std::nullopt,
      "0.0");
  add_field("sector_max_deg", "double", "End angle of the affected laser scan sector, in degrees.",
      std::nullopt, std::nullopt, "0.0");
  add_field("sector_value", "special_float",
      "Replacement range value for points inside the affected sector.", std::nullopt, std::nullopt,
      "inf");

  return schema;
}

std::vector<FaultConfigField> ScanFaultInjector::config_schema() const
{
  return static_config_schema();
}

}  // namespace ros2_fault_injection::injectors
