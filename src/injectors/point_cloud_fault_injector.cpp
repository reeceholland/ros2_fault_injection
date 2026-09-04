// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/point_cloud_fault_injector.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ros2_fault_injection::injectors
{
  PointCloudFaultInjector::PointCloudFaultInjector(rclcpp::Node &node, const InjectorConfig &config)
      : FaultInjectorBase(node, config)
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

    pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(config_.topic->output_topic, qos);

    sub_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.topic->input_topic, qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
        { on_point_cloud(msg); });

    timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]()
                                     { flush_delayed(); });
  }

  std::vector<FaultConfigField> PointCloudFaultInjector::static_config_schema()
  {
    std::vector<FaultConfigField> schema;
    const auto add_field = [&schema](const std::string &key, const std::string &type,
                                     const std::string &description,
                                     const std::optional<double> &min_value = std::nullopt,
                                     const std::optional<double> &max_value = std::nullopt,
                                     const std::optional<std::string> &default_value = std::nullopt)
    {
      schema.push_back(FaultConfigField{key, type, description, min_value, max_value, default_value});
    };

    add_field("drop_probability", "double", "Probability of dropping a whole point cloud message", 0.0, 1.0, "0.0");
    add_field("delay_ms", "int", "Delay applied before publishing the message, in milliseconds.", 0.0, std::nullopt, "0");
    add_field("point_dropout_probability", "double", "Probability of invalidating each individual point in the point cloud", 0.0, 1.0, "0.0");
    add_field("range_noise_stddev", "double", "Standard deviation of Gaussian noise applied along the point range direction", 0.0, std::nullopt, "0.0");
    add_field("dust_point_probability", "double", "Probability of converting an existing point into a dust-like near return", 0.0, 1.0, "0.0");
    add_field("dust_min_range", "double", "Minimum range for generated dust-like returns", 0.0, std::nullopt, "0.2");
    add_field("dust_max_range", "double", "Maximum range for generated dust-like returns", 0.0, std::nullopt, "5.0");
    add_field("intensity_scale", "double", "Multiplier applied to intensity values when an intensity field exists", 0.0, std::nullopt, "1.0");

    return schema;
  }

  std::vector<FaultConfigField> PointCloudFaultInjector::config_schema() const
  {
    return static_config_schema();
  }

  void PointCloudFaultInjector::on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (should_drop())
    {
      return;
    }

    auto out = *msg;
    apply_point_dropout(out);
    apply_range_noise(out);
    apply_dust_returns(out);
    apply_intensity_scale(out);

    const auto delay = active_delay();
    if (delay.count() > 0)
    {
      delayed_.push_back(DelayedPointCloud{out, node_.now() + rclcpp::Duration(delay)});
      return;
    }

    pub_->publish(out);
  }

  void PointCloudFaultInjector::flush_delayed()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = node_.now();

    while (!delayed_.empty() && delayed_.front().release_time <= now)
    {
      pub_->publish(delayed_.front().msg);
      delayed_.pop_front();
    }
  }

  void PointCloudFaultInjector::apply_point_dropout(sensor_msgs::msg::PointCloud2 &msg)
  {
    const double drop_probability = active_max_double("point_dropout_probability", 0.0);

    if (drop_probability <= 0.0)
    {
      return;
    }

    if (!has_field(msg, "x") || !has_field(msg, "y") || !has_field(msg, "z"))
    {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "PointCloud2 message does not have x, y, z fields; skipping point dropout");
      return;
    }

    std::bernoulli_distribution should_drop_point(drop_probability);

    sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

    for (; x != x.end(); ++x, ++y, ++z)
    {
      if (!should_drop_point(rng_))
      {
        continue;
      }

      *x = std::numeric_limits<float>::quiet_NaN();
      *y = std::numeric_limits<float>::quiet_NaN();
      *z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  void PointCloudFaultInjector::apply_range_noise(sensor_msgs::msg::PointCloud2 &msg)
  {
    const double range_noise_stddev = active_max_double("range_noise_stddev", 0.0);

    if (range_noise_stddev <= 0.0)
    {
      return;
    }

    if (!has_field(msg, "x") || !has_field(msg, "y") || !has_field(msg, "z"))
    {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "PointCloud2 message does not have x, y, z fields; skipping range noise");
      return;
    }

    std::normal_distribution<float> noise_dist(0.0, range_noise_stddev);

    sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

    for (; x != x.end(); ++x, ++y, ++z)
    {
      if (std::isnan(*x) || std::isnan(*y) || std::isnan(*z))
      {
        continue;
      }

      const float noise = noise_dist(rng_);
      *x += noise;
      *y += noise;
      *z += noise;
    }
  }

  void PointCloudFaultInjector::apply_dust_returns(sensor_msgs::msg::PointCloud2 &msg)
  {
    const double dust_return_probability = active_max_double("dust_return_probability", 0.0);

    if (dust_return_probability <= 0.0)
    {
      return;
    }

    if (!has_field(msg, "x") || !has_field(msg, "y") || !has_field(msg, "z"))
    {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "PointCloud2 message does not have x, y, z fields; skipping dust returns");
      return;
    }

    std::bernoulli_distribution should_add_dust_return(dust_return_probability);

    sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(msg, "z");

    for (; x != x.end(); ++x, ++y, ++z)
    {
      if (!should_add_dust_return(rng_))
      {
        continue;
      }

      *x = std::numeric_limits<float>::quiet_NaN();
      *y = std::numeric_limits<float>::quiet_NaN();
      *z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  bool PointCloudFaultInjector::has_field(
      const sensor_msgs::msg::PointCloud2 &msg,
      const std::string &field_name) const
  {
    return std::any_of(
        msg.fields.begin(), msg.fields.end(),
        [&field_name](const sensor_msgs::msg::PointField &field)
        {
          return field.name == field_name;
        });
  }
} // namespace ros2_fault_injection::injectors