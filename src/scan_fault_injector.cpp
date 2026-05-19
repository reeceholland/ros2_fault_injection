#include "ros2_fault_injection/scan_fault_injector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <random>

namespace ros2_fault_injection
{

ScanFaultInjector::ScanFaultInjector(
    rclcpp::Node &node,
    const InjectorConfig &config)
    : FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.qos_depth));

  pub_ = node_.create_publisher<sensor_msgs::msg::LaserScan>(
      config_.output_topic,
      qos);

  sub_ = node_.create_subscription<sensor_msgs::msg::LaserScan>(
      config_.input_topic,
      qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        on_scan(msg);
      });

  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds{10},
      [this]()
      {
        flush_delayed();
      });
}

void ScanFaultInjector::on_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop())
  {
    return;
  }

  auto out = *msg;
  apply_range_bias(out);
  apply_range_noise(out);

  const auto delay = active_delay();
  if (delay.count() > 0)
  {
    delayed_.push_back(DelayedScan{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void ScanFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now)
  {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void ScanFaultInjector::apply_range_bias(sensor_msgs::msg::LaserScan &msg)
{
  const double bias = active_sum_double("range_bias", 0.0);
  if (bias == 0.0)
  {
    return;
  }

  for (auto &range : msg.ranges)
  {
    if (!std::isfinite(range))
    {
      continue;
    }

    range = std::clamp(
        static_cast<double>(range) + bias,
        static_cast<double>(msg.range_min),
        static_cast<double>(msg.range_max));
  }
}

void ScanFaultInjector::apply_range_noise(sensor_msgs::msg::LaserScan &msg)
{
  const double noise_stddev = active_max_double("range_noise_stddev", 0.0);
  if (noise_stddev <= 0.0)
  {
    return;
  }

  std::normal_distribution<double> distribution(0.0, noise_stddev);

  for (auto &range : msg.ranges)
  {
    if (!std::isfinite(range))
    {
      continue;
    }

    range = std::clamp(
        static_cast<double>(range) + distribution(rng_),
        static_cast<double>(msg.range_min),
        static_cast<double>(msg.range_max));
  }
}

void ScanFaultInjector::warn_unknown_config_keys(const FaultConfig &fault_config) const
{
  static constexpr std::array<const char *, 4> known_keys = {
      "drop_probability",
      "delay_ms",
      "range_bias",
      "range_noise_stddev"};

  for (const auto &[key, value] : fault_config.config)
  {
    (void)value;
    const bool known =
        std::find(std::begin(known_keys), std::end(known_keys), key) != std::end(known_keys);
    if (!known)
    {
      RCLCPP_WARN(
          node_.get_logger(),
          "Fault '%s' has unknown config key '%s'",
          fault_config.id.c_str(),
          key.c_str());
    }
  }
}

} // namespace ros2_fault_injection
