#include "ros2_fault_injection/odom_fault_injector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <random>

namespace ros2_fault_injection {

OdomFaultInjector::OdomFaultInjector(rclcpp::Node& node, const InjectorConfig& config)
    : FaultInjectorBase(node, config) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.qos_depth));

  // The injector sits in the odom data path: input odom is read, optionally
  // modified or delayed, then republished on the configured output topic.
  pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.output_topic, qos);

  sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      config_.input_topic, qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

  // Delayed odom samples are released by a small periodic flush instead of
  // creating one timer per message.
  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]() { flush_delayed(); });
}

void OdomFaultInjector::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  // Copy first so the original incoming message is never mutated in place.
  auto out = *msg;

  apply_bias(out);
  apply_noise(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedOdom{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void OdomFaultInjector::flush_delayed() {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void OdomFaultInjector::apply_bias(nav_msgs::msg::Odometry& msg) {
  // Biases stack additively, so separate faults can represent separate error
  // sources while still targeting the same odom stream.
  msg.pose.pose.position.x += active_sum_double("x_bias", 0.0);
  msg.pose.pose.position.y += active_sum_double("y_bias", 0.0);
}

void OdomFaultInjector::apply_noise(nav_msgs::msg::Odometry& msg) {
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

void OdomFaultInjector::warn_unknown_config_keys(const FaultConfig& fault_config) const {
  static constexpr std::array<const char*, 6> known_keys = {
      "drop_probability", "x_bias", "y_bias", "x_noise_stddev", "y_noise_stddev", "delay_ms"};

  for (const auto& [key, value] : fault_config.config) {
    (void)value;
    const bool known =
        std::find(std::begin(known_keys), std::end(known_keys), key) != std::end(known_keys);
    if (!known) {
      RCLCPP_WARN(node_.get_logger(), "Fault '%s' has unknown config key '%s'",
                  fault_config.id.c_str(), key.c_str());
    }
  }
}

}  // namespace ros2_fault_injection
