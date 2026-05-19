#include "ros2_fault_injection/odom_fault_injector.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <stdexcept>

namespace ros2_fault_injection
{
  namespace
  {

    double get_double(
        const FaultConfig &fault,
        const std::string &key,
        double fallback)
    {
      const auto it = fault.config.find(key);
      if (it == fault.config.end())
      {
        return fallback;
      }

      return std::stod(it->second);
    }

    int get_int(
        const FaultConfig &fault,
        const std::string &key,
        int fallback)
    {
      const auto it = fault.config.find(key);
      if (it == fault.config.end())
      {
        return fallback;
      }

      return std::stoi(it->second);
    }

  } // namespace

  OdomFaultInjector::OdomFaultInjector(
      rclcpp::Node &node,
      const InjectorConfig &config)
      : node_(node),
        config_(config),
        rng_(std::random_device{}())
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.qos_depth));

    // The injector sits in the odom data path: input odom is read, optionally
    // modified or delayed, then republished on the configured output topic.
    pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(
        config_.output_topic,
        qos);

    sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.input_topic,
        qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg)
        {
          on_odom(msg);
        });

    // Delayed odom samples are released by a small periodic flush instead of
    // creating one timer per message.
    timer_ = node_.create_wall_timer(
        std::chrono::milliseconds{10},
        [this]()
        {
          flush_delayed();
        });
  }

  std::string OdomFaultInjector::id() const
  {
    return config_.id;
  }

  void OdomFaultInjector::add_fault(const FaultConfig &fault_config)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    faults_[fault_config.id] = fault_config;
    active_[fault_config.id] = false;
  }

  void OdomFaultInjector::activate_fault(const std::string &fault_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (faults_.find(fault_id) == faults_.end())
    {
      RCLCPP_WARN(
          node_.get_logger(),
          "Cannot activate unknown odom fault '%s'",
          fault_id.c_str());
      return;
    }

    active_[fault_id] = true;
  }

  void OdomFaultInjector::deactivate_fault(const std::string &fault_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (faults_.find(fault_id) == faults_.end())
    {
      RCLCPP_WARN(
          node_.get_logger(),
          "Cannot deactivate unknown odom fault '%s'",
          fault_id.c_str());
      return;
    }

    active_[fault_id] = false;
  }

  void OdomFaultInjector::on_odom(
      const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (should_drop())
    {
      return;
    }

    // Copy first so the original incoming message is never mutated in place.
    auto out = *msg;

    apply_bias(out);
    apply_noise(out);

    const auto delay = active_delay();
    if (delay.count() > 0)
    {
      delayed_.push_back(
          DelayedOdom{
              out,
              node_.now() + rclcpp::Duration(delay)});
      return;
    }

    pub_->publish(out);
  }

  void OdomFaultInjector::flush_delayed()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = node_.now();

    while (!delayed_.empty() && delayed_.front().release_time <= now)
    {
      pub_->publish(delayed_.front().msg);
      delayed_.pop_front();
    }
  }

  bool OdomFaultInjector::should_drop()
  {
    double probability = 0.0;

    // If multiple active faults specify drop probability, use the strongest
    // one. This keeps overlapping drop faults predictable.
    for (const auto &[fault_id, is_active] : active_)
    {
      if (!is_active)
      {
        continue;
      }

      const auto &fault = faults_.at(fault_id);
      probability = std::max(
          probability,
          get_double(fault, "drop_probability", 0.0));
    }

    if (probability <= 0.0)
    {
      return false;
    }

    probability = std::clamp(probability, 0.0, 1.0);

    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(rng_) < probability;
  }

  void OdomFaultInjector::apply_bias(nav_msgs::msg::Odometry &msg)
  {
    double x_bias = 0.0;
    double y_bias = 0.0;

    // Biases stack additively, so separate faults can represent separate error
    // sources while still targeting the same odom stream.
    for (const auto &[fault_id, is_active] : active_)
    {
      if (!is_active)
      {
        continue;
      }

      const auto &fault = faults_.at(fault_id);

      x_bias += get_double(fault, "x_bias", 0.0);
      y_bias += get_double(fault, "y_bias", 0.0);
    }

    msg.pose.pose.position.x += x_bias;
    msg.pose.pose.position.y += y_bias;
  }

  void OdomFaultInjector::apply_noise(nav_msgs::msg::Odometry &msg)
  {
    double x_noise_stddev = 0.0;
    double y_noise_stddev = 0.0;

    for (const auto &[fault_id, is_active] : active_)
    {
      if (!is_active)
      {
        continue;
      }

      const auto &fault = faults_.at(fault_id);

      x_noise_stddev = std::max(
          x_noise_stddev,
          get_double(fault, "x_noise_stddev", 0.0));
      y_noise_stddev = std::max(
          y_noise_stddev,
          get_double(fault, "y_noise_stddev", 0.0));
    }

    if (x_noise_stddev > 0.0)
    {
      std::normal_distribution<double> distribution(0.0, x_noise_stddev);
      msg.pose.pose.position.x += distribution(rng_);
    }

    if (y_noise_stddev > 0.0)
    {
      std::normal_distribution<double> distribution(0.0, y_noise_stddev);
      msg.pose.pose.position.y += distribution(rng_);
    }
  }

  std::chrono::milliseconds OdomFaultInjector::active_delay()
  {
    int delay_ms = 0;

    // Use the largest active delay rather than summing delays. That avoids
    // surprising growth when two delay faults overlap.
    for (const auto &[fault_id, is_active] : active_)
    {
      if (!is_active)
      {
        continue;
      }

      const auto &fault = faults_.at(fault_id);
      delay_ms = std::max(delay_ms, get_int(fault, "delay_ms", 0));
    }

    return std::chrono::milliseconds{delay_ms};
  }

  bool OdomFaultInjector::has_fault(const std::string &fault_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return faults_.find(fault_id) != faults_.end();
  }

  std::vector<std::string> OdomFaultInjector::fault_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> ids;
    for (const auto &[id, _] : faults_)
    {
      ids.push_back(id);
    }
    return ids;
  }

  std::vector<std::string> OdomFaultInjector::active_fault_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> ids;
    for (const auto &[id, is_active] : active_)
    {
      if (is_active)
      {
        ids.push_back(id);
      }
    }
    return ids;
  }

} // namespace ros2_fault_injection
