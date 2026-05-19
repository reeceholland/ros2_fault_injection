#include "ros2_fault_injection/scan_fault_injector.hpp"

#include <chrono>
#include <random>
#include <stdexcept>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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

  ScanFaultInjector::ScanFaultInjector(
      rclcpp::Node &node,
      const InjectorConfig &config)
      : node_(node),
        config_(config),
        rng_(std::random_device{}())
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

  std::string ScanFaultInjector::id() const
  {
    return config_.id;
  }

  void ScanFaultInjector::add_fault(const FaultConfig &fault_config)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    faults_[fault_config.id] = fault_config;
    active_[fault_config.id] = false;
  }

  void ScanFaultInjector::activate_fault(const std::string &fault_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (faults_.find(fault_id) == faults_.end())
    {
      RCLCPP_WARN(
          node_.get_logger(),
          "Cannot activate unknown scan fault '%s'",
          fault_id.c_str());
      return;
    }

    active_[fault_id] = true;
  }

  void ScanFaultInjector::deactivate_fault(const std::string &fault_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (faults_.find(fault_id) == faults_.end())
    {
      RCLCPP_WARN(
          node_.get_logger(),
          "Cannot deactivate unknown scan fault '%s'",
          fault_id.c_str());
      return;
    }

    active_[fault_id] = false;
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

  bool ScanFaultInjector::should_drop()
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
    double bias = 0.0;

    for (const auto &[fault_id, is_active] : active_)
    {
      if (!is_active)
      {
        continue;
      }

      const auto &fault = faults_.at(fault_id);
      bias = get_double(fault, "range_bias", 0.0);
    }

    for (auto &range : msg.ranges)
    {
      range += bias;
    }
  }

  void ScanFaultInjector::apply_range_noise(sensor_msgs::msg::LaserScan &msg)
  {
    double noise_stddev = 0.0;

    for (const auto &[fault_id, is_active] : active_)
    {
      if (!is_active)
      {
        continue;
      }

      const auto &fault = faults_.at(fault_id);
      noise_stddev = std::max(
          noise_stddev,
          get_double(fault, "range_noise_stddev", 0.0));
    }

    if (noise_stddev <= 0.0)
    {
      return;
    }

    std::normal_distribution<double> distribution(0.0, noise_stddev);

    for (auto &range : msg.ranges)
    {
      range += distribution(rng_);
    }
  }

  std::chrono::milliseconds ScanFaultInjector::active_delay()
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

  bool ScanFaultInjector::has_fault(const std::string &fault_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return faults_.find(fault_id) != faults_.end();
  }

  std::vector<std::string> ScanFaultInjector::fault_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> ids;
    for (const auto &[id, _] : faults_)
    {
      ids.push_back(id);
    }
    return ids;
  }

  std::vector<std::string> ScanFaultInjector::active_fault_ids() const
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