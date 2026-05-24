// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/imu_fault_injector.hpp"

namespace ros2_fault_injection
{

ImuFaultInjector::ImuFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

  pub_ = node_.create_publisher<sensor_msgs::msg::Imu>(config_.topic->output_topic, qos);

  sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      config_.topic->input_topic, qos,
    [this](sensor_msgs::msg::Imu::SharedPtr msg) {on_imu(msg);});

  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]() {flush_delayed();});
}

void ImuFaultInjector::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  auto out = *msg;

  apply_angular_velocity_bias(out);
  apply_angular_velocity_noise(out);
  apply_linear_acceleration_bias(out);
  apply_linear_acceleration_noise(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedImu{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void ImuFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void ImuFaultInjector::apply_angular_velocity_bias(sensor_msgs::msg::Imu & msg)
{
  const double bias = active_sum_double("angular_velocity_z_bias", 0.0);

  if (bias == 0.0) {
    return;
  }
  msg.angular_velocity.z += bias;
}

void ImuFaultInjector::apply_angular_velocity_noise(sensor_msgs::msg::Imu & msg)
{
  const double stddev = active_max_double("angular_velocity_z_noise_stddev", 0.0);

  if (stddev == 0.0) {
    return;
  }

  std::normal_distribution<double> dist(0.0, stddev);
  msg.angular_velocity.z += dist(rng_);
}

void ImuFaultInjector::apply_linear_acceleration_bias(sensor_msgs::msg::Imu & msg)
{
  const double x_bias = active_sum_double("linear_acceleration_x_bias", 0.0);
  const double y_bias = active_sum_double("linear_acceleration_y_bias", 0.0);
  const double z_bias = active_sum_double("linear_acceleration_z_bias", 0.0);

  if (x_bias == 0.0 && y_bias == 0.0 && z_bias == 0.0) {
    return;
  }

  msg.linear_acceleration.x += x_bias;
  msg.linear_acceleration.y += y_bias;
  msg.linear_acceleration.z += z_bias;
}

void ImuFaultInjector::apply_linear_acceleration_noise(sensor_msgs::msg::Imu & msg)
{
  const double x_stddev = active_max_double("linear_acceleration_x_noise_stddev", 0.0);
  const double y_stddev = active_max_double("linear_acceleration_y_noise_stddev", 0.0);
  const double z_stddev = active_max_double("linear_acceleration_z_noise_stddev", 0.0);

  if (x_stddev == 0.0 && y_stddev == 0.0 && z_stddev == 0.0) {
    return;
  }

  std::normal_distribution<double> x_dist(0.0, x_stddev);
  std::normal_distribution<double> y_dist(0.0, y_stddev);
  std::normal_distribution<double> z_dist(0.0, z_stddev);
  msg.linear_acceleration.x += x_dist(rng_);
  msg.linear_acceleration.y += y_dist(rng_);
  msg.linear_acceleration.z += z_dist(rng_);
}

}  // namespace ros2_fault_injection
