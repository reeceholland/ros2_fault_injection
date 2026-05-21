#include "ros2_fault_injection/joint_state_fault_injector.hpp"

namespace ros2_fault_injection {
JointStateFaultInjector::JointStateFaultInjector(rclcpp::Node& node, const InjectorConfig& config)
    : FaultInjectorBase(node, config) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.qos_depth));

  pub_ = node_.create_publisher<sensor_msgs::msg::JointState>(config_.output_topic, qos);

  sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
      config_.input_topic, qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });

  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]() { flush_delayed(); });
}

void JointStateFaultInjector::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  auto out = *msg;

  apply_bias(out);
  apply_noise(out);

  const auto delay = active_delay();
  if (delay.count() > 0) {
    delayed_.push_back(DelayedJointState{out, node_.now() + rclcpp::Duration(delay)});
    return;
  }

  pub_->publish(out);
}

void JointStateFaultInjector::flush_delayed() {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void JointStateFaultInjector::apply_bias(sensor_msgs::msg::JointState& msg) {
  const double bias = active_sum_double("velocity_bias", 0.0);

  if (bias == 0.0) {
    return;
  }

  for (auto& velocity : msg.velocity) {
    velocity += bias;
  }
}

void JointStateFaultInjector::apply_noise(sensor_msgs::msg::JointState& msg) {
  const double noise_stddev = active_max_double("velocity_noise_stddev", 0.0);

  if (noise_stddev == 0.0) {
    return;
  }

  std::normal_distribution<double> distribution(0.0, noise_stddev);

  for (auto& velocity : msg.velocity) {
    velocity += distribution(rng_);
  }
}

}  // namespace ros2_fault_injection