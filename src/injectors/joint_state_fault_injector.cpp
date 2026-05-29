// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/injectors/joint_state_fault_injector.hpp"

namespace ros2_fault_injection
{
JointStateFaultInjector::JointStateFaultInjector(rclcpp::Node & node, const InjectorConfig & config)
: FaultInjectorBase(node, config)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic->qos_depth));

  pub_ = node_.create_publisher<sensor_msgs::msg::JointState>(config_.topic->output_topic, qos);

  sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
        config_.topic->input_topic, qos,
    [this](sensor_msgs::msg::JointState::SharedPtr msg)
    {on_joint_state(msg);});

  timer_ = node_.create_wall_timer(std::chrono::milliseconds{10}, [this]()
      {flush_delayed();});
}

void JointStateFaultInjector::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
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

void JointStateFaultInjector::flush_delayed()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = node_.now();

  while (!delayed_.empty() && delayed_.front().release_time <= now) {
    pub_->publish(delayed_.front().msg);
    delayed_.pop_front();
  }
}

void JointStateFaultInjector::apply_bias(sensor_msgs::msg::JointState & msg)
{
  const double bias = active_sum_double("velocity_bias", 0.0);

  if (bias == 0.0) {
    return;
  }

  for (auto & velocity : msg.velocity) {
    velocity += bias;
  }
}

void JointStateFaultInjector::apply_noise(sensor_msgs::msg::JointState & msg)
{
  const double noise_stddev = active_max_double("velocity_noise_stddev", 0.0);

  if (noise_stddev == 0.0) {
    return;
  }

  std::normal_distribution<double> distribution(0.0, noise_stddev);

  for (auto & velocity : msg.velocity) {
    velocity += distribution(rng_);
  }
}

std::vector<FaultConfigField> JointStateFaultInjector::static_config_schema()
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
  add_field("velocity_bias", "double", "Additive bias applied to every joint velocity.",
      std::nullopt, std::nullopt, "0.0");
  add_field("velocity_noise_stddev", "double",
      "Standard deviation of Gaussian noise applied to joint velocities.", 0.0, std::nullopt,
      "0.0");

  return schema;
}

std::vector<FaultConfigField> JointStateFaultInjector::config_schema() const
{
  return static_config_schema();
}

} // namespace ros2_fault_injection
