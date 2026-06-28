// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__IMU_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__IMU_FAULT_INJECTOR_HPP_

#include <deque>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection::injectors
{

  /**
   * @brief Fault injector for `sensor_msgs/msg/Imu` streams.
   *
   * Subscribes to the configured input topic, applies active IMU faults,
   * and republishes the result on the configured output topic.
   */
class ImuFaultInjector : public FaultInjectorBase
{
public:
    /**
     * @brief Create the IMU fault injector.
     *
     * @param node Node used to create publishers, subscriptions, and timers.
     * @param config Injector topic and QoS configuration.
     */
  explicit ImuFaultInjector(rclcpp::Node & node, const InjectorConfig & config);

  static std::vector<FaultConfigField> static_config_schema();

  std::vector<FaultConfigField> config_schema() const override;

private:
  struct DelayedImu
  {
    sensor_msgs::msg::Imu msg;
    rclcpp::Time release_time;
  };

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void flush_delayed();

  void apply_angular_velocity_bias(sensor_msgs::msg::Imu & msg);
  void apply_angular_velocity_noise(sensor_msgs::msg::Imu & msg);
  void apply_linear_acceleration_bias(sensor_msgs::msg::Imu & msg);
  void apply_linear_acceleration_noise(sensor_msgs::msg::Imu & msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedImu> delayed_;
};

} // namespace ros2_fault_injection::injectors

namespace ros2_fault_injection
{
using injectors::ImuFaultInjector;
}  // namespace ros2_fault_injection
#endif // ROS2_FAULT_INJECTION__IMU_FAULT_INJECTOR_HPP_
