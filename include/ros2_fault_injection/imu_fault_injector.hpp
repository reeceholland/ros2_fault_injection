#ifndef ROS2_FAULT_INJECTION__IMU_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__IMU_FAULT_INJECTOR_HPP_

#include <deque>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ros2_fault_injection/fault_injector_base.hpp"

namespace ros2_fault_injection {

class ImuFaultInjector : public FaultInjectorBase {
public:
  explicit ImuFaultInjector(rclcpp::Node& node, const InjectorConfig& config);

private:
  struct DelayedImu {
    sensor_msgs::msg::Imu msg;
    rclcpp::Time release_time;
  };

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void flush_delayed();

  void apply_angular_velocity_bias(sensor_msgs::msg::Imu& msg);
  void apply_angular_velocity_noise(sensor_msgs::msg::Imu& msg);
  void apply_linear_acceleration_bias(sensor_msgs::msg::Imu& msg);
  void apply_linear_acceleration_noise(sensor_msgs::msg::Imu& msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedImu> delayed_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__IMU_FAULT_INJECTOR_HPP_