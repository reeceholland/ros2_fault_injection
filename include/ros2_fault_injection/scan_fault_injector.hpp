#ifndef ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_

#include <deque>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ros2_fault_injection/fault_injector_base.hpp"

namespace ros2_fault_injection {

class ScanFaultInjector : public FaultInjectorBase {
public:
  explicit ScanFaultInjector(rclcpp::Node& node, const InjectorConfig& config);

private:
  struct DelayedScan {
    sensor_msgs::msg::LaserScan msg;
    rclcpp::Time release_time;
  };

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void flush_delayed();

  void apply_range_bias(sensor_msgs::msg::LaserScan& msg);
  void apply_range_noise(sensor_msgs::msg::LaserScan& msg);
  void apply_sector_dropout(sensor_msgs::msg::LaserScan& msg);
  void warn_unknown_config_keys(const FaultConfig& fault_config) const override;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedScan> delayed_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_
