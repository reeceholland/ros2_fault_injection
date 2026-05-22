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

/**
 * @brief Fault injector for `sensor_msgs/msg/LaserScan` streams.
 *
 * Subscribes to the configured input topic, applies active laser scan faults,
 * and republishes the result on the configured output topic.
 */
class ScanFaultInjector : public FaultInjectorBase {
public:
  /**
   * @brief Create the laser scan fault injector.
   *
   * @param node Node used to create publishers, subscriptions, and timers.
   * @param config Injector topic and QoS configuration.
   */
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

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<DelayedScan> delayed_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_
