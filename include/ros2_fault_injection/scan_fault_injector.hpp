#ifndef ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ros2_fault_injection/fault_injector.hpp"

namespace ros2_fault_injection
{

  class ScanFaultInjector : public FaultInjector
  {
  public:
    explicit ScanFaultInjector(rclcpp::Node &node, const InjectorConfig &config);

    std::string id() const override;
    void add_fault(const FaultConfig &fault_config) override;
    void activate_fault(const std::string &fault_id) override;
    void deactivate_fault(const std::string &fault_id) override;
    bool has_fault(const std::string &fault_id) const override;
    std::vector<std::string> fault_ids() const override;
    std::vector<std::string> active_fault_ids() const override;

  private:
    struct DelayedScan
    {
      sensor_msgs::msg::LaserScan msg;
      rclcpp::Time release_time;
    };
    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void flush_delayed();

    bool should_drop();
    void apply_range_bias(sensor_msgs::msg::LaserScan &msg);
    void apply_range_noise(sensor_msgs::msg::LaserScan &msg);
    std::chrono::milliseconds active_delay();
    void warn_unknown_config_keys(const FaultConfig &fault_config) const;

    rclcpp::Node &node_;
    InjectorConfig config_;
    std::mt19937 rng_;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, FaultConfig> faults_;
    std::unordered_map<std::string, bool> active_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<DelayedScan> delayed_;
  };

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__SCAN_FAULT_INJECTOR_HPP_
