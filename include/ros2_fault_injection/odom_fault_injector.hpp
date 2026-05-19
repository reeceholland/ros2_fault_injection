#ifndef ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_

#include <deque>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/fault_injector.hpp"

namespace ros2_fault_injection
{
  class OdomFaultInjector : public FaultInjector
  {
  public:
    explicit OdomFaultInjector(rclcpp::Node &node, const InjectorConfig &config);

    std::string id() const override;
    void add_fault(const FaultConfig &fault_config) override;
    void activate_fault(const std::string &fault_id) override;
    void deactivate_fault(const std::string &fault_id) override;
    bool has_fault(const std::string &fault_id) const override;

  private:
    struct DelayedOdom
    {
      nav_msgs::msg::Odometry msg;
      rclcpp::Time release_time;
    };

    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void flush_delayed();

    bool should_drop();
    void apply_bias(nav_msgs::msg::Odometry &msg);
    void apply_noise(nav_msgs::msg::Odometry &msg);
    std::chrono::milliseconds active_delay();

    rclcpp::Node &node_;
    InjectorConfig config_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, FaultConfig> faults_;
    std::unordered_map<std::string, bool> active_;

    std::deque<DelayedOdom> delayed_;
    std::mt19937 rng_;
  };
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__ODOM_FAULT_INJECTOR_HPP_