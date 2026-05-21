#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_BASE_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_BASE_HPP_

#include <chrono>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_injector.hpp"

namespace ros2_fault_injection {

class FaultInjectorBase : public FaultInjector {
public:
  explicit FaultInjectorBase(rclcpp::Node& node, InjectorConfig config);

  std::string id() const override;
  void add_fault(const FaultConfig& fault_config) override;
  void activate_fault(const std::string& fault_id) override;
  void deactivate_fault(const std::string& fault_id) override;
  bool has_fault(const std::string& fault_id) const override;
  std::optional<FaultConfig> get_fault_config(const std::string& fault_id) const override;
  std::vector<std::string> fault_ids() const override;
  std::vector<std::string> active_fault_ids() const override;

protected:
  double active_max_double(const std::string& key, double fallback = 0.0) const;
  double active_sum_double(const std::string& key, double fallback = 0.0) const;
  int active_max_int(const std::string& key, int fallback = 0) const;
  bool should_drop();
  std::chrono::milliseconds active_delay() const;

  rclcpp::Node& node_;
  InjectorConfig config_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, FaultConfig> faults_;
  std::unordered_map<std::string, bool> active_;
  std::mt19937 rng_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_INJECTOR_BASE_HPP_
