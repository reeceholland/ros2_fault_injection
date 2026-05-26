// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_BASE_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_BASE_HPP_

#include <chrono>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/core/fault_injector.hpp"

namespace ros2_fault_injection
{

  /**
   * @brief Shared implementation for typed fault injectors.
   *
   * This base class stores fault configuration, active state, random number
   * generation, and common helpers for drop/delay/config aggregation. Derived
   * classes provide the message-specific ROS subscriptions and mutation logic.
   */
class FaultInjectorBase : public FaultInjector
{
public:
    /**
     * @brief Construct a base injector.
     *
     * @param node ROS node used by derived classes.
     * @param config Injector configuration from the scenario.
     */
  explicit FaultInjectorBase(rclcpp::Node & node, InjectorConfig config);

  std::string id() const override;
  std::string type() const override;
  void add_fault(const FaultConfig & fault_config) override;
  void activate_fault(const std::string & fault_id) override;
  void deactivate_fault(const std::string & fault_id) override;
  bool has_fault(const std::string & fault_id) const override;
  std::optional<FaultConfig> get_fault_config(const std::string & fault_id) const override;
  std::vector<std::string> fault_ids() const override;
  std::vector<std::string> active_fault_ids() const override;
  bool set_fault_config_value(
    const std::string & fault_id, const std::string & key,
    const std::string & value) override;
  void clear_faults() override;

protected:
    /**
     * @brief Get the largest active numeric config value for a key.
     *
     * @param key Config key to scan across active faults.
     * @param fallback Value returned when no active fault provides the key.
     * @return Largest parsed value or fallback.
     */
  double active_max_double(const std::string & key, double fallback = 0.0) const;

    /**
     * @brief Sum active numeric config values for a key.
     *
     * @param key Config key to scan across active faults.
     * @param fallback Value returned when no active fault provides the key.
     * @return Sum of parsed values or fallback.
     */
  double active_sum_double(const std::string & key, double fallback = 0.0) const;

    /**
     * @brief Get the largest active integer config value for a key.
     *
     * @param key Config key to scan across active faults.
     * @param fallback Value returned when no active fault provides the key.
     * @return Largest parsed value or fallback.
     */
  int active_max_int(const std::string & key, int fallback = 0) const;

  bool active_bool(const std::string & key, bool fallback = false) const;

  std::string active_string(const std::string & key, const std::string & fallback = "") const;

    /**
     * @brief Decide whether the current message should be dropped.
     *
     * @return true when an active `drop_probability` fault drops the message.
     */
  bool should_drop();

    /**
     * @brief Get the current active delay.
     *
     * @return Delay duration from active `delay_ms` faults.
     */
  std::chrono::milliseconds active_delay() const;

  rclcpp::Node & node_;
  InjectorConfig config_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, FaultConfig> faults_;
  std::unordered_map<std::string, bool> active_;
  std::mt19937 rng_;
};

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_INJECTOR_BASE_HPP_
