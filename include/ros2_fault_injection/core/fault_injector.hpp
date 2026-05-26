// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/config/fault_config_schema.hpp"

namespace ros2_fault_injection
{

  /**
   * @brief Common interface implemented by all typed fault injectors.
   *
   * The interface keeps scheduling and services independent of message type. A
   * typed injector owns the ROS subscription/publisher pair for a specific
   * message type and applies the active faults during message callbacks.
   */
class FaultInjector
{
public:
  virtual ~FaultInjector() = default;

    /**
     * @brief Get the injector id from the scenario.
     *
     * @return Injector identifier.
     */
  virtual std::string id() const = 0;

    /**
     * @brief Get the injector type from the scenario.
     *
     * The type is used for schema validation and plugin selection. It may
     * differ from the injector id; for example, injector id `motor_feedback`
     * can have type `joint_state`.
     *
     * @return Injector type string.
     */
  virtual std::string type() const = 0;

  virtual std::vector<FaultConfigField> config_schema() const = 0;

    /**
     * @brief Register or replace a fault owned by this injector.
     *
     * @param fault_config Fault configuration to store.
     */
  virtual void add_fault(const FaultConfig & fault_config) = 0;

    /**
     * @brief Mark a known fault active.
     *
     * @param fault_id Fault identifier to activate.
     */
  virtual void activate_fault(const std::string & fault_id) = 0;

    /**
     * @brief Mark a fault inactive.
     *
     * @param fault_id Fault identifier to deactivate.
     */
  virtual void deactivate_fault(const std::string & fault_id) = 0;

    /**
     * @brief Check whether this injector owns a fault id.
     *
     * @param fault_id Fault identifier to look up.
     * @return true when the fault is registered.
     */
  virtual bool has_fault(const std::string & fault_id) const = 0;

    /**
     * @brief Get the stored configuration for a fault.
     *
     * @param fault_id Fault identifier to look up.
     * @return Fault configuration when found, otherwise std::nullopt.
     */
  virtual std::optional<FaultConfig> get_fault_config(const std::string & fault_id) const = 0;

    /**
     * @brief Update one runtime config value for a registered fault.
     *
     * This updates entries inside `FaultConfig::config`; it does not mutate
     * top-level scheduling fields such as `start` or `duration`.
     *
     * @param fault_id Fault identifier to update.
     * @param key Config key to set.
     * @param value New value stored as text.
     * @return true when the fault exists and the value was stored.
     */
  virtual bool set_fault_config_value(
    const std::string & fault_id, const std::string & key,
    const std::string & value) = 0;

    /**
     * @brief List all fault ids registered with this injector.
     *
     * @return Fault identifiers.
     */
  virtual std::vector<std::string> fault_ids() const = 0;

    /**
     * @brief List currently active fault ids.
     *
     * @return Active fault identifiers.
     */
  virtual std::vector<std::string> active_fault_ids() const = 0;

  virtual void clear_faults() = 0;
};

  /// Map of injector id to injector instance.
using InjectorMap = std::unordered_map<std::string, std::shared_ptr<FaultInjector>>;
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_INJECTOR_HPP_
