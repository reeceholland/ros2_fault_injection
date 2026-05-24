// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/fault_injector_base.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>

namespace ros2_fault_injection
{
namespace
{

double get_double(const FaultConfig & fault, const std::string & key, double fallback)
{
  const auto it = fault.config.find(key);
  if (it == fault.config.end()) {
    return fallback;
  }

  return std::stod(it->second);
}

int get_int(const FaultConfig & fault, const std::string & key, int fallback)
{
  const auto it = fault.config.find(key);
  if (it == fault.config.end()) {
    return fallback;
  }

  return std::stoi(it->second);
}

}  // namespace

FaultInjectorBase::FaultInjectorBase(rclcpp::Node & node, InjectorConfig config)
: node_(node), config_(std::move(config)), rng_(std::random_device{}()) {}

std::string FaultInjectorBase::id() const
{
  return config_.id;
}

void FaultInjectorBase::add_fault(const FaultConfig & fault_config)
{
  std::lock_guard<std::mutex> lock(mutex_);

  faults_[fault_config.id] = fault_config;
  active_[fault_config.id] = false;
}

void FaultInjectorBase::activate_fault(const std::string & fault_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (faults_.find(fault_id) == faults_.end()) {
    RCLCPP_WARN(node_.get_logger(), "Cannot activate unknown fault '%s'", fault_id.c_str());
    return;
  }

  active_[fault_id] = true;
}

void FaultInjectorBase::deactivate_fault(const std::string & fault_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (faults_.find(fault_id) == faults_.end()) {
    RCLCPP_WARN(node_.get_logger(), "Cannot deactivate unknown fault '%s'", fault_id.c_str());
    return;
  }

  active_[fault_id] = false;
}

bool FaultInjectorBase::has_fault(const std::string & fault_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.find(fault_id) != faults_.end();
}

std::optional<FaultConfig> FaultInjectorBase::get_fault_config(const std::string & fault_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto it = faults_.find(fault_id);
  if (it == faults_.end()) {
    return std::nullopt;
  }

  return it->second;
}

bool FaultInjectorBase::set_fault_config_value(
  const std::string & fault_id, const std::string & key,
  const std::string & value)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_id);
  if (it == faults_.end()) {
    RCLCPP_WARN(node_.get_logger(), "Cannot set config for unknown fault '%s'", fault_id.c_str());
    return false;
  }

  it->second.config[key] = value;
  return true;
}

std::vector<std::string> FaultInjectorBase::fault_ids() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> ids;
  for (const auto & [id, _] : faults_) {
    ids.push_back(id);
  }
  return ids;
}

std::vector<std::string> FaultInjectorBase::active_fault_ids() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> ids;
  for (const auto & [id, is_active] : active_) {
    if (is_active) {
      ids.push_back(id);
    }
  }
  return ids;
}

double FaultInjectorBase::active_max_double(const std::string & key, double fallback) const
{
  double value = fallback;

  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    value = std::max(value, get_double(faults_.at(fault_id), key, fallback));
  }

  return value;
}

double FaultInjectorBase::active_sum_double(const std::string & key, double fallback) const
{
  double value = 0.0;

  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    value += get_double(faults_.at(fault_id), key, fallback);
  }

  return value;
}

int FaultInjectorBase::active_max_int(const std::string & key, int fallback) const
{
  int value = fallback;

  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    value = std::max(value, get_int(faults_.at(fault_id), key, fallback));
  }

  return value;
}

bool FaultInjectorBase::active_bool(const std::string & key, bool fallback) const
{
  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto it = faults_.at(fault_id).config.find(key);
    if (it != faults_.at(fault_id).config.end()) {
      return it->second == "true";
    }
  }

  return fallback;
}

std::string FaultInjectorBase::active_string(
  const std::string & key,
  const std::string & fallback) const
{
  for (const auto & [fault_id, is_active] : active_) {
    if (!is_active) {
      continue;
    }

    const auto it = faults_.at(fault_id).config.find(key);
    if (it != faults_.at(fault_id).config.end()) {
      return it->second;
    }
  }

  return fallback;
}

bool FaultInjectorBase::should_drop()
{
  double probability = active_max_double("drop_probability", 0.0);
  if (probability <= 0.0) {
    return false;
  }

  probability = std::clamp(probability, 0.0, 1.0);

  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(rng_) < probability;
}

std::chrono::milliseconds FaultInjectorBase::active_delay() const
{
  return std::chrono::milliseconds{active_max_int("delay_ms", 0)};
}

}  // namespace ros2_fault_injection
