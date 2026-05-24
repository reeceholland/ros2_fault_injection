// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_scheduler.hpp"

namespace ros2_fault_injection
{
namespace
{

using namespace std::chrono_literals;

class FakeFaultInjector : public FaultInjector {
public:
  std::string id() const override
  {
    return "fake_injector";
  }

  void add_fault(const FaultConfig & fault_config) override
  {
    faults_[fault_config.id] = fault_config;
    active_.erase(fault_config.id);
  }

  void activate_fault(const std::string & fault_id) override
  {
    if (has_fault(fault_id)) {
      active_.insert(fault_id);
    }
  }

  void deactivate_fault(const std::string & fault_id) override
  {
    active_.erase(fault_id);
  }

  bool has_fault(const std::string & fault_id) const override
  {
    return faults_.find(fault_id) != faults_.end();
  }

  std::optional<FaultConfig> get_fault_config(const std::string & fault_id) const override
  {
    const auto it = faults_.find(fault_id);
    if (it != faults_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<std::string> fault_ids() const override
  {
    std::vector<std::string> ids;
    for (const auto & [id, _] : faults_) {
      ids.push_back(id);
    }
    return ids;
  }

  std::vector<std::string> active_fault_ids() const override
  {
    return std::vector<std::string>(active_.begin(), active_.end());
  }

  bool is_active(const std::string & fault_id) const
  {
    return active_.find(fault_id) != active_.end();
  }

  bool set_fault_config_value(
    const std::string & fault_id, const std::string & key,
    const std::string & value) override
  {
    const auto it = faults_.find(fault_id);
    if (it == faults_.end()) {
      return false;
    }

    it->second.config[key] = value;
    return true;
  }

private:
  std::unordered_map<std::string, FaultConfig> faults_;
  std::unordered_set<std::string> active_;
};

void spin_for(const rclcpp::Node::SharedPtr & node, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
}

FaultConfig make_fault(const std::string & id)
{
  FaultConfig fault;
  fault.id = id;
  fault.injector_id = "fake_injector";
  return fault;
}

}  // namespace

TEST(FaultScheduler, StartupActiveFaultStopsAfterDuration) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_scheduler");
  FaultEventPublisher events(*node);
  FaultScheduler scheduler(*node, events);

  FakeFaultInjector injector;

  auto fault = make_fault("startup_fault");
  fault.duration = 50ms;
  injector.add_fault(fault);

  scheduler.schedule({fault}, injector, {"startup_fault"});

  EXPECT_TRUE(injector.is_active("startup_fault"));

  spin_for(node, 100ms);

  EXPECT_FALSE(injector.is_active("startup_fault"));

  rclcpp::shutdown();
}

TEST(FaultScheduler, ScheduledFaultStartsAndStops) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_scheduler");
  FaultEventPublisher events(*node);
  FaultScheduler scheduler(*node, events);

  FakeFaultInjector injector;

  auto fault = make_fault("scheduled_fault");
  fault.start = 50ms;
  fault.duration = 50ms;
  injector.add_fault(fault);

  scheduler.schedule({fault}, injector, {});

  EXPECT_FALSE(injector.is_active("scheduled_fault"));

  spin_for(node, 75ms);

  EXPECT_TRUE(injector.is_active("scheduled_fault"));

  spin_for(node, 75ms);

  EXPECT_FALSE(injector.is_active("scheduled_fault"));

  rclcpp::shutdown();
}

TEST(FaultScheduler, ManualOnlyTestStaysInactive) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_scheduler");
  FaultEventPublisher events(*node);
  FaultScheduler scheduler(*node, events);

  FakeFaultInjector injector;
  auto fault = make_fault("manual_only_fault");

  injector.add_fault(fault);

  scheduler.schedule({fault}, injector, {});

  spin_for(node, 50ms);

  EXPECT_FALSE(injector.is_active("manual_only_fault"));

  rclcpp::shutdown();
}
}  // namespace ros2_fault_injection
