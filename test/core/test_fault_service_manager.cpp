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
#include "ros2_fault_injection/core/fault_service_manager.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"

namespace ros2_fault_injection {
namespace {

using namespace std::chrono_literals;

class FakeFaultInjector : public FaultInjector {
public:
  explicit FakeFaultInjector(std::string id) : id_(std::move(id)) {}

  std::string id() const override {
    return id_;
  }

  void add_fault(const FaultConfig& fault_config) override {
    faults_[fault_config.id] = fault_config;
    active_.erase(fault_config.id);
  }

  void activate_fault(const std::string& fault_id) override {
    if (has_fault(fault_id)) {
      active_.insert(fault_id);
    }
  }

  void deactivate_fault(const std::string& fault_id) override {
    active_.erase(fault_id);
  }

  bool has_fault(const std::string& fault_id) const override {
    return faults_.find(fault_id) != faults_.end();
  }

  std::optional<FaultConfig> get_fault_config(const std::string& fault_id) const override {
    const auto it = faults_.find(fault_id);
    if (it == faults_.end()) {
      return std::nullopt;
    }

    return it->second;
  }

  bool set_fault_config_value(const std::string& fault_id, const std::string& key,
                              const std::string& value) override {
    const auto it = faults_.find(fault_id);
    if (it == faults_.end()) {
      return false;
    }

    it->second.config[key] = value;
    return true;
  }

  std::vector<std::string> fault_ids() const override {
    std::vector<std::string> ids;
    for (const auto& [fault_id, _] : faults_) {
      ids.push_back(fault_id);
    }
    return ids;
  }

  std::vector<std::string> active_fault_ids() const override {
    return std::vector<std::string>(active_.begin(), active_.end());
  }

  bool is_active(const std::string& fault_id) const {
    return active_.find(fault_id) != active_.end();
  }

private:
  std::string id_;
  std::unordered_map<std::string, FaultConfig> faults_;
  std::unordered_set<std::string> active_;
};

FaultConfig make_fault() {
  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";
  return fault;
}

void spin_for(const rclcpp::Node::SharedPtr& node, std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
}

std::optional<msg::FaultEvent> call_set_fault_state_and_wait_for_event(
    const rclcpp::Node::SharedPtr& node, bool active,
    const rclcpp::Client<srv::SetFaultState>::SharedPtr& client,
    const std::shared_ptr<msg::FaultEvent>& latest_event) {
  auto request = std::make_shared<srv::SetFaultState::Request>();
  request->fault_id = "odom_bias";
  request->active = active;

  auto future = client->async_send_request(request);
  const auto result =
      rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

  if (result != rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
    return std::nullopt;
  }

  spin_for(node, 100ms);

  if (latest_event->fault_id.empty()) {
    return std::nullopt;
  }

  return *latest_event;
}

}  // namespace

TEST(FaultServiceManager, SetFaultStatePublishesActiveEvent) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_active");
  auto injector = std::make_shared<FakeFaultInjector>("odom");
  injector->add_fault(make_fault());

  FaultServiceManager::InjectorMap injectors;
  injectors["odom"] = injector;

  FaultEventPublisher event_publisher(*node);
  FaultServiceManager services(*node, injectors, event_publisher);

  auto latest_event = std::make_shared<msg::FaultEvent>();
  auto subscription = node->create_subscription<msg::FaultEvent>(
      "fault_injection/events", 10,
      [latest_event](const msg::FaultEvent& event) { *latest_event = event; });

  auto client = node->create_client<srv::SetFaultState>("fault_injection/set_fault_state");
  ASSERT_TRUE(client->wait_for_service(500ms));

  const auto event = call_set_fault_state_and_wait_for_event(node, true, client, latest_event);

  ASSERT_TRUE(event.has_value());
  EXPECT_TRUE(injector->is_active("odom_bias"));
  EXPECT_EQ(event->fault_id, "odom_bias");
  EXPECT_EQ(event->injector_id, "odom");
  EXPECT_EQ(event->state, "active");
  EXPECT_EQ(event->source, "manual");
  EXPECT_NE(event->details.find("x_bias=1.0"), std::string::npos);

  (void)subscription;
  rclcpp::shutdown();
}

TEST(FaultServiceManager, SetFaultStatePublishesInactiveEvent) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_inactive");
  auto injector = std::make_shared<FakeFaultInjector>("odom");
  injector->add_fault(make_fault());
  injector->activate_fault("odom_bias");

  FaultServiceManager::InjectorMap injectors;
  injectors["odom"] = injector;

  FaultEventPublisher event_publisher(*node);
  FaultServiceManager services(*node, injectors, event_publisher);

  auto latest_event = std::make_shared<msg::FaultEvent>();
  auto subscription = node->create_subscription<msg::FaultEvent>(
      "fault_injection/events", 10,
      [latest_event](const msg::FaultEvent& event) { *latest_event = event; });

  auto client = node->create_client<srv::SetFaultState>("fault_injection/set_fault_state");
  ASSERT_TRUE(client->wait_for_service(500ms));

  const auto event = call_set_fault_state_and_wait_for_event(node, false, client, latest_event);

  ASSERT_TRUE(event.has_value());
  EXPECT_FALSE(injector->is_active("odom_bias"));
  EXPECT_EQ(event->fault_id, "odom_bias");
  EXPECT_EQ(event->injector_id, "odom");
  EXPECT_EQ(event->state, "inactive");
  EXPECT_EQ(event->source, "manual");
  EXPECT_NE(event->details.find("x_bias=1.0"), std::string::npos);

  (void)subscription;
  rclcpp::shutdown();
}

TEST(FaultServiceManager, SetFaultConfigPublishesConfigUpdatedEvent) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_config_update");
  auto injector = std::make_shared<FakeFaultInjector>("odom");
  injector->add_fault(make_fault());

  FaultServiceManager::InjectorMap injectors;
  injectors["odom"] = injector;

  FaultEventPublisher event_publisher(*node);
  FaultServiceManager services(*node, injectors, event_publisher);

  auto latest_event = std::make_shared<msg::FaultEvent>();
  auto subscription = node->create_subscription<msg::FaultEvent>(
      "fault_injection/events", 10,
      [latest_event](const msg::FaultEvent& event) { *latest_event = event; });

  auto client = node->create_client<srv::SetFaultConfig>("fault_injection/set_fault_config");
  ASSERT_TRUE(client->wait_for_service(500ms));

  auto request = std::make_shared<srv::SetFaultConfig::Request>();
  request->fault_id = "odom_bias";
  request->key = "x_bias";
  request->value = "2.0";

  auto future = client->async_send_request(request);
  const auto result =
      rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);
  ASSERT_TRUE(injector->has_fault("odom_bias"));
  EXPECT_EQ(injector->get_fault_config("odom_bias")->config["x_bias"], "2.0");

  spin_for(node, 100ms);

  ASSERT_FALSE(latest_event->fault_id.empty());
  EXPECT_EQ(latest_event->fault_id, "odom_bias");
  EXPECT_EQ(latest_event->injector_id, "odom");
  EXPECT_EQ(latest_event->state, "config_updated");
  EXPECT_EQ(latest_event->source, "manual");
  EXPECT_NE(latest_event->details.find("config_update={x_bias=2.0}"), std::string::npos);

  (void)subscription;
  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection
