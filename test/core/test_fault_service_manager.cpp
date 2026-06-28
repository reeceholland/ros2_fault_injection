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
#include "ros2_fault_injection/core/fault_service_manager.hpp"
#include "ros2_fault_injection/config/fault_config_schema.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"
#include "ros2_fault_injection/srv/get_fault_config.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"

namespace ros2_fault_injection
{
namespace
{

using namespace std::chrono_literals;

FaultConfigField make_test_schema_field(const std::string & key)
{
  FaultConfigField field;
  field.key = key;

  if (key == "drop_probability") {
    field.type = "double";
    field.min_value = 0.0;
    field.max_value = 1.0;
    return field;
  }

  if (key == "delay_ms") {
    field.type = "int";
    field.min_value = 0.0;
    return field;
  }

  if (key == "force_failure") {
    field.type = "bool";
    return field;
  }

  if (key == "sector_value") {
    field.type = "special_float";
    return field;
  }

  if (key == "parent_frame" || key == "child_frame") {
    field.type = "non_empty_string";
    return field;
  }

  if (key.find("noise_stddev") != std::string::npos ||
    key.find("covariance") != std::string::npos)
  {
    field.type = "double";
    field.min_value = 0.0;
    return field;
  }

  field.type = "double";
  return field;
}

class FakeFaultInjector : public FaultInjector
{
public:
  explicit FakeFaultInjector(std::string id, std::string type = "odom")
  : id_(std::move(id)), type_(std::move(type))
  {
    for (const auto & key : allowed_config_keys_for_injector_type(type_)) {
      schema_.push_back(make_test_schema_field(key));
    }
  }

  std::string id() const override
  {
    return id_;
  }

  std::string type() const override
  {
    return type_;
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
    if (it == faults_.end()) {
      return std::nullopt;
    }

    return it->second;
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

  std::vector<std::string> fault_ids() const override
  {
    std::vector<std::string> ids;
    for (const auto &[fault_id, _] : faults_) {
      ids.push_back(fault_id);
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

  void clear_faults() override
  {
    faults_.clear();
    active_.clear();
  }

  std::vector<FaultConfigField> config_schema() const override
  {
    return schema_;
  }

  void add_schema_field(const FaultConfigField & field)
  {
    schema_.push_back(field);
  }

private:
  std::string id_;
  std::string type_;
  std::unordered_map<std::string, FaultConfig> faults_;
  std::unordered_set<std::string> active_;
  std::vector<FaultConfigField> schema_;
};

FaultConfig make_fault()
{
  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";
  return fault;
}

void spin_for(const rclcpp::Node::SharedPtr & node, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
}

std::optional<msg::FaultEvent> call_set_fault_state_and_wait_for_event(
  const rclcpp::Node::SharedPtr & node, bool active,
  const rclcpp::Client<srv::SetFaultState>::SharedPtr & client,
  const std::shared_ptr<msg::FaultEvent> & latest_event)
{
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

ReloadScenarioResult test_reload_callback()
{
  return ReloadScenarioResult{false, "test reload callback"};
}

std::string test_scenario_file_provider()
{
  return "/tmp/test_scenario.yaml";
}

std::optional<std::string> test_scenario_content_provider()
{
  return std::string{"injectors: []\n"};
}

FaultServiceManager::ReportResult test_request_report_callback()
{
  return FaultServiceManager::ReportResult{
    true, "Created scenario report", "/tmp/test_scenario.yaml", "passed",
    "# Fault Injection Scenario Report\n"};
}

}   // namespace

TEST(FaultServiceManager, SetFaultStatePublishesActiveEvent)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_active");
    auto injector = std::make_shared<FakeFaultInjector>("odom");
    injector->add_fault(make_fault());

    FaultServiceManager::InjectorMap injectors;
    injectors["odom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto latest_event = std::make_shared<msg::FaultEvent>();
    auto subscription = node->create_subscription<msg::FaultEvent>(
        "fault_injection/events", 10,
    [latest_event](const msg::FaultEvent & event)
    {*latest_event = event;});

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

TEST(FaultServiceManager, SetFaultStatePublishesInactiveEvent)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_inactive");
    auto injector = std::make_shared<FakeFaultInjector>("odom");
    injector->add_fault(make_fault());
    injector->activate_fault("odom_bias");

    FaultServiceManager::InjectorMap injectors;
    injectors["odom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto latest_event = std::make_shared<msg::FaultEvent>();
    auto subscription = node->create_subscription<msg::FaultEvent>(
        "fault_injection/events", 10,
    [latest_event](const msg::FaultEvent & event)
    {*latest_event = event;});

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

TEST(FaultServiceManager, SetFaultConfigPublishesConfigUpdatedEvent)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_config_update");
    auto injector = std::make_shared<FakeFaultInjector>("odom");
    injector->add_fault(make_fault());

    FaultServiceManager::InjectorMap injectors;
    injectors["odom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto latest_event = std::make_shared<msg::FaultEvent>();
    auto subscription = node->create_subscription<msg::FaultEvent>(
        "fault_injection/events", 10,
    [latest_event](const msg::FaultEvent & event)
    {*latest_event = event;});

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

TEST(FaultServiceManager, GetFaultConfigReturnsCurrentConfig)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_get_config");
    auto injector = std::make_shared<FakeFaultInjector>("odom", "odom");

    FaultConfig fault;
    fault.id = "odom_bias";
    fault.injector_id = "odom";
    fault.config["x_bias"] = "1.0";
    fault.config["drop_probability"] = "0.25";
    injector->add_fault(fault);

    FaultServiceManager::InjectorMap injectors;
    injectors["odom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto client = node->create_client<srv::GetFaultConfig>("fault_injection/get_fault_config");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::GetFaultConfig::Request>();
    request->fault_id = "odom_bias";

    auto future = client->async_send_request(request);
    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);

    const auto response = future.get();
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->injector_id, "odom");
    EXPECT_EQ(response->injector_type, "odom");
    ASSERT_EQ(response->keys.size(), response->values.size());

    EXPECT_EQ(response->keys, (std::vector<std::string>{"drop_probability", "x_bias"}));
    EXPECT_EQ(response->values, (std::vector<std::string>{"0.25", "1.0"}));

    rclcpp::shutdown();
}

TEST(FaultServiceManager, SetFaultConfigRejectsInvalidValue)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_invalid_config_update");
    auto injector = std::make_shared<FakeFaultInjector>("imu", "imu");

    FaultConfig fault;
    fault.id = "imu_linear_acceleration_noise";
    fault.injector_id = "imu";
    fault.config["drop_probability"] = "0.5";
    injector->add_fault(fault);

    FaultServiceManager::InjectorMap injectors;
    injectors["imu"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto client = node->create_client<srv::SetFaultConfig>("fault_injection/set_fault_config");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::SetFaultConfig::Request>();
    request->fault_id = "imu_linear_acceleration_noise";
    request->key = "drop_probability";
    request->value = "Helloo";

    auto future = client->async_send_request(request);
    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(future.get()->success);

    const auto updated_fault = injector->get_fault_config("imu_linear_acceleration_noise");
    ASSERT_TRUE(updated_fault.has_value());
    EXPECT_EQ(updated_fault->config.at("drop_probability"), "0.5");

    rclcpp::shutdown();
}

TEST(FaultServiceManager, SetFaultConfigAllowsInjectorOwnedKeyUnknownToCentralSchema)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_plugin_config_update");
    auto injector = std::make_shared<FakeFaultInjector>("custom", "custom");

    auto fault = make_fault();
    fault.id = "custom_fault";
    fault.injector_id = "custom";
    injector->add_fault(fault);

    FaultConfigField field;
    field.key = "plugin_only_key";
    injector->add_schema_field(field);

    FaultServiceManager::InjectorMap injectors;
    injectors["custom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto client = node->create_client<srv::SetFaultConfig>("fault_injection/set_fault_config");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::SetFaultConfig::Request>();
    request->fault_id = "custom_fault";
    request->key = "plugin_only_key";
    request->value = "plugin_value";

    auto future = client->async_send_request(request);
    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);

    const auto updated_fault = injector->get_fault_config("custom_fault");
    ASSERT_TRUE(updated_fault.has_value());
    EXPECT_EQ(updated_fault->config.at("plugin_only_key"), "plugin_value");

    rclcpp::shutdown();
}

TEST(FaultServiceManager, SetFaultConfigValidatesAgainstInjectorType)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_config_update_type");
    auto injector = std::make_shared<FakeFaultInjector>("motor_feedback", "joint_state");

    FaultConfig fault;
    fault.id = "motor_velocity_bias";
    fault.injector_id = "motor_feedback";
    fault.config["velocity_bias"] = "0.1";
    injector->add_fault(fault);

    FaultServiceManager::InjectorMap injectors;
    injectors["motor_feedback"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto client = node->create_client<srv::SetFaultConfig>("fault_injection/set_fault_config");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::SetFaultConfig::Request>();
    request->fault_id = "motor_velocity_bias";
    request->key = "velocity_bias";
    request->value = "0.25";

    auto future = client->async_send_request(request);
    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);

    const auto updated_fault = injector->get_fault_config("motor_velocity_bias");
    ASSERT_TRUE(updated_fault.has_value());
    EXPECT_EQ(updated_fault->config.at("velocity_bias"), "0.25");

    rclcpp::shutdown();
}

TEST(FaultServiceManager, GetFaultSchemaUsesInjectorOwnedSchema)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_injector_schema");

    auto injector = std::make_shared<FakeFaultInjector>("custom");

    auto fault = make_fault();
    fault.id = "custom_fault";
    fault.injector_id = "custom";
    injector->add_fault(fault);

    FaultConfigField field;
    field.key = "plugin_only_key";
    injector->add_schema_field(field);

    FaultServiceManager::InjectorMap injectors;
    injectors["custom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;
    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    test_scenario_file_provider, test_scenario_content_provider, test_request_report_callback);

    auto client = node->create_client<srv::GetFaultSchema>("fault_injection/get_fault_schema");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::GetFaultSchema::Request>();
    request->fault_id = "custom_fault";

    auto future = client->async_send_request(request);
    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});
    auto response = future.get();
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(response->success);

    const auto & keys = response->keys;
    EXPECT_NE(std::find(keys.begin(), keys.end(), "plugin_only_key"), keys.end());

    rclcpp::shutdown();
}

TEST(FaultServiceManager, GetScenarioReturnsScenarioFileAndContent)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_get_scenario");
    auto injector = std::make_shared<FakeFaultInjector>("odom");

    FaultServiceManager::InjectorMap injectors;
    injectors["odom"] = injector;

    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;

    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder, test_reload_callback,
    []()
    {return "/tmp/test_scenario.yaml";},
    []()
    {return std::optional<std::string>{"injectors: []\n"};}, test_request_report_callback);

    auto client = node->create_client<srv::GetScenario>("fault_injection/get_scenario");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::GetScenario::Request>();
    auto future = client->async_send_request(request);

    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);

    const auto response = future.get();
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->scenario_file, "/tmp/test_scenario.yaml");
    EXPECT_EQ(response->content, "injectors: []\n");
    EXPECT_NE(response->message.find("Retrieved scenario"), std::string::npos);

    rclcpp::shutdown();
}

TEST(FaultServiceManager, GetScenarioFailsWhenContentUnavailable)
  {
    rclcpp::init(0, nullptr);

    auto node =
    std::make_shared<rclcpp::Node>("test_fault_service_manager_get_scenario_unavailable");
    FaultServiceManager::InjectorMap injectors;
    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;

    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder,
    test_reload_callback,
    []()
    {return "/tmp/missing_scenario.yaml";},
    []()
    {return std::nullopt;}, test_request_report_callback);

    auto client = node->create_client<srv::GetScenario>("/fault_injection/get_scenario");
    ASSERT_TRUE(client->wait_for_service(500ms));

    auto request = std::make_shared<srv::GetScenario::Request>();
    auto future = client->async_send_request(request);

    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);

    const auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->scenario_file, "/tmp/missing_scenario.yaml") << response->message;
    EXPECT_TRUE(response->content.empty());
    EXPECT_NE(response->message.find("failed"), std::string::npos);

    rclcpp::shutdown();
}

TEST(FaultServiceManager, GetScenarioUsesLatestProviderValue)
  {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_fault_service_manager_get_scenario_latest");
    FaultServiceManager::InjectorMap injectors;
    FaultEventPublisher event_publisher(*node);
    core::FaultEventRecorder event_recorder;

    std::string content = "version: one\n";

    FaultServiceManager services(
    *node, injectors, event_publisher, event_recorder,
    test_reload_callback,
    []()
    {return "/tmp/test_scenario.yaml";},
    [&content]()
    {return std::optional<std::string>{content};}, test_request_report_callback);

    auto client = node->create_client<srv::GetScenario>("/fault_injection/get_scenario");
    ASSERT_TRUE(client->wait_for_service(500ms));

    content = "version: two\n";

    auto request = std::make_shared<srv::GetScenario::Request>();
    auto future = client->async_send_request(request);

    const auto result =
    rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds{500});

    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);

    const auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->content, "version: two\n");

    rclcpp::shutdown();
}

} // namespace ros2_fault_injection
