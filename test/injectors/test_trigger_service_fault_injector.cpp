// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_fault_injection/injectors/trigger_service_fault_injector.hpp"

namespace ros2_fault_injection::injectors
{
namespace
{

using namespace std::chrono_literals;

InjectorConfig make_injector_config(const std::string & suffix)
{
  InjectorConfig config;
  config.id = "trigger_service";
  config.type = "trigger_service";
  config.trigger_service = TriggerServiceEndpointConfig{
    "/test/trigger_proxy_" + suffix,
    "/test/trigger_target_" + suffix,
  };
  return config;
}

FaultConfig make_force_failure_fault()
{
  FaultConfig fault;
  fault.id = "trigger_failure";
  fault.injector_id = "trigger_service";
  fault.config["force_failure"] = "true";
  fault.config["failure_message"] = "Injected trigger failure";
  return fault;
}

FaultConfig make_delay_fault()
{
  FaultConfig fault;
  fault.id = "trigger_delay";
  fault.injector_id = "trigger_service";
  fault.config["delay_ms"] = "100";
  return fault;
}

bool wait_for_service(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (client->wait_for_service(10ms)) {
      return true;
    }
  }

  return false;
}

std_srvs::srv::Trigger::Response::SharedPtr call_trigger(
  const rclcpp::Node::SharedPtr & client_node,
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  const auto result = rclcpp::spin_until_future_complete(client_node, future, 1s);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    return nullptr;
  }

  return future.get();
}

class SpinningExecutor {
public:
  explicit SpinningExecutor(const rclcpp::Node::SharedPtr & node)
  : executor_(rclcpp::ExecutorOptions{}, 2)
  {
    executor_.add_node(node);
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~SpinningExecutor()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread thread_;
};

}  // namespace

TEST(TriggerServiceFaultInjector, ForwardsTargetResponseWhenNoFaultActive) {
  rclcpp::init(0, nullptr);

  auto proxy_node = std::make_shared<rclcpp::Node>("test_trigger_proxy_passthrough");
  auto target_node = std::make_shared<rclcpp::Node>("test_trigger_target_passthrough");
  auto client_node = std::make_shared<rclcpp::Node>("test_trigger_client_passthrough");

  std::atomic<int> target_calls{0};
  auto target_service = target_node->create_service<std_srvs::srv::Trigger>(
      "/test/trigger_target_passthrough",
    [&target_calls](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++target_calls;
      response->success = true;
      response->message = "target response";
      });

  SpinningExecutor target_spin(target_node);

  TriggerServiceFaultInjector injector(*proxy_node, make_injector_config("passthrough"));
  SpinningExecutor proxy_spin(proxy_node);

  auto client =
    client_node->create_client<std_srvs::srv::Trigger>("/test/trigger_proxy_passthrough");
  ASSERT_TRUE(wait_for_service(client, 500ms));

  const auto response = call_trigger(client_node, client);

  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "target response");
  EXPECT_EQ(target_calls.load(), 1);

  (void)target_service;
  rclcpp::shutdown();
}

TEST(TriggerServiceFaultInjector, ForceFailureReturnsInjectedFailureWithoutCallingTarget) {
  rclcpp::init(0, nullptr);

  auto proxy_node = std::make_shared<rclcpp::Node>("test_trigger_proxy_failure");
  auto target_node = std::make_shared<rclcpp::Node>("test_trigger_target_failure");
  auto client_node = std::make_shared<rclcpp::Node>("test_trigger_client_failure");

  std::atomic<int> target_calls{0};
  auto target_service = target_node->create_service<std_srvs::srv::Trigger>(
      "/test/trigger_target_failure",
    [&target_calls](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++target_calls;
      response->success = true;
      response->message = "target response";
      });

  SpinningExecutor target_spin(target_node);

  TriggerServiceFaultInjector injector(*proxy_node, make_injector_config("failure"));
  const auto fault = make_force_failure_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);
  SpinningExecutor proxy_spin(proxy_node);

  auto client = client_node->create_client<std_srvs::srv::Trigger>("/test/trigger_proxy_failure");
  ASSERT_TRUE(wait_for_service(client, 500ms));

  const auto response = call_trigger(client_node, client);

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Injected trigger failure");
  EXPECT_EQ(target_calls.load(), 0);

  (void)target_service;
  rclcpp::shutdown();
}

TEST(TriggerServiceFaultInjector, DelayFaultDelaysResponse) {
  rclcpp::init(0, nullptr);

  auto proxy_node = std::make_shared<rclcpp::Node>("test_trigger_proxy_delay");
  auto target_node = std::make_shared<rclcpp::Node>("test_trigger_target_delay");
  auto client_node = std::make_shared<rclcpp::Node>("test_trigger_client_delay");

  auto target_service = target_node->create_service<std_srvs::srv::Trigger>(
      "/test/trigger_target_delay",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
      response->message = "target response";
      });

  SpinningExecutor target_spin(target_node);

  TriggerServiceFaultInjector injector(*proxy_node, make_injector_config("delay"));
  const auto fault = make_delay_fault();
  injector.add_fault(fault);
  injector.activate_fault(fault.id);
  SpinningExecutor proxy_spin(proxy_node);

  auto client = client_node->create_client<std_srvs::srv::Trigger>("/test/trigger_proxy_delay");
  ASSERT_TRUE(wait_for_service(client, 500ms));

  const auto start = std::chrono::steady_clock::now();
  const auto response = call_trigger(client_node, client);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_GE(elapsed, 100ms);

  (void)target_service;
  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection::injectors
