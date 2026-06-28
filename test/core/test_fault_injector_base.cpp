// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/config/fault_config_schema.hpp"
#include "ros2_fault_injection/core/fault_controller.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_event_recorder.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_injector_base.hpp"
#include "ros2_fault_injection/core/fault_injector_factory.hpp"
#include "ros2_fault_injection/core/fault_scheduler.hpp"
#include "ros2_fault_injection/core/fault_service_manager.hpp"

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace rfi_core = ros2_fault_injection::core;
namespace rfi_config = ros2_fault_injection::config;
namespace rfi_msg = ros2_fault_injection::msg;
namespace rfi_srv = ros2_fault_injection::srv;



class TestFaultInjector : public rfi_core::FaultInjectorBase {
public:
  TestFaultInjector(rclcpp::Node & node, const rfi_config::InjectorConfig & config)
  : rfi_core::FaultInjectorBase(node, config) {}
};

TEST(FaultInjectorBase, ReturnsStoredFaultConfig) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  rfi_config::FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);

  const auto result = injector.get_fault_config("odom_bias");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "odom_bias");
  EXPECT_EQ(result->injector_id, "odom");
  EXPECT_EQ(result->config.at("x_bias"), "1.0");

  EXPECT_FALSE(injector.get_fault_config("missing_fault").has_value());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, StartsWithNoFaults) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  EXPECT_TRUE(injector.fault_ids().empty());
  EXPECT_TRUE(injector.active_fault_ids().empty());
  EXPECT_FALSE(injector.has_fault("missing"));

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, AddFaultRegistersItInactive) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  rfi_config::FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);

  EXPECT_TRUE(injector.has_fault("odom_bias"));
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, ActivateKnownFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  rfi_config::FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault("odom_bias");

  EXPECT_TRUE(injector.has_fault("odom_bias"));
  EXPECT_EQ(injector.active_fault_ids(), std::vector<std::string>{"odom_bias"});

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, DeactivateKnownFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  rfi_config::FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault("odom_bias");
  injector.deactivate_fault("odom_bias");

  EXPECT_TRUE(injector.has_fault("odom_bias"));
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, UnknownActivateDoesNotCreateFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  injector.activate_fault("unknown_fault");

  EXPECT_FALSE(injector.has_fault("unknown_fault"));
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, SameIdReplacesConfigAndResetsInactive) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  rfi_config::FaultConfig fault;
  fault.id = "fault1";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault("fault1");

  // Add a new fault with the same ID but different config
  rfi_config::FaultConfig new_fault;
  new_fault.id = "fault1";
  new_fault.injector_id = "odom";
  new_fault.config["x_bias"] = "2.0";

  injector.add_fault(new_fault);

  // The old config should be replaced and the fault should be inactive
  const auto result = injector.get_fault_config("fault1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->config.at("x_bias"), "2.0");
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, SetFaultConfigValueUpdatesExistingFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  rfi_config::FaultConfig fault;
  fault.id = "odom_yaw_bias";
  fault.injector_id = "odom";
  fault.config["yaw_bias_deg"] = "10.0";

  injector.add_fault(fault);

  const bool updated = injector.set_fault_config_value("odom_yaw_bias", "yaw_bias_deg", "20.0");

  EXPECT_TRUE(updated);

  const auto result = injector.get_fault_config("odom_yaw_bias");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->config.at("yaw_bias_deg"), "20.0");

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, SetFaultConfigValueRejectsUnknownFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  rfi_config::InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.topic = rfi_config::TopicEndpointConfig{
    "/odom_raw",
    "/odom",
    10,
  };

  TestFaultInjector injector(*node, injector_config);

  const bool updated = injector.set_fault_config_value("missing_fault", "yaw_bias_deg", "20.0");

  EXPECT_FALSE(updated);
  EXPECT_FALSE(injector.has_fault("missing_fault"));

  rclcpp::shutdown();
}

