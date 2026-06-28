// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <memory>
#include <string>

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

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/core/fault_injector_factory.hpp"

namespace rfi_core = ros2_fault_injection::core;
namespace rfi_config = ros2_fault_injection::config;
namespace rfi_msg = ros2_fault_injection::msg;
namespace rfi_srv = ros2_fault_injection::srv;


namespace
{

class FaultInjectorFactoryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

rfi_config::InjectorConfig make_topic_config(const std::string & type)
{
  rfi_config::InjectorConfig config;
  config.id = type + "_injector";
  config.type = type;

  rfi_config::TopicEndpointConfig topic;
  topic.input_topic = "/test/" + type + "_raw";
  topic.output_topic = "/test/" + type;
  topic.qos_depth = 10;

  config.topic = topic;
  return config;
}

rfi_config::InjectorConfig make_trigger_service_config()
{
  rfi_config::InjectorConfig config;
  config.id = "trigger_service_injector";
  config.type = "trigger_service";

  rfi_config::TriggerServiceEndpointConfig trigger_service;
  trigger_service.proxy_service = "/test/trigger_proxy";
  trigger_service.target_service = "/test/trigger_target";

  config.trigger_service = trigger_service;
  return config;
}

}   // namespace

TEST_F(FaultInjectorFactoryTest, CreatesBuiltInTopicInjectors)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_injector_factory_topic");
    rfi_core::FaultInjectorFactory factory(*node);

    for (const auto & type : {"odom", "scan", "joint_state", "imu", "tf"}) {
    auto injector = factory.create(make_topic_config(type));

    ASSERT_NE(injector, nullptr) << "Failed to create injector type: " << type;
    EXPECT_EQ(injector->id(), std::string(type) + "_injector");
    }
}

TEST_F(FaultInjectorFactoryTest, CreatesBuiltInTriggerServiceInjector)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_injector_factory_service");
    rfi_core::FaultInjectorFactory factory(*node);

    auto injector = factory.create(make_trigger_service_config());

    ASSERT_NE(injector, nullptr);
    EXPECT_EQ(injector->id(), "trigger_service_injector");
}

TEST_F(FaultInjectorFactoryTest, UnknownInjectorTypeReturnsNullptr)
  {
    auto node = std::make_shared<rclcpp::Node>("test_fault_injector_factory_unknown");
    rfi_core::FaultInjectorFactory factory(*node);

    auto config = make_topic_config("unknown_type");

    auto injector = factory.create(config);

    EXPECT_EQ(injector, nullptr);
}
