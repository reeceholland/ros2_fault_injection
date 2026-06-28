// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/core/fault_controller.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"

#include <fstream>
#include <memory>
#include <string>

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

namespace rfi_core = ros2_fault_injection::core;
namespace rfi_config = ros2_fault_injection::config;
namespace rfi_msg = ros2_fault_injection::msg;
namespace rfi_srv = ros2_fault_injection::srv;


namespace
{

class FaultControllerTest : public ::testing::Test
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

std::string write_temp_yaml(const std::string & filename, const std::string & content)
{
  const auto path = "/tmp/" + filename;
  std::ofstream file(path);
  file << content;
  return path;
}

std::string odom_scenario_yaml(
  const std::string & x_bias,
  const std::string & drop_probability = "0.5")
{
  return
    R"(
injector:
  id: odom
  type: odom
  input_topic: /test/odom_raw
  output_topic: /test/odom
  qos_depth: 10

faults:
  - id: odom_drop
    injector_id: odom
    active_on_startup: false
    config:
      drop_probability: )"
    +
    drop_probability +
    R"(
  - id: odom_bias
    injector_id: odom
    active_on_startup: false
    config:
      x_bias: )"
    +
    x_bias + R"(
)";
}

std::string incompatible_odom_scenario_yaml()
{
  return
    R"(
injector:
  id: odom_new
  type: odom
  input_topic: /test/odom_raw
  output_topic: /test/odom
  qos_depth: 10

faults:
  - id: odom_drop
    injector_id: odom_new
    active_on_startup: false
    config:
      drop_probability: 0.5
  - id: odom_bias
    injector_id: odom_new
    active_on_startup: false
    config:
      x_bias: 2.0
)";
}

}  // namespace

TEST_F(FaultControllerTest, ValidReloadUpdatesFaultConfig) {
  const auto path =
    write_temp_yaml("ros2_fault_injection_reload_valid.yaml", odom_scenario_yaml("1.0"));

  auto node = std::make_shared<rclcpp::Node>("test_fault_controller_reload_valid");
  rfi_core::FaultEventPublisher event_publisher(*node);

  auto scenario = rfi_config::load_scenario_config(path);
  rfi_core::FaultController controller(*node, path, scenario, event_publisher);

  auto injector = controller.injectors().at("odom");
  ASSERT_TRUE(injector->has_fault("odom_bias"));

  auto initial_fault = injector->get_fault_config("odom_bias");
  ASSERT_TRUE(initial_fault.has_value());
  EXPECT_EQ(initial_fault->config.at("x_bias"), "1.0");

  write_temp_yaml("ros2_fault_injection_reload_valid.yaml", odom_scenario_yaml("2.0"));

  const auto reload_result = controller.reload_scenario();
  EXPECT_TRUE(reload_result.success) << reload_result.message;

  auto updated_fault = injector->get_fault_config("odom_bias");
  ASSERT_TRUE(updated_fault.has_value());
  EXPECT_EQ(updated_fault->config.at("x_bias"), "2.0");
}

TEST_F(FaultControllerTest, InvalidReloadDoesNotChangeFaultConfig) {
  const auto path =
    write_temp_yaml("ros2_fault_injection_reload_invalid.yaml", odom_scenario_yaml("1.0"));

  auto node = std::make_shared<rclcpp::Node>("test_fault_controller_reload_invalid");
  rfi_core::FaultEventPublisher event_publisher(*node);

  auto scenario = rfi_config::load_scenario_config(path);
  rfi_core::FaultController controller(*node, path, scenario, event_publisher);

  auto injector = controller.injectors().at("odom");
  ASSERT_TRUE(injector->has_fault("odom_drop"));
  ASSERT_TRUE(injector->has_fault("odom_bias"));

  write_temp_yaml(
    "ros2_fault_injection_reload_invalid.yaml",
    odom_scenario_yaml("2.0", "2.0"));

  const auto reload_result = controller.reload_scenario();
  EXPECT_FALSE(reload_result.success);

  auto unchanged_drop_fault = injector->get_fault_config("odom_drop");
  ASSERT_TRUE(unchanged_drop_fault.has_value());
  EXPECT_EQ(unchanged_drop_fault->config.at("drop_probability"), "0.5");

  auto unchanged_bias_fault = injector->get_fault_config("odom_bias");
  ASSERT_TRUE(unchanged_bias_fault.has_value());
  EXPECT_EQ(unchanged_bias_fault->config.at("x_bias"), "1.0");
}

TEST_F(FaultControllerTest, ReloadWithChangedInjectorsIsRejected) {
  const auto path = write_temp_yaml(
    "ros2_fault_injection_reload_changed_injectors.yaml", odom_scenario_yaml("1.0"));

  auto node = std::make_shared<rclcpp::Node>("test_fault_controller_reload_changed_injectors");
  rfi_core::FaultEventPublisher event_publisher(*node);

  auto scenario = rfi_config::load_scenario_config(path);
  rfi_core::FaultController controller(*node, path, scenario, event_publisher);

  write_temp_yaml(
    "ros2_fault_injection_reload_changed_injectors.yaml",
    incompatible_odom_scenario_yaml());

  const auto reload_result = controller.reload_scenario();
  EXPECT_FALSE(reload_result.success);

  const auto injector = controller.injectors().at("odom");
  auto original_drop_fault = injector->get_fault_config("odom_drop");
  ASSERT_TRUE(original_drop_fault.has_value());
  EXPECT_EQ(original_drop_fault->config.at("drop_probability"), "0.5");

  auto original_bias_fault = injector->get_fault_config("odom_bias");
  ASSERT_TRUE(original_bias_fault.has_value());
  EXPECT_EQ(original_bias_fault->config.at("x_bias"), "1.0");
}
