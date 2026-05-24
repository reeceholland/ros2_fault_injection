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

namespace ros2_fault_injection
{
namespace
{
std::string write_temp_yaml(const std::string & filename, const std::string & content)
{
  const auto path = "/tmp/" + filename;
  std::ofstream file(path);
  file << content;
  file.close();
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
}   // namespace

TEST(FaultController, ValidReloadUpdatesFaultConfig)
  {
    rclcpp::init(0, nullptr);

    const auto path = write_temp_yaml("ros2_fault_injection_reload_valid.yaml",
                                      odom_scenario_yaml("1.0"));

    auto node = std::make_shared<rclcpp::Node>("test_fault_controller_reload_valid");
    FaultEventPublisher event_publisher(*node);

    auto scenario = load_scenario_config(path);
    FaultController controller(*node, path, scenario, event_publisher);

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

    rclcpp::shutdown();
}

TEST(FaultController, InvalidReloadDoesNotChangeFaultConfig)
  {
    rclcpp::init(0, nullptr);

    const auto path = write_temp_yaml("ros2_fault_injection_reload_invalid.yaml",
                                      odom_scenario_yaml("1.0"));

    auto node = std::make_shared<rclcpp::Node>("test_fault_controller_reload_invalid");
    FaultEventPublisher event_publisher(*node);

    auto scenario = load_scenario_config(path);
    FaultController controller(*node, path, scenario, event_publisher);

    auto injector = controller.injectors().at("odom");
    ASSERT_TRUE(injector->has_fault("odom_drop"));

    auto initial_fault = injector->get_fault_config("odom_drop");
    ASSERT_TRUE(initial_fault.has_value());
    EXPECT_EQ(initial_fault->config.at("drop_probability"), "0.5");

    write_temp_yaml("ros2_fault_injection_reload_invalid.yaml", odom_scenario_yaml("1.0", "2.0"));
    const auto reload_result = controller.reload_scenario();
    EXPECT_FALSE(reload_result.success);

    auto updated_fault = injector->get_fault_config("odom_drop");
    ASSERT_TRUE(updated_fault.has_value());
    EXPECT_EQ(updated_fault->config.at("drop_probability"), "0.5");

    rclcpp::shutdown();
}

TEST(FaultController, ReloadWithChangedInjectorsIsRejected)
  {
    rclcpp::init(0, nullptr);

    const auto path = write_temp_yaml("ros2_fault_injection_reload_changed_injectors.yaml",
                                      odom_scenario_yaml("1.0"));

    auto node = std::make_shared<rclcpp::Node>("test_fault_controller_reload_changed_injectors");
    FaultEventPublisher event_publisher(*node);

    auto scenario = load_scenario_config(path);
    FaultController controller(*node, path, scenario, event_publisher);

    write_temp_yaml("ros2_fault_injection_reload_changed_injectors.yaml",
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
      x_bias: 1.0
)");

    const auto reload_result = controller.reload_scenario();
    EXPECT_FALSE(reload_result.success);

    auto original_fault = controller.injectors().at("odom")->get_fault_config("odom_drop");
    ASSERT_TRUE(original_fault.has_value());
    EXPECT_EQ(original_fault->config.at("drop_probability"), "0.5");
    auto original_bias = controller.injectors().at("odom")->get_fault_config("odom_bias");
    ASSERT_TRUE(original_bias.has_value());
    EXPECT_EQ(original_bias->config.at("x_bias"), "1.0");

    rclcpp::shutdown();
}
} // namespace ros2_fault_injection
