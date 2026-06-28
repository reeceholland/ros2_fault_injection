// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/config/scenario_validator.hpp"
#include "ros2_fault_injection/core/fault_controller.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_scenario_runner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fault_scenario_runner");

  node->declare_parameter<std::string>("scenario_file", "");
  node->declare_parameter<double>("timeout", 30.0);
  node->declare_parameter<std::string>("report_file", "");

  const auto scenario_file = node->get_parameter("scenario_file").as_string();
  const auto timeout_seconds = node->get_parameter("timeout").as_double();
  const auto report_file = node->get_parameter("report_file").as_string();

  if (scenario_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter: scenario_file");
    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::ScenarioConfig scenario;

  try {
    scenario = ros2_fault_injection::load_scenario_config(scenario_file);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load scenario config '%s': %s",
                 scenario_file.c_str(), error.what());
    rclcpp::shutdown();
    return 1;
  }

  const auto validation_result = ros2_fault_injection::validate_scenario(scenario);

  for (const auto & warning : validation_result.warnings) {
    RCLCPP_WARN(node->get_logger(), "Scenario config warning: %s", warning.c_str());
  }

  if (!validation_result.ok()) {
    for (const auto & error : validation_result.errors) {
      RCLCPP_ERROR(node->get_logger(), "Scenario config error: %s", error.c_str());
    }

    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::FaultEventPublisher event_publisher(*node);
  ros2_fault_injection::FaultController controller(
    *node, scenario_file, scenario, event_publisher);

  ros2_fault_injection::core::FaultScenarioRunner runner(
    *node, controller,
    std::chrono::milliseconds{static_cast<std::int64_t>(timeout_seconds * 1000.0)});

  const auto result = runner.run();

  if (!report_file.empty()) {
    std::ofstream output(report_file);
    output << result.report_markdown;
  }

  if (result.success) {
    RCLCPP_INFO(node->get_logger(), "%s", result.message.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", result.message.c_str());
  }

  rclcpp::shutdown();
  return result.success ? 0 : 1;
}
