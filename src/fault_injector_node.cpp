#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/core/fault_controller.hpp"

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"
#include "ros2_fault_injection/core/fault_service_manager.hpp"
#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/config/scenario_validator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // This node is the framework runner: it loads a scenario, creates every
  // configured injector, registers faults with the matching injector, and
  // schedules activation windows.
  auto node = std::make_shared<rclcpp::Node>("fault_injector");

  node->declare_parameter<std::string>("scenario_file", "");

  const auto scenario_file = node->get_parameter("scenario_file").as_string();

  if (scenario_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter: scenario_file");
    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::ScenarioConfig scenario;

  try {
    // Keep file parsing separate from ROS wiring so the parser stays easy to
    // test and injectors do not need to know YAML exists.
    scenario = ros2_fault_injection::load_scenario_config(scenario_file);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load scenario config %s : %s",
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

  ros2_fault_injection::FaultEventPublisher event_pub(*node);

  ros2_fault_injection::FaultController controller(*node, scenario, event_pub);

  auto fault_service_manager = std::make_shared<ros2_fault_injection::FaultServiceManager>(
      *node, controller.injectors(), event_pub);

  ros2_fault_injection::FaultScheduler scheduler(*node, event_pub);

  RCLCPP_INFO(node->get_logger(),
              "Fault injector running with %zu injectors using scenario file %s",
              controller.injectors().size(), scenario_file.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
