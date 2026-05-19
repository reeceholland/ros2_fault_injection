#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/odom_fault_injector.hpp"
#include "ros2_fault_injection/scenario_config.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odom_fault_injector");

  node->declare_parameter<std::string>("config_file", "");

  const auto config_file =
      node->get_parameter("config_file").as_string();

  if (config_file.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter: config_file");
    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::ScenarioConfig scenario;

  try
  {
    scenario = ros2_fault_injection::load_scenario_config(config_file);
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to load scenario config %s : %s",
        config_file.c_str(),
        error.what());

    rclcpp::shutdown();
    return 1;
  }

  auto injector =
      std::make_shared<ros2_fault_injection::OdomFaultInjector>(
          *node,
          scenario.injector);

  for (const auto &fault : scenario.faults)
  {
    if (fault.injector_id != scenario.injector.id)
    {
      RCLCPP_WARN(
          node->get_logger(),
          "Fault '%s' has injector_id '%s' which does not match the scenario injector id '%s', skipping",
          fault.id.c_str(),
          fault.injector_id.c_str(),
          scenario.injector.id.c_str());
      continue;
    }
    injector->add_fault(fault);
  }

  for (const auto &fault_id : scenario.initially_active_faults)
  {
    injector->activate_fault(fault_id);
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Odom fault injector running: %s -> %s using config %s",
      scenario.injector.input_topic.c_str(),
      scenario.injector.output_topic.c_str(),
      config_file.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
