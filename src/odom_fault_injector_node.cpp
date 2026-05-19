#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/odom_fault_injector.hpp"
#include "ros2_fault_injection/scenario_config.hpp"
#include "ros2_fault_injection/fault_scheduler.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // This node is the first framework runner: it loads a scenario, creates the
  // odom injector, registers faults with it, and schedules activation windows.
  auto node = std::make_shared<rclcpp::Node>("odom_fault_injector");

  node->declare_parameter<std::string>("scenario_file", "");

  const auto scenario_file =
      node->get_parameter("scenario_file").as_string();

  if (scenario_file.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter: scenario_file");
    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::ScenarioConfig scenario;

  try
  {
    // Keep file parsing separate from ROS wiring so the parser stays easy to
    // test and the injector does not need to know YAML exists.
    scenario = ros2_fault_injection::load_scenario_config(scenario_file);
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to load scenario config %s : %s",
        scenario_file.c_str(),
        error.what());

    rclcpp::shutdown();
    return 1;
  }

  auto injector =
      std::make_shared<ros2_fault_injection::OdomFaultInjector>(
          *node,
          scenario.injector);

  // Register every fault with the injector first. Activation is a separate
  // step so immediate and scheduled faults use the same injector interface.
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

  ros2_fault_injection::FaultScheduler scheduler(*node);
  scheduler.schedule(scenario.faults, *injector, scenario.initially_active_faults);

  RCLCPP_INFO(
      node->get_logger(),
      "Odom fault injector running: %s -> %s using scenario file %s",
      scenario.injector.input_topic.c_str(),
      scenario.injector.output_topic.c_str(),
      scenario_file.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
