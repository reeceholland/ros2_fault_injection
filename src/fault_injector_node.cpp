#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/fault_event_publisher.hpp"
#include "ros2_fault_injection/fault_scheduler.hpp"
#include "ros2_fault_injection/odom_fault_injector.hpp"
#include "ros2_fault_injection/scan_fault_injector.hpp"
#include "ros2_fault_injection/scenario_config.hpp"
#include "ros2_fault_injection/srv/list_faults.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // This node is the first framework runner: it loads a scenario, creates the
  // odom injector, registers faults with it, and schedules activation windows.
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
    // test and the injector does not need to know YAML exists.
    scenario = ros2_fault_injection::load_scenario_config(scenario_file);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load scenario config %s : %s",
                 scenario_file.c_str(), error.what());

    rclcpp::shutdown();
    return 1;
  }
  std::shared_ptr<ros2_fault_injection::FaultInjector> injector;
  if (scenario.injector.type == "odom") {
    injector = std::make_shared<ros2_fault_injection::OdomFaultInjector>(*node, scenario.injector);
  } else if (scenario.injector.type == "scan") {
    injector = std::make_shared<ros2_fault_injection::ScanFaultInjector>(*node, scenario.injector);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Unknown injector type '%s' in scenario file '%s'",
                 scenario.injector.type.c_str(), scenario_file.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Register every fault with the injector first. Activation is a separate
  // step so immediate and scheduled faults use the same injector interface.
  for (const auto& fault : scenario.faults) {
    if (fault.injector_id != scenario.injector.id) {
      RCLCPP_WARN(node->get_logger(),
                  "Fault '%s' has injector_id '%s' which does not match the scenario injector id "
                  "'%s', skipping",
                  fault.id.c_str(), fault.injector_id.c_str(), scenario.injector.id.c_str());
      continue;
    }
    injector->add_fault(fault);
  }
  ros2_fault_injection::FaultEventPublisher event_pub(*node);

  auto set_fault_state_service = node->create_service<ros2_fault_injection::srv::SetFaultState>(
      "/fault_injection/set_fault_state",
      [node, injector, &event_pub](
          const std::shared_ptr<ros2_fault_injection::srv::SetFaultState::Request> request,
          std::shared_ptr<ros2_fault_injection::srv::SetFaultState::Response> response) {
        if (request->fault_id.empty()) {
          response->success = false;
          response->message = "fault_id cannot be empty";
          RCLCPP_WARN(node->get_logger(), "Received request with empty fault_id");
          return;
        }

        if (!injector->has_fault(request->fault_id)) {
          response->success = false;
          response->message = "unknown fault_id: " + request->fault_id;
          RCLCPP_WARN(node->get_logger(), "Received request with unknown fault_id: %s",
                      request->fault_id.c_str());
          return;
        }

        std_msgs::msg::String event_msg;

        if (request->active) {
          injector->activate_fault(request->fault_id);
          response->message = "activated fault: " + request->fault_id;
          event_msg.data = "activated fault:" + request->fault_id;
          event_pub.publish(request->fault_id, "activated");
          RCLCPP_INFO(node->get_logger(), "Activated fault: %s", request->fault_id.c_str());
        } else {
          injector->deactivate_fault(request->fault_id);
          response->message = "deactivated fault: " + request->fault_id;
          event_msg.data = "deactivated fault:" + request->fault_id;
          event_pub.publish(request->fault_id, "deactivated");
          RCLCPP_INFO(node->get_logger(), "Deactivated fault: %s", request->fault_id.c_str());
        }

        response->success = true;
      });

  auto list_faults_service = node->create_service<ros2_fault_injection::srv::ListFaults>(
      "/fault_injection/list_faults",
      [node, injector](
          const std::shared_ptr<ros2_fault_injection::srv::ListFaults::Request> request,
          std::shared_ptr<ros2_fault_injection::srv::ListFaults::Response> response) {
        (void)request;  // unused

        response->fault_ids = injector->fault_ids();
        response->active_fault_ids = injector->active_fault_ids();
      });

  ros2_fault_injection::FaultScheduler scheduler(*node, event_pub);
  scheduler.schedule(scenario.faults, *injector, scenario.initially_active_faults);

  RCLCPP_INFO(node->get_logger(), "Fault injector running: %s -> %s using scenario file %s",
              scenario.injector.input_topic.c_str(), scenario.injector.output_topic.c_str(),
              scenario_file.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
