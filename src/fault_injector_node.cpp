#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/fault_event_publisher.hpp"
#include "ros2_fault_injection/fault_injector.hpp"
#include "ros2_fault_injection/fault_injector_factory.hpp"
#include "ros2_fault_injection/fault_scheduler.hpp"
#include "ros2_fault_injection/joint_state_fault_injector.hpp"
#include "ros2_fault_injection/odom_fault_injector.hpp"
#include "ros2_fault_injection/scan_fault_injector.hpp"
#include "ros2_fault_injection/scenario_config.hpp"
#include "ros2_fault_injection/scenario_validator.hpp"
#include "ros2_fault_injection/srv/list_faults.hpp"
#include "ros2_fault_injection/srv/set_fault_state.hpp"

int main(int argc, char** argv) {
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
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load scenario config %s : %s",
                 scenario_file.c_str(), error.what());

    rclcpp::shutdown();
    return 1;
  }

  const auto validation_result = ros2_fault_injection::validate_scenario(scenario);

  for (const auto& warning : validation_result.warnings) {
    RCLCPP_WARN(node->get_logger(), "Scenario config warning: %s", warning.c_str());
  }

  if (!validation_result.ok()) {
    for (const auto& error : validation_result.errors) {
      RCLCPP_ERROR(node->get_logger(), "Scenario config error: %s", error.c_str());
    }

    rclcpp::shutdown();
    return 1;
  }

  std::unordered_map<std::string, std::shared_ptr<ros2_fault_injection::FaultInjector>> injectors;
  ros2_fault_injection::FaultInjectorFactory injector_factory(*node);

  for (const auto& injector_config : scenario.injectors) {
    auto injector = injector_factory.create(injector_config);
    if (!injector) {
      RCLCPP_ERROR(node->get_logger(), "Unknown injector type '%s' for injector '%s'",
                   injector_config.type.c_str(), injector_config.id.c_str());
      rclcpp::shutdown();
      return 1;
    }

    injectors[injector_config.id] = injector;
  }

  // Register every fault with the injector it targets. The validator has
  // already checked these ids, but keep a defensive runtime check here too.
  for (const auto& fault : scenario.faults) {
    const auto injector_it = injectors.find(fault.injector_id);
    if (injector_it == injectors.end()) {
      RCLCPP_WARN(node->get_logger(), "Fault '%s' targets unknown injector_id '%s', skipping",
                  fault.id.c_str(), fault.injector_id.c_str());
      continue;
    }

    injector_it->second->add_fault(fault);
  }

  ros2_fault_injection::FaultEventPublisher event_pub(*node);

  auto find_injector_for_fault =
      [&injectors](
          const std::string& fault_id) -> std::shared_ptr<ros2_fault_injection::FaultInjector> {
    for (const auto& [injector_id, injector] : injectors) {
      (void)injector_id;
      if (injector->has_fault(fault_id)) {
        return injector;
      }
    }

    return nullptr;
  };

  auto set_fault_state_service = node->create_service<ros2_fault_injection::srv::SetFaultState>(
      "/fault_injection/set_fault_state",
      [node, find_injector_for_fault, &event_pub](
          const std::shared_ptr<ros2_fault_injection::srv::SetFaultState::Request> request,
          std::shared_ptr<ros2_fault_injection::srv::SetFaultState::Response> response) {
        if (request->fault_id.empty()) {
          response->success = false;
          response->message = "fault_id cannot be empty";
          RCLCPP_WARN(node->get_logger(), "Received request with empty fault_id");
          return;
        }

        const auto injector = find_injector_for_fault(request->fault_id);
        if (!injector) {
          response->success = false;
          response->message = "unknown fault_id: " + request->fault_id;
          RCLCPP_WARN(node->get_logger(), "Received request with unknown fault_id: %s",
                      request->fault_id.c_str());
          return;
        }

        if (request->active) {
          injector->activate_fault(request->fault_id);
          response->message = "activated fault: " + request->fault_id;
          event_pub.publish(request->fault_id, "active");
          RCLCPP_INFO(node->get_logger(), "Activated fault: %s", request->fault_id.c_str());
        } else {
          injector->deactivate_fault(request->fault_id);
          response->message = "deactivated fault: " + request->fault_id;
          event_pub.publish(request->fault_id, "inactive");
          RCLCPP_INFO(node->get_logger(), "Deactivated fault: %s", request->fault_id.c_str());
        }

        response->success = true;
      });

  auto list_faults_service = node->create_service<ros2_fault_injection::srv::ListFaults>(
      "/fault_injection/list_faults",
      [&injectors](const std::shared_ptr<ros2_fault_injection::srv::ListFaults::Request> request,
                   std::shared_ptr<ros2_fault_injection::srv::ListFaults::Response> response) {
        (void)request;

        for (const auto& [injector_id, injector] : injectors) {
          (void)injector_id;

          const auto fault_ids = injector->fault_ids();
          response->fault_ids.insert(response->fault_ids.end(), fault_ids.begin(), fault_ids.end());

          const auto active_fault_ids = injector->active_fault_ids();
          response->active_fault_ids.insert(response->active_fault_ids.end(),
                                            active_fault_ids.begin(), active_fault_ids.end());
        }
      });

  ros2_fault_injection::FaultScheduler scheduler(*node, event_pub);

  for (const auto& [injector_id, injector] : injectors) {
    std::vector<ros2_fault_injection::FaultConfig> targeted_faults;
    for (const auto& fault : scenario.faults) {
      if (fault.injector_id == injector_id) {
        targeted_faults.push_back(fault);
      }
    }

    scheduler.schedule(targeted_faults, *injector, scenario.initially_active_faults);
  }

  RCLCPP_INFO(node->get_logger(),
              "Fault injector running with %zu injectors using scenario file %s", injectors.size(),
              scenario_file.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
