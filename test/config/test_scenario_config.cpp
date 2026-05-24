#include "ros2_fault_injection/config/scenario_config.hpp"

#include <chrono>
#include <fstream>

#include <gtest/gtest.h>

std::string write_temp_yaml(const std::string & contents)
{
  const auto path = "/tmp/ros2_fault_injection_test_scenario.yaml";

  std::ofstream file(path);
  file << contents;
  file.close();

  return path;
}

TEST(ScenarioConfig, ParseMultiInjectorScenario) {
  const auto path =
    write_temp_yaml(
    R"(
injectors:
  - id: odom
    type: odom
    input_topic: /odom_raw
    output_topic: /odom
    qos_depth: 10
  - id: scan
    type: scan
    input_topic: /scan_raw
    output_topic: /scan
    qos_depth: 5
  - id: motor_feedback
    type: joint_state
    input_topic: /platform/motors/feedback_raw
    output_topic: /platform/motors/feedback
    qos_depth: 10
faults:
  - id: odom_bias
    injector_id: odom
    config:
      x_bias: 1.0
      drop_probability: 0.25
  - id: startup_fault
    injector_id: odom
    active_on_startup: true
    config:
      x_bias: 2.0
  - id: scan_noise
    injector_id: scan
    config:
      range_noise_stddev: 0.1
  - id: motor_fault
    injector_id: motor_feedback
    config:
      velocity_bias: -0.5
  )");

  const auto scenario = ros2_fault_injection::load_scenario_config(path);
  ASSERT_EQ(scenario.injectors.size(), 3u);
  EXPECT_EQ(scenario.injectors[0].id, "odom");
  EXPECT_EQ(scenario.injectors[1].id, "scan");
  EXPECT_EQ(scenario.injectors[2].id, "motor_feedback");

  ASSERT_EQ(scenario.faults.size(), 4u);
  EXPECT_EQ(scenario.faults[0].injector_id, "odom");
  ASSERT_EQ(scenario.initially_active_faults.size(), 1u);
  EXPECT_EQ(scenario.initially_active_faults.front(), "startup_fault");
  EXPECT_EQ(scenario.faults[1].injector_id, "odom");
  EXPECT_EQ(scenario.faults[2].injector_id, "scan");
  EXPECT_EQ(scenario.faults[3].injector_id, "motor_feedback");
}

TEST(ScenarioConfig, ParseTriggerServiceInjector) {
  const auto path =
    write_temp_yaml(
    R"(
injector:
  id: trigger_service
  type: trigger_service
  proxy_service: /trigger_fault
  target_service: /target_service
faults:
  - id: trigger_fault
    injector_id: trigger_service
    config:
      force_failure: true
      delay_ms: 100
  )");

  const auto scenario = ros2_fault_injection::load_scenario_config(path);
  ASSERT_EQ(scenario.injectors.size(), 1u);
  EXPECT_EQ(scenario.injectors[0].id, "trigger_service");
  EXPECT_EQ(scenario.injectors[0].type, "trigger_service");
  ASSERT_TRUE(scenario.injectors[0].trigger_service.has_value());
  EXPECT_EQ(scenario.injectors[0].trigger_service->proxy_service, "/trigger_fault");
  EXPECT_EQ(scenario.injectors[0].trigger_service->target_service, "/target_service");
  ASSERT_EQ(scenario.faults.size(), 1u);
  EXPECT_EQ(scenario.faults[0].injector_id, "trigger_service");
  EXPECT_EQ(scenario.faults[0].config.at("force_failure"), "true");
  EXPECT_EQ(scenario.faults[0].config.at("delay_ms"), "100");
}
