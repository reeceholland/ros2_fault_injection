// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/config/scenario_validator.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace ros2_fault_injection
{
namespace
{

std::string config_path(const std::string & filename)
{
  return std::string(ROS2_FAULT_INJECTION_SOURCE_DIR) + "/config/" + filename;
}

}  // namespace

TEST(ScenarioExamples, ShippedExampleConfigsParseAndValidate) {
  const std::vector<std::string> files = {
    "odom_faults.yaml",
    "scan_faults.yaml",
    "imu_faults.yaml",
    "motor_feedback_faults.yaml",
    "trigger_service_faults.yaml",
    "tf_faults.yaml",
    "multi_injector_faults.yaml",
  };

  for (const auto & file : files) {
    SCOPED_TRACE(file);

    const auto scenario = load_scenario_config(config_path(file));
    const auto result = validate_scenario(scenario);

    EXPECT_TRUE(result.ok());
    EXPECT_TRUE(result.errors.empty());
    EXPECT_TRUE(result.warnings.empty());
  }
}

}  // namespace ros2_fault_injection
