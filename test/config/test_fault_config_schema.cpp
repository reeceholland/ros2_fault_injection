// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/fault_config_schema.hpp"

#include <gtest/gtest.h>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/config/scenario_config.hpp"

namespace ros2_fault_injection::config
{

TEST(FaultConfigSchema, AcceptsValidFaultConfig)
{
  EXPECT_TRUE(is_allowed_config_key("odom", "x_bias"));
  EXPECT_TRUE(is_allowed_config_key("odom", "drop_probability"));
  EXPECT_TRUE(is_allowed_config_key("scan", "range_bias"));
  EXPECT_TRUE(is_allowed_config_key("joint_state", "velocity_bias"));
  EXPECT_TRUE(is_allowed_config_key("imu", "linear_acceleration_x_noise_stddev"));
  EXPECT_TRUE(is_allowed_config_key("imu", "angular_velocity_z_bias"));
}

TEST(FaultConfigSchema, RejectsInvalidFaultConfig)
{
  EXPECT_FALSE(is_allowed_config_key("odom", "range_bias"));
  EXPECT_FALSE(is_allowed_config_key("scan", "velocity_bias"));
  EXPECT_FALSE(is_allowed_config_key("joint_state", "x_bias"));
  EXPECT_FALSE(is_allowed_config_key("imu", "x_bias"));
  EXPECT_FALSE(is_allowed_config_key("unknown_injector", "some_key"));
}

TEST(FaultConfigSchema, ValidatesRuntimeConfigValues)
{
  EXPECT_FALSE(validate_config_value("imu", "drop_probability", "0.25").has_value());
  EXPECT_FALSE(validate_config_value("imu", "linear_acceleration_x_noise_stddev",
      "0.1").has_value());
  EXPECT_FALSE(validate_config_value("trigger_service", "force_failure", "true").has_value());
  EXPECT_FALSE(validate_config_value("scan", "sector_value", "-inf").has_value());

  EXPECT_TRUE(validate_config_value("imu", "drop_probability", "Helloo").has_value());
  EXPECT_TRUE(validate_config_value("imu", "drop_probability", "1.5").has_value());
  EXPECT_TRUE(validate_config_value("imu", "linear_acceleration_x_noise_stddev",
      "-0.1").has_value());
  EXPECT_TRUE(validate_config_value("odom", "delay_ms", "10.5").has_value());
  EXPECT_TRUE(validate_config_value("trigger_service", "force_failure", "yes").has_value());
  EXPECT_TRUE(validate_config_value("tf", "parent_frame", "").has_value());
}

}  // namespace ros2_fault_injection::config
