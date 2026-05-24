#include "ros2_fault_injection/config/fault_config_schema.hpp"

#include <gtest/gtest.h>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/config/scenario_config.hpp"

namespace ros2_fault_injection
{

TEST(FaultConfigSchema, AcceptsValidFaultConfig) {
  EXPECT_TRUE(is_allowed_config_key("odom", "x_bias"));
  EXPECT_TRUE(is_allowed_config_key("odom", "drop_probability"));
  EXPECT_TRUE(is_allowed_config_key("scan", "range_bias"));
  EXPECT_TRUE(is_allowed_config_key("joint_state", "velocity_bias"));
  EXPECT_TRUE(is_allowed_config_key("imu", "linear_acceleration_x_noise_stddev"));
  EXPECT_TRUE(is_allowed_config_key("imu", "angular_velocity_z_bias"));
}

TEST(FaultConfigSchema, RejectsInvalidFaultConfig) {
  EXPECT_FALSE(is_allowed_config_key("odom", "range_bias"));
  EXPECT_FALSE(is_allowed_config_key("scan", "velocity_bias"));
  EXPECT_FALSE(is_allowed_config_key("joint_state", "x_bias"));
  EXPECT_FALSE(is_allowed_config_key("imu", "x_bias"));
  EXPECT_FALSE(is_allowed_config_key("unknown_injector", "some_key"));
}

}  // namespace ros2_fault_injection
