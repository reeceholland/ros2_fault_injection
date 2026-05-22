#include "ros2_fault_injection/scenario_validator.hpp"

#include <chrono>
#include <string>

#include <gtest/gtest.h>

namespace {

ros2_fault_injection::ScenarioConfig valid_odom_scenario() {
  ros2_fault_injection::ScenarioConfig scenario;
  scenario.injector.id = "odom";
  scenario.injector.type = "odom";
  scenario.injector.input_topic = "/odom_raw";
  scenario.injector.output_topic = "/odom";
  scenario.injector.qos_depth = 10;
  scenario.injectors.push_back(scenario.injector);

  ros2_fault_injection::FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";
  fault.config["drop_probability"] = "0.25";

  scenario.faults.push_back(fault);
  return scenario;
}

}  // namespace

TEST(ScenarioValidator, AcceptsValidOdomScenario) {
  const auto result = ros2_fault_injection::validate_scenario(valid_odom_scenario());

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.errors.empty());
  EXPECT_TRUE(result.warnings.empty());
}

TEST(ScenarioValidator, RejectsUnsupportedInjectorType) {
  auto scenario = valid_odom_scenario();
  scenario.injector.type = "camera";
  scenario.injectors.front().type = "camera";

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_FALSE(result.ok());
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors.front().find("unsupported injector.type"), std::string::npos);
}

TEST(ScenarioValidator, WarnsWhenDurationHasNoStart) {
  auto scenario = valid_odom_scenario();
  scenario.faults.front().duration = std::chrono::seconds{10};

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_TRUE(result.ok());
  ASSERT_EQ(result.warnings.size(), 1u);
  EXPECT_NE(result.warnings.front().find("duration but no start"), std::string::npos);
}

TEST(ScenarioValidator, RejectsNegativeDuration) {
  auto scenario = valid_odom_scenario();
  scenario.faults.front().duration = -std::chrono::seconds{1};

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_FALSE(result.ok());
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors.front().find("negative duration"), std::string::npos);
}

TEST(ScenarioValidator, RejectsInvalidDropProbability) {
  auto scenario = valid_odom_scenario();
  scenario.faults.front().config["drop_probability"] = "1.5";

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_FALSE(result.ok());
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors.front().find("drop_probability"), std::string::npos);
}

TEST(ScenarioValidator, WarnsOnUnknownScanKey) {
  ros2_fault_injection::ScenarioConfig scenario;
  scenario.injector.id = "scan";
  scenario.injector.type = "scan";
  scenario.injector.input_topic = "/scan_raw";
  scenario.injector.output_topic = "/scan";
  scenario.injector.qos_depth = 10;
  scenario.injectors.push_back(scenario.injector);

  ros2_fault_injection::FaultConfig fault;
  fault.id = "scan_fault";
  fault.injector_id = "scan";
  fault.config["range_noise_stddev"] = "0.05";
  fault.config["x_bias"] = "1.0";
  scenario.faults.push_back(fault);

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_TRUE(result.ok());
  ASSERT_EQ(result.warnings.size(), 1u);
  EXPECT_NE(result.warnings.front().find("unknown config key 'x_bias'"), std::string::npos);
}


TEST(ScenarioValidator, RejectsDuplicateInjectorIds) {
  auto scenario = valid_odom_scenario();
  scenario.injectors.push_back(scenario.injector);

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_FALSE(result.ok());
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors.front().find("duplicate injector id"), std::string::npos);
}


TEST(ScenarioValidator, AllowsStartupActiveDurationWithoutStartWarning) {
  auto scenario = valid_odom_scenario();
  scenario.initially_active_faults.push_back("odom_bias");
  scenario.faults.front().duration = std::chrono::seconds{10};

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.warnings.empty());
}

TEST(ScenarioValidator, WarnsWhenStartupActiveFaultAlsoHasStart) {
  auto scenario = valid_odom_scenario();
  scenario.initially_active_faults.push_back("odom_bias");
  scenario.faults.front().start = std::chrono::seconds{12};
  scenario.faults.front().duration = std::chrono::seconds{8};

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_TRUE(result.ok());
  ASSERT_EQ(result.warnings.size(), 1u);
  EXPECT_NE(result.warnings.front().find("active_on_startup=true and start"), std::string::npos);
}

TEST(ScenarioValidator, RejectsNegativeYawNoiseStddevDeg) {
  auto scenario = valid_odom_scenario();
  scenario.faults.front().config["yaw_noise_stddev_deg"] = "-1.0";

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_FALSE(result.ok());
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors.front().find("yaw_noise_stddev_deg"), std::string::npos);
}


TEST(ScenarioValidator, AcceptsValidImuScenario) {
  ros2_fault_injection::ScenarioConfig scenario;
  scenario.injector.id = "imu";
  scenario.injector.type = "imu";
  scenario.injector.input_topic = "/sensors/imu_raw";
  scenario.injector.output_topic = "/sensors/imu";
  scenario.injector.qos_depth = 10;
  scenario.injectors.push_back(scenario.injector);

  ros2_fault_injection::FaultConfig fault;
  fault.id = "imu_accel_noise";
  fault.injector_id = "imu";
  fault.config["linear_acceleration_x_noise_stddev"] = "0.15";
  fault.config["linear_acceleration_y_noise_stddev"] = "0.15";
  fault.config["linear_acceleration_z_noise_stddev"] = "0.05";
  scenario.faults.push_back(fault);

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.errors.empty());
  EXPECT_TRUE(result.warnings.empty());
}

TEST(ScenarioValidator, RejectsNegativeImuNoiseStddev) {
  ros2_fault_injection::ScenarioConfig scenario;
  scenario.injector.id = "imu";
  scenario.injector.type = "imu";
  scenario.injector.input_topic = "/sensors/imu_raw";
  scenario.injector.output_topic = "/sensors/imu";
  scenario.injector.qos_depth = 10;
  scenario.injectors.push_back(scenario.injector);

  ros2_fault_injection::FaultConfig fault;
  fault.id = "imu_accel_noise";
  fault.injector_id = "imu";
  fault.config["linear_acceleration_x_noise_stddev"] = "-0.15";
  scenario.faults.push_back(fault);

  const auto result = ros2_fault_injection::validate_scenario(scenario);

  EXPECT_FALSE(result.ok());
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors.front().find("linear_acceleration_x_noise_stddev"), std::string::npos);
}
