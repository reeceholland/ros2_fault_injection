#include "ros2_fault_injection/utils/fault_descriptions.hpp"

#include <gtest/gtest.h>

#include "ros2_fault_injection/config/fault_config.hpp"
#include "ros2_fault_injection/config/scenario_config.hpp"

namespace ros2_fault_injection {

TEST(FaultDescriptions, DescribeConfigFormatsConfig) {
  std::unordered_map<std::string, std::string> config;
  config["x_bias"] = "1.0";
  config["drop_probability"] = "0.5";

  const auto description = describe_config(config);

  EXPECT_NE(description.find("x_bias=1.0"), std::string::npos);
  EXPECT_NE(description.find("drop_probability=0.5"), std::string::npos);
}

TEST(FaultDescriptions, DescribeFaultFormatsFault) {
  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.start = std::chrono::milliseconds{1000};
  fault.duration = std::chrono::milliseconds{500};
  fault.config["x_bias"] = "1.0";

  const auto description = describe_fault(fault);

  EXPECT_NE(description.find("scheduled"), std::string::npos);
  EXPECT_NE(description.find("start=1000ms"), std::string::npos);
  EXPECT_NE(description.find("duration=500ms"), std::string::npos);
  EXPECT_NE(description.find("x_bias=1.0"), std::string::npos);
}

TEST(FaultDescriptions, DescribeConfigUpdateFormatsUpdate) {
  const auto description = describe_config_update("x_bias", "2.0");

  EXPECT_EQ(description, "config_update={x_bias=2.0}");
}

}  // namespace ros2_fault_injection