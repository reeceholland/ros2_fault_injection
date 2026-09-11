// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/campaign_config.hpp"

#include <string>

#include <gtest/gtest.h>

namespace
{

ros2_fault_injection::config::ScenarioConfig make_scenario()
{
  ros2_fault_injection::config::ScenarioConfig scenario;

  ros2_fault_injection::config::FaultConfig fault;
  fault.id = "cmd_vel_delay";
  fault.injector_id = "cmd_vel";
  fault.config["delay_ms"] = "100";

  scenario.faults.push_back(fault);

  return scenario;
}

}  // namespace

TEST(CampaignConfig, ApplyCampaignValueUpdatesMatchingFaultConfig)
{
  auto scenario = make_scenario();

  ros2_fault_injection::core::CampaignVariant variant;
  variant.fault_id = "cmd_vel_delay";
  variant.key = "delay_ms";

  std::string error;
  const auto result =
    ros2_fault_injection::core::apply_campaign_value(scenario, variant, "500", error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(scenario.faults.size(), 1u);
  EXPECT_EQ(scenario.faults.front().config.at("delay_ms"), "500");
}

TEST(CampaignConfig, ApplyCampaignValueCanAddNewConfigKey)
{
  auto scenario = make_scenario();

  ros2_fault_injection::core::CampaignVariant variant;
  variant.fault_id = "cmd_vel_delay";
  variant.key = "drop_probability";

  std::string error;
  const auto result =
    ros2_fault_injection::core::apply_campaign_value(scenario, variant, "0.25", error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(scenario.faults.front().config.at("drop_probability"), "0.25");
}

TEST(CampaignConfig, ApplyCampaignValueRejectsUnknownFault)
{
  auto scenario = make_scenario();

  ros2_fault_injection::core::CampaignVariant variant;
  variant.fault_id = "missing_fault";
  variant.key = "delay_ms";

  std::string error;
  const auto result =
    ros2_fault_injection::core::apply_campaign_value(scenario, variant, "500", error);

  EXPECT_FALSE(result);
  EXPECT_NE(error.find("missing_fault"), std::string::npos);
  EXPECT_EQ(scenario.faults.front().config.at("delay_ms"), "100");
}
