// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "ros2_fault_injection/core/campaign_runner.hpp"

class CampaignRunnerTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(CampaignRunnerTest, CountsEveryValueAndRepeatEvenWhenBaseScenarioIsMissing)
{
  rclcpp::Node node("campaign_runner_test");
  ros2_fault_injection::CampaignConfig config;
  config.name = "missing_base";
  config.base_scenario = "/nonexistent/campaign/scenario.yaml";
  config.repeats = 2;
  ros2_fault_injection::CampaignVariant variant;
  variant.fault_id = "delay";
  variant.key = "delay_ms";
  variant.values = {"0", "10", "20"};
  config.variants.push_back(variant);
  ros2_fault_injection::CampaignRunner runner(node, config, std::chrono::milliseconds(100));
  const auto result = runner.run();
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.total_runs, 6);
  EXPECT_EQ(result.failed_runs, 6);
  ASSERT_EQ(result.runs.size(), 6u);
  EXPECT_EQ(result.runs.back().repeat, 2);
  EXPECT_EQ(result.runs.back().value, "20");
  EXPECT_NE(result.runs.front().result.message.find("failed to load"), std::string::npos);
}

TEST_F(CampaignRunnerTest, RejectsEmptyCampaign)
{
  rclcpp::Node node("empty_campaign_test");
  ros2_fault_injection::CampaignRunner runner(
    node, {}, std::chrono::milliseconds(100));
  const auto result = runner.run();
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failed_runs, 1);
}
