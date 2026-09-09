// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/campaign_config_parser.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

namespace
{

std::string write_temp_campaign_yaml(const std::string & contents)
{
  const auto path = "/tmp/ros2_fault_injection_test_campaign.yaml";

  std::ofstream file(path);
  file << contents;
  file.close();

  return path;
}

std::string valid_campaign_yaml()
{
  return
    R"(
campaign:
  name: cmd_vel_delay_sweep
  base_scenario: config/omnisim_faults.yaml
  timeout: 100.0
  repeats: 3

  variants:
    - fault_id: cmd_vel_delay
      key: delay_ms
      values: ["0", "100", "250", "500", "1000"]
)";
}

}  // namespace

TEST(CampaignConfigParser, ParsesValidCampaign)
{
  const auto path = write_temp_campaign_yaml(valid_campaign_yaml());

  const auto campaign = ros2_fault_injection::config::load_campaign_config(path);

  EXPECT_EQ(campaign.name, "cmd_vel_delay_sweep");
  EXPECT_EQ(campaign.base_scenario, "config/omnisim_faults.yaml");
  EXPECT_DOUBLE_EQ(campaign.timeout, 100.0);
  EXPECT_EQ(campaign.repeats, 3);

  ASSERT_EQ(campaign.variants.size(), 1u);
  EXPECT_EQ(campaign.variants.front().fault_id, "cmd_vel_delay");
  EXPECT_EQ(campaign.variants.front().key, "delay_ms");
  ASSERT_EQ(campaign.variants.front().values.size(), 5u);
  EXPECT_EQ(campaign.variants.front().values.front(), "0");
  EXPECT_EQ(campaign.variants.front().values.back(), "1000");
}

TEST(CampaignConfigParser, RejectsMissingCampaignBlock)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
name: cmd_vel_delay_sweep
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}

TEST(CampaignConfigParser, RejectsMissingBaseScenario)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
campaign:
  name: cmd_vel_delay_sweep
  variants:
    - fault_id: cmd_vel_delay
      key: delay_ms
      values: ["100"]
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}

TEST(CampaignConfigParser, RejectsNonPositiveRepeats)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
campaign:
  name: cmd_vel_delay_sweep
  base_scenario: config/omnisim_faults.yaml
  repeats: 0
  variants:
    - fault_id: cmd_vel_delay
      key: delay_ms
      values: ["100"]
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}

TEST(CampaignConfigParser, RejectsEmptyVariants)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
campaign:
  name: cmd_vel_delay_sweep
  base_scenario: config/omnisim_faults.yaml
  variants: []
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}

TEST(CampaignConfigParser, RejectsVariantWithoutFaultId)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
campaign:
  name: cmd_vel_delay_sweep
  base_scenario: config/omnisim_faults.yaml
  variants:
    - key: delay_ms
      values: ["100"]
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}

TEST(CampaignConfigParser, RejectsVariantWithoutKey)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
campaign:
  name: cmd_vel_delay_sweep
  base_scenario: config/omnisim_faults.yaml
  variants:
    - fault_id: cmd_vel_delay
      values: ["100"]
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}

TEST(CampaignConfigParser, RejectsVariantWithoutValues)
{
  const auto path =
    write_temp_campaign_yaml(
    R"(
campaign:
  name: cmd_vel_delay_sweep
  base_scenario: config/omnisim_faults.yaml
  variants:
    - fault_id: cmd_vel_delay
      key: delay_ms
)");

  EXPECT_THROW(
    ros2_fault_injection::config::load_campaign_config(path),
    std::runtime_error);
}
