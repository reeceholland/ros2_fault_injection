// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__CORE__CAMPAIGN_CONFIG_HPP_
#define ROS2_FAULT_INJECTION__CORE__CAMPAIGN_CONFIG_HPP_

#include <string>
#include <vector>

#include "ros2_fault_injection/config/scenario_config.hpp"

namespace ros2_fault_injection::core
{

/**
 * @brief One fault parameter sweep in a campaign.
 *
 * A variant names a fault, one config key on that fault, and the values that
 * should be tried across repeated scenario runs.
 */
struct CampaignVariant
{
  std::string fault_id;
  std::string key;
  std::vector<std::string> values;
};

/**
 * @brief Configuration for a campaign of scenario runs.
 *
 * Campaigns let the framework run the same scenario repeatedly while changing
 * one or more fault configuration values. The first milestone only parses this
 * data; campaign execution can be built on top once the file format is stable.
 */
struct CampaignConfig
{
  std::string name;
  std::string base_scenario;
  double timeout{60.0};
  int repeats{1};
  std::vector<CampaignVariant> variants;
};

/**
 * @brief Apply one campaign value to a scenario fault config.
 *
 * @param scenario Scenario to update in memory.
 * @param variant Campaign variant describing the fault and config key.
 * @param value Config value to apply.
 * @param error Human-readable error when the update fails.
 * @return true when the matching fault was found and updated.
 */
bool apply_campaign_value(
  config::ScenarioConfig & scenario,
  const CampaignVariant & variant,
  const std::string & value,
  std::string & error);

}  // namespace ros2_fault_injection::core

namespace ros2_fault_injection
{
using core::apply_campaign_value;
using core::CampaignConfig;
using core::CampaignVariant;
}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__CORE__CAMPAIGN_CONFIG_HPP_
