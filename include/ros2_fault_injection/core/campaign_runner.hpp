// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__CORE__CAMPAIGN_RUNNER_HPP_
#define ROS2_FAULT_INJECTION__CORE__CAMPAIGN_RUNNER_HPP_

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "ros2_fault_injection/core/campaign_config.hpp"
#include "ros2_fault_injection/core/fault_scenario_runner.hpp"

namespace ros2_fault_injection::core
{

/**
 * @brief Result for one generated scenario run inside a campaign.
 */
struct CampaignScenarioRun
{
  int repeat{0};
  std::size_t variant_index{0};
  std::string fault_id;
  std::string key;
  std::string value;
  ScenarioRunnerResult result;
};

/**
 * @brief Aggregate campaign execution result.
 */
struct CampaignRunResult
{
  std::string campaign_name;
  int total_runs{0};
  int passed_runs{0};
  int failed_runs{0};
  std::vector<CampaignScenarioRun> runs;

  bool success() const
  {
    return total_runs > 0 && failed_runs == 0;
  }
};

/**
 * @brief Runs a campaign by expanding variants into scenario executions.
 */
class CampaignRunner
{
public:
  CampaignRunner(
    rclcpp::Node & node,
    CampaignConfig campaign,
    std::chrono::milliseconds timeout);

  CampaignRunResult run();

private:
  CampaignScenarioRun run_scenario(
    int repeat,
    std::size_t variant_index,
    const CampaignVariant & variant,
    const std::string & value) const;

  rclcpp::Node & node_;
  CampaignConfig campaign_;
  std::chrono::milliseconds timeout_;
};

}  // namespace ros2_fault_injection::core

namespace ros2_fault_injection
{
using core::CampaignRunResult;
using core::CampaignRunner;
using core::CampaignScenarioRun;
}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__CORE__CAMPAIGN_RUNNER_HPP_
