// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/campaign_runner.hpp"

#include <exception>
#include <sstream>
#include <string>
#include <utility>

#include "ros2_fault_injection/config/scenario_config.hpp"
#include "ros2_fault_injection/config/scenario_validator.hpp"
#include "ros2_fault_injection/core/fault_controller.hpp"
#include "ros2_fault_injection/core/fault_event_publisher.hpp"

namespace ros2_fault_injection::core
{
namespace
{

ScenarioRunnerResult failed_scenario_result(const std::string & message)
{
  return ScenarioRunnerResult{
    false,
    message,
    "failed",
    "",
  };
}

std::string join_validation_errors(const config::ValidationResult & validation)
{
  std::ostringstream message;
  message << "generated scenario validation failed";

  for (const auto & error : validation.errors) {
    message << "; " << error;
  }

  return message.str();
}

void record_run(CampaignRunResult & campaign_result, CampaignScenarioRun run)
{
  ++campaign_result.total_runs;

  if (run.result.success) {
    ++campaign_result.passed_runs;
  } else {
    ++campaign_result.failed_runs;
  }

  campaign_result.runs.push_back(std::move(run));
}

}  // namespace

CampaignRunner::CampaignRunner(
  rclcpp::Node & node,
  CampaignConfig campaign,
  std::chrono::milliseconds timeout)
: node_(node), campaign_(std::move(campaign)), timeout_(timeout)
{
}

CampaignRunResult CampaignRunner::run()
{
  CampaignRunResult campaign_result;
  campaign_result.campaign_name = campaign_.name;

  if (campaign_.variants.empty() || campaign_.repeats <= 0 || timeout_.count() <= 0) {
    record_run(
      campaign_result,
      CampaignScenarioRun{0, 0, "", "", "",
        failed_scenario_result(
          "campaign requires variants, positive repeats and a positive timeout")});
    return campaign_result;
  }

  for (int repeat = 1; repeat <= campaign_.repeats; ++repeat) {
    for (std::size_t variant_index = 0; variant_index < campaign_.variants.size();
      ++variant_index)
    {
      const auto & variant = campaign_.variants[variant_index];

      if (variant.values.empty()) {
        record_run(campaign_result, CampaignScenarioRun{
            repeat, variant_index + 1, variant.fault_id, variant.key, "",
            failed_scenario_result("campaign variant has no values")});
      }
      for (const auto & value : variant.values) {
        if (!rclcpp::ok()) {
          record_run(campaign_result, CampaignScenarioRun{
              repeat, variant_index + 1, variant.fault_id, variant.key, value,
              failed_scenario_result("ROS shutdown before campaign completed")});
          return campaign_result;
        }
        record_run(campaign_result, run_scenario(repeat, variant_index + 1, variant, value));
      }
    }
  }

  return campaign_result;
}

CampaignScenarioRun CampaignRunner::run_scenario(
  int repeat,
  std::size_t variant_index,
  const CampaignVariant & variant,
  const std::string & value) const
{
  CampaignScenarioRun run;
  run.repeat = repeat;
  run.variant_index = variant_index;
  run.fault_id = variant.fault_id;
  run.key = variant.key;
  run.value = value;

  config::ScenarioConfig scenario;

  try {
    scenario = config::load_scenario_config(campaign_.base_scenario);
  } catch (const std::exception & error) {
    run.result = failed_scenario_result(
      "failed to load base scenario '" + campaign_.base_scenario + "': " + error.what());
    return run;
  }

  std::string error;
  if (!apply_campaign_value(scenario, variant, value, error)) {
    run.result = failed_scenario_result(error);
    return run;
  }

  const auto validation = config::validate_scenario(scenario);
  if (!validation.ok()) {
    run.result = failed_scenario_result(join_validation_errors(validation));
    return run;
  }

  try {
    FaultEventPublisher event_publisher(node_);
    FaultController controller(node_, campaign_.base_scenario, scenario, event_publisher);
    FaultScenarioRunner scenario_runner(node_, controller, timeout_);

    run.result = scenario_runner.run();
  } catch (const std::exception & error) {
    run.result = failed_scenario_result(std::string("scenario execution failed: ") + error.what());
  }
  return run;
}

}  // namespace ros2_fault_injection::core
