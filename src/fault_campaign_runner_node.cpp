// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/config/campaign_config_parser.hpp"
#include "ros2_fault_injection/core/campaign_runner.hpp"

namespace
{

std::string table_cell(std::string value)
{
  std::string escaped;
  escaped.reserve(value.size());

  for (const auto character : value) {
    if (character == '|') {
      escaped += "\\|";
    } else if (character == '\n' || character == '\r') {
      escaped += ' ';
    } else {
      escaped += character;
    }
  }

  return escaped;
}

std::string create_campaign_report_markdown(
  const ros2_fault_injection::CampaignRunResult & result)
{
  std::ostringstream report;
  report << "# Fault Injection Campaign: " << result.campaign_name << "\n\n";
  report << "- Total runs: " << result.total_runs << "\n";
  report << "- Passed runs: " << result.passed_runs << "\n";
  report << "- Failed runs: " << result.failed_runs << "\n\n";
  report << "| Repeat | Variant | Fault | Key | Value | Result | Message |\n";
  report << "| --- | --- | --- | --- | --- | --- | --- |\n";

  for (const auto & run : result.runs) {
    report << "| " << run.repeat
           << " | " << run.variant_index
           << " | " << table_cell(run.fault_id)
           << " | " << table_cell(run.key)
           << " | " << table_cell(run.value)
           << " | " << (run.result.success ? "passed" : "failed")
           << " | " << table_cell(run.result.message)
           << " |\n";
  }

  return report.str();
}

void resolve_base_scenario_relative_to_campaign(
  ros2_fault_injection::CampaignConfig & campaign,
  const std::string & campaign_file)
{
  const std::filesystem::path base_scenario_path{campaign.base_scenario};

  if (base_scenario_path.is_absolute()) {
    return;
  }

  const auto campaign_path = std::filesystem::absolute(campaign_file);
  campaign.base_scenario =
    (campaign_path.parent_path() / base_scenario_path).lexically_normal().string();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fault_campaign_runner");

  node->declare_parameter<std::string>("campaign_file", "");
  node->declare_parameter<std::string>("report_file", "");
  node->declare_parameter<double>("timeout_override", 0.0);

  const auto campaign_file = node->get_parameter("campaign_file").as_string();
  const auto report_file = node->get_parameter("report_file").as_string();
  const auto timeout_override = node->get_parameter("timeout_override").as_double();

  if (campaign_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter: campaign_file");
    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::CampaignConfig campaign;

  try {
    campaign = ros2_fault_injection::load_campaign_config(campaign_file);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load campaign config '%s': %s",
                 campaign_file.c_str(), error.what());
    rclcpp::shutdown();
    return 1;
  }

  resolve_base_scenario_relative_to_campaign(campaign, campaign_file);

  const auto timeout_seconds =
    timeout_override > 0.0 ? timeout_override : campaign.timeout;

  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0 ||
    timeout_seconds >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1000.0 ||
    !std::isfinite(timeout_override) || timeout_override < 0.0)
  {
    RCLCPP_ERROR(node->get_logger(),
      "Timeout must be finite, positive and representable in milliseconds");
    rclcpp::shutdown();
    return 1;
  }

  ros2_fault_injection::CampaignRunner runner(
    *node, campaign,
    std::chrono::milliseconds{static_cast<std::int64_t>(timeout_seconds * 1000.0)});

  const auto result = runner.run();

  if (!report_file.empty()) {
    std::ofstream output(report_file);
    output << create_campaign_report_markdown(result);
    output.flush();
    if (!output) {
      RCLCPP_ERROR(node->get_logger(), "Failed to write campaign report: %s", report_file.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  if (result.success()) {
    RCLCPP_INFO(node->get_logger(), "Campaign passed: %d/%d runs passed",
                result.passed_runs, result.total_runs);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Campaign failed: %d/%d runs failed",
                 result.failed_runs, result.total_runs);
  }

  rclcpp::shutdown();
  return result.success() ? 0 : 1;
}
