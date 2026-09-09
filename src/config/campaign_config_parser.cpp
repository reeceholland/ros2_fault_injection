// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/campaign_config_parser.hpp"

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace ros2_fault_injection::config
{
namespace
{

std::string required_string(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    throw std::runtime_error("Missing required key: " + key);
  }

  return node[key].as<std::string>();
}

std::string scalar_to_string(const YAML::Node & node)
{
  if (!node.IsScalar()) {
    throw std::runtime_error("Campaign variant values must be scalar YAML values");
  }

  return node.as<std::string>();
}

core::CampaignVariant parse_variant(const YAML::Node & node)
{
  core::CampaignVariant variant;
  variant.fault_id = required_string(node, "fault_id");
  variant.key = required_string(node, "key");

  if (!node["values"]) {
    throw std::runtime_error("Missing required key: values");
  }

  if (!node["values"].IsSequence()) {
    throw std::runtime_error("Campaign variant values must be a sequence");
  }

  for (const auto & value_node : node["values"]) {
    variant.values.push_back(scalar_to_string(value_node));
  }

  if (variant.values.empty()) {
    throw std::runtime_error("Campaign variant values must not be empty");
  }

  return variant;
}

}  // namespace

core::CampaignConfig load_campaign_config(const std::string & path)
{
  const auto root = YAML::LoadFile(path);

  if (!root["campaign"]) {
    throw std::runtime_error("Campaign file is missing required 'campaign' block");
  }

  const auto campaign_node = root["campaign"];

  core::CampaignConfig campaign;
  campaign.name = required_string(campaign_node, "name");
  campaign.base_scenario = required_string(campaign_node, "base_scenario");

  if (campaign_node["timeout"]) {
    campaign.timeout = campaign_node["timeout"].as<double>();
  }

  if (campaign_node["repeats"]) {
    campaign.repeats = campaign_node["repeats"].as<int>();
  }

  if (campaign.repeats <= 0) {
    throw std::runtime_error("Campaign repeats must be greater than 0");
  }

  if (campaign.timeout <= 0.0) {
    throw std::runtime_error("Campaign timeout must be greater than 0");
  }

  if (!campaign_node["variants"]) {
    throw std::runtime_error("Missing required key: variants");
  }

  if (!campaign_node["variants"].IsSequence()) {
    throw std::runtime_error("Campaign variants must be a sequence");
  }

  for (const auto & variant_node : campaign_node["variants"]) {
    campaign.variants.push_back(parse_variant(variant_node));
  }

  if (campaign.variants.empty()) {
    throw std::runtime_error("Campaign variants must not be empty");
  }

  return campaign;
}

}  // namespace ros2_fault_injection::config
