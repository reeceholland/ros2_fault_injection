// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__CONFIG__CAMPAIGN_CONFIG_PARSER_HPP_
#define ROS2_FAULT_INJECTION__CONFIG__CAMPAIGN_CONFIG_PARSER_HPP_

#include <string>

#include "ros2_fault_injection/core/campaign_config.hpp"

namespace ros2_fault_injection::config
{

/**
 * @brief Load a campaign YAML file.
 *
 * @param path Filesystem path to a campaign YAML file.
 * @return Parsed campaign configuration.
 * @throws std::exception when the file is missing required fields or cannot be parsed.
 */
core::CampaignConfig load_campaign_config(const std::string & path);

}  // namespace ros2_fault_injection::config

namespace ros2_fault_injection
{
using config::load_campaign_config;
}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__CONFIG__CAMPAIGN_CONFIG_PARSER_HPP_
