// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/campaign_config.hpp"

#include <string>

namespace ros2_fault_injection::core
{

bool apply_campaign_value(
  config::ScenarioConfig & scenario,
  const CampaignVariant & variant,
  const std::string & value,
  std::string & error)
{
  for (auto & fault : scenario.faults) {
    if (fault.id == variant.fault_id) {
      fault.config[variant.key] = value;
      error.clear();
      return true;
    }
  }

  error = "unknown campaign fault_id: " + variant.fault_id;
  return false;
}

}  // namespace ros2_fault_injection::core
