// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "ros2_fault_injection/assertions/assertion_result.hpp"

namespace ros2_fault_injection::core
{
struct ScenarioReport
{
  std::string scenario_file;
  rclcpp::Time start_at;
  rclcpp::Time finished_at;

  std::vector<std::string> injector_ids;
  std::vector<std::string> fault_ids;
  std::vector<assertions::AssertionResult> assertion_results;

  std::string final_result;
};

}
