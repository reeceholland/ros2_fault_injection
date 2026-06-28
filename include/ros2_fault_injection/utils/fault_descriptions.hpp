// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_DESCRIPTIONS_HPP_
#define ROS2_FAULT_INJECTION__FAULT_DESCRIPTIONS_HPP_

#include <string>
#include <unordered_map>

#include "ros2_fault_injection/config/fault_config.hpp"

namespace ros2_fault_injection::utils
{

std::string describe_fault(const FaultConfig & fault);

std::string describe_config(const std::unordered_map<std::string, std::string> & config);

std::string describe_config_update(const std::string & key, const std::string & value);

}  // namespace ros2_fault_injection::utils


namespace ros2_fault_injection
{
using utils::describe_config;
using utils::describe_config_update;
using utils::describe_fault;
}  // namespace ros2_fault_injection
#endif  // ROS2_FAULT_INJECTION__FAULT_DESCRIPTIONS_HPP_
