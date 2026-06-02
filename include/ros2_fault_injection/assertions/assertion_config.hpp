// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTION_CONFIG_HPP_
#define ROS2_FAULT_INJECTION__ASSERTION_CONFIG_HPP_

#include <optional>
#include <string>

namespace ros2_fault_injection
{
struct AssertionConfig
{
  std::string id;
  std::string type;
  std::string topic;
  std::string fault_id;
  std::string state;
  std::optional<double> min_hz;
  std::optional<double> window;
  std::optional<double> within;
  std::optional<double> duration;
};
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__ASSERTION_CONFIG_HPP_
