// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTION_RESULT_HPP_
#define ROS2_FAULT_INJECTION__ASSERTION_RESULT_HPP_

#include <string>

namespace ros2_fault_injection
{
enum class AssertionState
{
  Pending,
  Passed,
  Failed,
};

struct AssertionResult
{
  std::string id;
  AssertionState state;
  std::string message;
};
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__ASSERTION_RESULT_HPP_
