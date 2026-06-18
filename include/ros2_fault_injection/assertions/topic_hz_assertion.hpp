// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTIONS__TOPIC_HZ_ASSERTION_HPP_
#define ROS2_FAULT_INJECTION__ASSERTIONS__TOPIC_HZ_ASSERTION_HPP_

#include <string>
#include <vector>

#include "rclcpp/time.hpp"

#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"

namespace ros2_fault_injection::assertions
{

class TopicHzAssertion
{
public:
  explicit TopicHzAssertion(const AssertionConfig & config);

  const std::string & topic() const;
  void observe_message(rclcpp::Time stamp);
  void update(double elapsed_seconds, const rclcpp::Time & now);
  AssertionResult result() const;

private:
  AssertionConfig config_;
  AssertionResult result_;
  std::vector<rclcpp::Time> message_times_;
};

} // namespace ros2_fault_injection::assertions

#endif // ROS2_FAULT_INJECTION__ASSERTIONS__TOPIC_HZ_ASSERTION_HPP_
