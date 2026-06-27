// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>

#include "rclcpp/time.hpp"

#include "ros2_fault_injection/core/fault_event_publisher.hpp"

namespace ros2_fault_injection::core
{
struct FaultEventRecord
{
  rclcpp::Time stamp;
  std::string fault_id;
  std::string injector_id;
  std::string state;
  std::string source;
  std::string details;
};

inline FaultEventRecord to_record(const FaultEvent & event, const rclcpp::Time & timestamp)
{
  FaultEventRecord record;
  record.stamp = timestamp;
  record.fault_id = event.fault_id;
  record.injector_id = event.injector_id;
  record.state = event.state;
  record.source = event.source;
  record.details = event.details;
  return record;
}
}
