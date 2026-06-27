#pragma once

#include <string>
#include <vector>

#include "ros2_fault_injection/core/fault_event_record.hpp"

namespace ros2_fault_injection::core
{
  class FaultEventRecorder
  {
  public:
    void record(const FaultEventRecord &event);

    std::vector<FaultEventRecord> events() const;

    std::vector<FaultEventRecord> events_for_fault(const std::string &fault_id) const;

  private:
    std::vector<FaultEventRecord> events_;
  };
}