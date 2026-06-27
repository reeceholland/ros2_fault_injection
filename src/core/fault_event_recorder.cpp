#include "ros2_fault_injection/core/fault_event_recorder.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace ros2_fault_injection::core
{
  void FaultEventRecorder::record(const FaultEventRecord &event)
  {
    events_.push_back(event);
  }

  std::vector<FaultEventRecord> FaultEventRecorder::events() const
  {
    return events_;
  }

  std::vector<FaultEventRecord> FaultEventRecorder::events_for_fault(const std::string &fault_id) const
  {
    std::vector<FaultEventRecord> result;
    for (const auto &event : events_)
    {
      if (event.fault_id == fault_id)
      {
        result.push_back(event);
      }
    }
    return result;
  }
}