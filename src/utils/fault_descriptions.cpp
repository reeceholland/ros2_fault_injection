#include "ros2_fault_injection/utils/fault_descriptions.hpp"

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

namespace ros2_fault_injection
{

std::string describe_config(const std::unordered_map<std::string, std::string> & config)
{
  std::vector<std::pair<std::string, std::string>> entries(config.begin(), config.end());
  std::sort(entries.begin(), entries.end());

  std::ostringstream out;
  out << "config={";

  for (size_t i = 0; i < entries.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << entries[i].first << "=" << entries[i].second;
  }
  out << "}";
  return out.str();
}

std::string describe_fault(const FaultConfig & fault)
{
  std::vector<std::string> parts;

  if (fault.start.has_value()) {
    parts.push_back("scheduled");
    parts.push_back("start=" + std::to_string(fault.start->count()) + "ms");
  } else {
    parts.push_back("manual-only");
  }

  if (fault.duration.has_value()) {
    parts.push_back("duration=" + std::to_string(fault.duration->count()) + "ms");
  }

  parts.push_back(describe_config(fault.config));

  std::ostringstream out;

  for (size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }

    out << parts[i];
  }

  return out.str();
}

std::string describe_config_update(const std::string & key, const std::string & value)
{
  return "config_update={" + key + "=" + value + "}";
}

}  // namespace ros2_fault_injection
