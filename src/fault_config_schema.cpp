#include "ros2_fault_injection/fault_config_schema.hpp"

namespace ros2_fault_injection {
namespace {

const std::unordered_set<std::string> kOdomKeys = {
    "drop_probability", "delay_ms",       "x_bias",       "y_bias",
    "x_noise_stddev",   "y_noise_stddev", "yaw_bias_deg", "yaw_noise_stddev_deg",
};

const std::unordered_set<std::string> kScanKeys = {
    "drop_probability", "delay_ms",       "range_bias",   "range_noise_stddev",
    "sector_min_deg",   "sector_max_deg", "sector_value",
};

const std::unordered_set<std::string> kJointStateKeys = {
    "drop_probability",
    "delay_ms",
    "velocity_bias",
    "velocity_noise_stddev",
};

const std::unordered_set<std::string> kEmptyKeys = {};

}  // namespace

const std::unordered_set<std::string>& allowed_config_keys_for_injector_type(
    const std::string& injector_type) {
  if (injector_type == "odom") {
    return kOdomKeys;
  }

  if (injector_type == "scan") {
    return kScanKeys;
  }

  if (injector_type == "joint_state") {
    return kJointStateKeys;
  }

  return kEmptyKeys;
}

bool is_allowed_config_key(const std::string& injector_type, const std::string& key) {
  const auto& keys = allowed_config_keys_for_injector_type(injector_type);
  return keys.find(key) != keys.end();
}

}  // namespace ros2_fault_injection