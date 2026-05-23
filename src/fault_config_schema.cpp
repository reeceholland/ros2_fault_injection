#include "ros2_fault_injection/fault_config_schema.hpp"

namespace ros2_fault_injection {
namespace {

const std::unordered_set<std::string> kOdomKeys = {
    "drop_probability",
    "delay_ms",
    "x_bias",
    "y_bias",
    "x_noise_stddev",
    "y_noise_stddev",
    "yaw_bias_deg",
    "yaw_noise_stddev_deg",
    "pose_covariance_scale",
    "twist_covariance_scale",
    "pose_covariance_floor",
    "twist_covariance_floor",
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

const std::unordered_set<std::string> kImuKeys = {
    "drop_probability",           "delay_ms",
    "angular_velocity_z_bias",    "angular_velocity_z_noise_stddev",
    "linear_acceleration_x_bias", "linear_acceleration_x_noise_stddev",
    "linear_acceleration_y_bias", "linear_acceleration_y_noise_stddev",
    "linear_acceleration_z_bias", "linear_acceleration_z_noise_stddev",
};

const std::unordered_set<std::string> kEmptyKeys = {};

const std::unordered_set<std::string> kTriggerServiceKeys = {
    "delay_ms",
    "force_failure",
    "failure_message",
};

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

  if (injector_type == "imu") {
    return kImuKeys;
  }

  if (injector_type == "trigger_service") {
    return kTriggerServiceKeys;
  }

  return kEmptyKeys;
}

bool is_allowed_config_key(const std::string& injector_type, const std::string& key) {
  const auto& keys = allowed_config_keys_for_injector_type(injector_type);
  return keys.find(key) != keys.end();
}

}  // namespace ros2_fault_injection