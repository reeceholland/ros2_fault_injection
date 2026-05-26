// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/config/fault_config_schema.hpp"

#include <charconv>
#include <system_error>
namespace ros2_fault_injection
{
namespace
{


bool parse_double(const std::string & text, double & value)
{
  const auto * begin = text.data();
  const auto * end = text.data() + text.size();

  const auto result = std::from_chars(begin, end, value);
  return result.ec == std::errc{} && result.ptr == end;
}

bool parse_int(const std::string & text, int & value)
{
  const auto * begin = text.data();
  const auto * end = text.data() + text.size();

  const auto result = std::from_chars(begin, end, value);
  return result.ec == std::errc{} && result.ptr == end;
}

bool is_special_float_value(const std::string & value)
{
  return value == "inf" || value == "+inf" || value == "-inf" || value == "nan";
}

bool contains(const std::unordered_set<std::string> & keys, const std::string & key)
{
  return keys.find(key) != keys.end();
}

const std::unordered_set<std::string> kNumberKeys = {
  "x_bias",
  "y_bias",
  "z_bias",
  "yaw_bias_deg",
  "roll_bias_deg",
  "pitch_bias_deg",
  "range_bias",
  "sector_min_deg",
  "sector_max_deg",
  "velocity_bias",
  "angular_velocity_z_bias",
  "linear_acceleration_x_bias",
  "linear_acceleration_y_bias",
  "linear_acceleration_z_bias",
};

const std::unordered_set<std::string> kNonNegativeNumberKeys = {
  "x_noise_stddev",
  "y_noise_stddev",
  "yaw_noise_stddev_deg",
  "pose_covariance_scale",
  "twist_covariance_scale",
  "pose_covariance_floor",
  "twist_covariance_floor",
  "range_noise_stddev",
  "velocity_noise_stddev",
  "angular_velocity_z_noise_stddev",
  "linear_acceleration_x_noise_stddev",
  "linear_acceleration_y_noise_stddev",
  "linear_acceleration_z_noise_stddev",
};

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
  "drop_probability", "delay_ms", "range_bias", "range_noise_stddev",
  "sector_min_deg", "sector_max_deg", "sector_value",
};

const std::unordered_set<std::string> kJointStateKeys = {
  "drop_probability",
  "delay_ms",
  "velocity_bias",
  "velocity_noise_stddev",
};

const std::unordered_set<std::string> kImuKeys = {
  "drop_probability", "delay_ms",
  "angular_velocity_z_bias", "angular_velocity_z_noise_stddev",
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

const std::unordered_set<std::string> kTfKeys = {
  "parent_frame", "child_frame", "drop_probability", "delay_ms", "x_bias",
  "y_bias", "z_bias", "roll_bias_deg", "pitch_bias_deg", "yaw_bias_deg",
};

}  // namespace

const std::unordered_set<std::string> & allowed_config_keys_for_injector_type(
  const std::string & injector_type)
{
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

  if (injector_type == "tf") {
    return kTfKeys;
  }

  return kEmptyKeys;
}

bool is_allowed_config_key(const std::string & injector_type, const std::string & key)
{
  const auto & keys = allowed_config_keys_for_injector_type(injector_type);
  return keys.find(key) != keys.end();
}


std::optional<std::string> validate_config_value(
  const std::string & injector_type,
  const std::string & key,
  const std::string & value)
{
  if (!is_allowed_config_key(injector_type, key)) {
    return "key '" + key + "' is not valid for injector type '" + injector_type + "'";
  }

  if (key == "drop_probability") {
    double parsed_value = 0.0;
    if (!parse_double(value, parsed_value)) {
      return "config '" + key + "' must be a number";
    }

    if (parsed_value < 0.0 || parsed_value > 1.0) {
      return "config '" + key + "' must be between 0 and 1";
    }

    return std::nullopt;
  }

  if (key == "delay_ms") {
    int parsed_value = 0;
    if (!parse_int(value, parsed_value)) {
      return "config '" + key + "' must be an integer";
    }

    if (parsed_value < 0) {
      return "config '" + key + "' must be >= 0";
    }

    return std::nullopt;
  }

  if (key == "force_failure") {
    if (value != "true" && value != "false") {
      return "config '" + key + "' must be 'true' or 'false'";
    }

    return std::nullopt;
  }

  if (key == "sector_value") {
    if (is_special_float_value(value)) {
      return std::nullopt;
    }

    double parsed_value = 0.0;
    if (!parse_double(value, parsed_value)) {
      return "config '" + key + "' must be a number, inf, -inf, or nan";
    }

    return std::nullopt;
  }

  if (key == "parent_frame" || key == "child_frame") {
    if (value.empty()) {
      return "config '" + key + "' must not be empty";
    }

    return std::nullopt;
  }

  if (contains(kNonNegativeNumberKeys, key)) {
    double parsed_value = 0.0;
    if (!parse_double(value, parsed_value)) {
      return "config '" + key + "' must be a number";
    }

    if (parsed_value < 0.0) {
      return "config '" + key + "' must be >= 0";
    }

    return std::nullopt;
  }

  if (contains(kNumberKeys, key)) {
    double parsed_value = 0.0;
    if (!parse_double(value, parsed_value)) {
      return "config '" + key + "' must be a number";
    }

    return std::nullopt;
  }

  return std::nullopt;
}

}  // namespace ros2_fault_injection
