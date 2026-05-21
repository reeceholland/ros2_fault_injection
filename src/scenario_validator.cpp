#include "ros2_fault_injection/scenario_validator.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <string>
#include <string_view>
#include <unordered_set>

#include "ros2_fault_injection/fault_config_schema.hpp"

namespace ros2_fault_injection {
namespace {

bool is_known_injector_type(const std::string& type) {
  return type == "odom" || type == "scan" || type == "joint_state";
}

bool parse_double(const std::string& text, double& value) {
  const auto* begin = text.data();
  const auto* end = text.data() + text.size();

  const auto result = std::from_chars(begin, end, value);
  return result.ec == std::errc{} && result.ptr == end;
}

bool parse_int(const std::string& text, int& value) {
  const auto* begin = text.data();
  const auto* end = text.data() + text.size();

  const auto result = std::from_chars(begin, end, value);
  return result.ec == std::errc{} && result.ptr == end;
}

void validate_injector(const InjectorConfig& injector, ValidationResult& result) {
  if (injector.id.empty()) {
    result.errors.push_back("injector.id must not be empty");
  }

  if (!is_known_injector_type(injector.type)) {
    result.errors.push_back("unsupported injector.type: '" + injector.type + "'");
  }

  if (injector.input_topic.empty()) {
    result.errors.push_back("injector.input_topic must not be empty");
  }

  if (injector.output_topic.empty()) {
    result.errors.push_back("injector.output_topic must not be empty");
  }

  if (injector.qos_depth == 0) {
    result.errors.push_back("injector.qos_depth must be greater than 0");
  }
}

void validate_number_key(const FaultConfig& fault, const std::string& key,
                         ValidationResult& result) {
  const auto it = fault.config.find(key);
  if (it == fault.config.end()) {
    return;
  }

  double value = 0.0;
  if (!parse_double(it->second, value)) {
    result.errors.push_back("fault '" + fault.id + "' config '" + key + "' must be a number");
  }
}

void validate_non_negative_number_key(const FaultConfig& fault, const std::string& key,
                                      ValidationResult& result) {
  const auto it = fault.config.find(key);
  if (it == fault.config.end()) {
    return;
  }

  double value = 0.0;
  if (!parse_double(it->second, value)) {
    result.errors.push_back("fault '" + fault.id + "' config '" + key + "' must be a number");
    return;
  }

  if (value < 0.0) {
    result.errors.push_back("fault '" + fault.id + "' config '" + key + "' must be >= 0");
  }
}

bool is_special_float_value(const std::string& value) {
  return value == "inf" || value == "+inf" || value == "-inf" || value == "nan";
}

void validate_sector_value(const FaultConfig& fault, ValidationResult& result) {
  const auto it = fault.config.find("sector_value");
  if (it == fault.config.end() || is_special_float_value(it->second)) {
    return;
  }

  double value = 0.0;
  if (!parse_double(it->second, value)) {
    result.errors.push_back("fault '" + fault.id +
                            "' config 'sector_value' must be a number, inf, -inf, or nan");
  }
}

void validate_drop_probability(const FaultConfig& fault, ValidationResult& result) {
  const auto it = fault.config.find("drop_probability");
  if (it == fault.config.end()) {
    return;
  }

  double value = 0.0;
  if (!parse_double(it->second, value)) {
    result.errors.push_back("fault '" + fault.id + "' config 'drop_probability' must be a number");
    return;
  }

  if (value < 0.0 || value > 1.0) {
    result.errors.push_back("fault '" + fault.id +
                            "' config 'drop_probability' must be between 0 and 1");
  }
}

void validate_delay_ms(const FaultConfig& fault, ValidationResult& result) {
  const auto it = fault.config.find("delay_ms");
  if (it == fault.config.end()) {
    return;
  }

  int value = 0;
  if (!parse_int(it->second, value)) {
    result.errors.push_back("fault '" + fault.id + "' config 'delay_ms' must be an integer");
    return;
  }

  if (value < 0) {
    result.errors.push_back("fault '" + fault.id + "' config 'delay_ms' must be >= 0");
  }
}

void validate_fault_keys(const FaultConfig& fault, const std::string& injector_type,
                         ValidationResult& result) {
  for (const auto& [key, value] : fault.config) {
    (void)value;

    if (!is_allowed_config_key(injector_type, key)) {
      result.warnings.push_back("fault '" + fault.id + "' has unknown config key '" + key + "'");
    }
  }
}

void validate_fault_values(const FaultConfig& fault, const std::string& injector_type,
                           ValidationResult& result) {
  validate_drop_probability(fault, result);
  validate_delay_ms(fault, result);

  if (injector_type == "odom") {
    validate_number_key(fault, "x_bias", result);
    validate_number_key(fault, "y_bias", result);
    validate_non_negative_number_key(fault, "x_noise_stddev", result);
    validate_non_negative_number_key(fault, "y_noise_stddev", result);
    validate_number_key(fault, "yaw_bias_deg", result);
    validate_non_negative_number_key(fault, "yaw_noise_stddev_deg", result);
  }

  if (injector_type == "scan") {
    validate_number_key(fault, "range_bias", result);
    validate_non_negative_number_key(fault, "range_noise_stddev", result);
    validate_number_key(fault, "sector_min_deg", result);
    validate_number_key(fault, "sector_max_deg", result);
    validate_sector_value(fault, result);
  }

  if (injector_type == "joint_state") {
    validate_number_key(fault, "velocity_bias", result);
    validate_non_negative_number_key(fault, "velocity_noise_stddev", result);
  }
}

bool is_initially_active(const ScenarioConfig& scenario, const FaultConfig& fault) {
  return std::find(scenario.initially_active_faults.begin(), scenario.initially_active_faults.end(),
                   fault.id) != scenario.initially_active_faults.end();
}

void validate_fault(const ScenarioConfig& scenario, const FaultConfig& fault,
                    const InjectorConfig& injector, ValidationResult& result) {
  const bool active_on_startup = is_initially_active(scenario, fault);
  if (fault.id.empty()) {
    result.errors.push_back("fault.id must not be empty");
  }

  if (fault.injector_id.empty()) {
    result.errors.push_back("fault '" + fault.id + "' injector_id must not be empty");
  }

  if (fault.injector_id != injector.id) {
    result.errors.push_back("fault '" + fault.id + "' targets injector_id '" + fault.injector_id +
                            "' but scenario injector is '" + injector.id + "'");
  }

  if (active_on_startup && fault.start) {
    result.warnings.push_back("fault '" + fault.id +
                              "' has active_on_startup=true and start; start will be ignored");
  }

  if (!active_on_startup && fault.duration && !fault.start) {
    result.warnings.push_back("fault '" + fault.id +
                              "' has duration but no start; duration will be ignored");
  }

  if (fault.start && *fault.start < std::chrono::milliseconds(0)) {
    result.errors.push_back("fault '" + fault.id + "' has negative start time");
  }

  if (fault.duration && *fault.duration < std::chrono::milliseconds(0)) {
    result.errors.push_back("fault '" + fault.id + "' has negative duration");
  }

  validate_fault_keys(fault, injector.type, result);
  validate_fault_values(fault, injector.type, result);
}

}  // namespace

ValidationResult validate_scenario(const ScenarioConfig& scenario) {
  ValidationResult result;

  std::unordered_set<std::string> injector_ids;
  for (const auto& injector : scenario.injectors) {
    if (!injector.id.empty() && !injector_ids.insert(injector.id).second) {
      result.errors.push_back("duplicate injector id: '" + injector.id + "'");
    }

    validate_injector(injector, result);
  }

  for (const auto& fault : scenario.faults) {
    const auto* injector = find_injector(scenario, fault.injector_id);

    if (injector == nullptr) {
      result.errors.push_back("fault '" + fault.id + "' references unknown injector_id '" +
                              fault.injector_id + "'");
      continue;
    }
    validate_fault(scenario, fault, *injector, result);
  }

  return result;
}

}  // namespace ros2_fault_injection