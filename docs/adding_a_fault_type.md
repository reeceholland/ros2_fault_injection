# Adding a Fault Type

A fault type is one new behavior inside an existing injector. Add a fault type when the message stream is already supported, but you want to mutate it in a new way.

For example, odometry covariance inflation belongs in the existing `odom` injector because it still operates on `nav_msgs/msg/Odometry`.

## 1. Pick the Config Key

Choose a key that describes the parameter, not the implementation detail.

Good examples:

```text
pose_covariance_scale
pose_covariance_floor
yaw_bias_deg
range_noise_stddev
```

Use units in the key when the unit is not obvious:

```text
yaw_bias_deg
delay_ms
```

## 2. Add the Key to the Schema

Update `fault_config_schema` so the key is accepted for the correct injector type.

Example for odom:

```cpp
const std::unordered_set<std::string> kOdomKeys = {
    "drop_probability",
    "delay_ms",
    "pose_covariance_scale",
    "pose_covariance_floor",
};
```

This keeps YAML validation, runtime config updates, and documentation aligned.

## 3. Add Value Validation

Update `scenario_validator` when the key has constraints.

Typical rules:

- standard deviations must be non-negative
- probabilities must be between `0.0` and `1.0`
- covariance scale/floor values must be non-negative
- enum-like strings should be checked against known values

For numeric non-negative keys, use the existing validator helper pattern:

```cpp
validate_non_negative_number_key(fault, "pose_covariance_scale", result);
```

## 4. Implement the Mutation

Add a small method to the typed injector and call it from the message callback.

For odom covariance:

```cpp
void OdomFaultInjector::apply_covariance_scale(nav_msgs::msg::Odometry& msg) {
  const double pose_scale = active_max_double("pose_covariance_scale", 1.0);
  const double pose_floor = active_max_double("pose_covariance_floor", 0.0);

  if (pose_scale != 1.0 || pose_floor > 0.0) {
    for (auto& value : msg.pose.covariance) {
      value = std::max(value * pose_scale, pose_floor);
    }
  }
}
```

Use helpers from `FaultInjectorBase` where possible:

- `active_max_double(...)` when the strongest active value should win
- `active_sum_double(...)` when active faults should stack additively
- `active_max_int(...)` for integer configuration
- `should_drop()` for common message dropping
- `active_delay()` for common message delay

## 5. Decide How Faults Combine

Be explicit about how multiple active faults interact.

Common patterns:

```text
bias values stack additively
noise values can use max or sum depending on desired behavior
scale/floor values usually use max
drop_probability usually uses max
delay_ms usually uses max
```

Document the choice in a short code comment when it is not obvious.

## 6. Add YAML Examples

Add a focused example in `config/` or extend `multi_injector_faults.yaml`.

Example:

```yaml
- id: odom_covariance_inflation
  injector_id: odom
  active_on_startup: false
  config:
    pose_covariance_scale: 10.0
    pose_covariance_floor: 0.05
    twist_covariance_scale: 10.0
    twist_covariance_floor: 0.05
```

## 7. Add Tests

Test the smallest behavior that proves the new fault works.

For a typed injector, prefer publishing a raw message and checking the output topic:

```text
publish /odom_raw -> injector -> receive /odom
```

For schema and validation:

- schema accepts the new key
- validator rejects invalid values
- service config updates accept the new key when appropriate

## 8. Update Documentation

Update:

- `README.md` fault key tables
- example YAML files
- Doxygen comments if the public header changed
- this guide if the process changed

Then regenerate docs:

```bash
cd /home/reece/fault_injection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection --cmake-target generate_doxygen_api
```

## Checklist

- Config key added to `fault_config_schema`
- Invalid values rejected in `scenario_validator`
- Typed injector applies the fault
- Callback calls the new mutation method
- Example YAML added
- README updated
- Tests added or updated
- Doxygen docs regenerate cleanly
