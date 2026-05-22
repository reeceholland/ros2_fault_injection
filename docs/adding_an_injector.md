# Adding an Injector

An injector is the message-specific part of the framework. It subscribes to a raw input topic, applies active faults, and republishes the result on the consumer-facing output topic.

Use this guide when adding support for a new ROS message type.

## 1. Pick the Injector Type

Choose a short `type` string for YAML scenarios. Existing examples are:

```text
odom
scan
joint_state
imu
```

The type should describe the stream being faulted, not the fault itself. For example, prefer `battery_state` over `battery_voltage_drop`.

## 2. Create the Typed Injector Class

Create a header in:

```text
include/ros2_fault_injection/
```

and a source file in:

```text
src/
```

The class should derive from `FaultInjectorBase`:

```cpp
class BatteryStateFaultInjector : public FaultInjectorBase {
public:
  explicit BatteryStateFaultInjector(rclcpp::Node& node, const InjectorConfig& config);

private:
  void on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
};
```

The constructor should create the subscription and publisher using `config.input_topic`, `config.output_topic`, and `config.qos_depth`.

## 3. Follow the Proxy Pattern

The callback should copy the incoming message, apply common faults, apply message-specific faults, then publish:

```cpp
void BatteryStateFaultInjector::on_battery_state(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (should_drop()) {
    return;
  }

  auto output = *msg;

  apply_voltage_bias(output);
  apply_voltage_noise(output);

  pub_->publish(output);
}
```

If the message type supports delay, follow the existing delayed-message pattern used by odom, scan, joint state, and IMU injectors.

## 4. Add Fault Config Keys

Add the new keys to `fault_config_schema` so validation, services, and docs agree on what the injector accepts.

Example:

```cpp
{"battery_state",
 {"voltage_bias", "voltage_noise_stddev", "drop_probability", "delay_ms"}}
```

Use the common keys when they make sense:

```text
drop_probability
delay_ms
```

Add value validation in `scenario_validator` for keys with constraints, such as non-negative standard deviations or probabilities in the range `0.0` to `1.0`.

## 5. Register the Injector in the Factory

Update `FaultInjectorFactory::create()` so the YAML type creates the new class:

```cpp
if (config.type == "battery_state") {
  return std::make_shared<BatteryStateFaultInjector>(node_, config);
}
```

Also include the new header in `fault_injector_factory.cpp`.

## 6. Update CMake

Add the new source file to the core library:

```cmake
add_library(${PROJECT_NAME}_core
  ...
  src/battery_state_fault_injector.cpp
)
```

If the injector needs a new ROS package dependency, add it to:

- `CMakeLists.txt`
- `package.xml`

Most sensor message injectors use `sensor_msgs`, which is already present.

## 7. Add Example YAML

Add either a focused config file in `config/` or extend `multi_injector_faults.yaml`:

```yaml
injectors:
  - id: battery
    type: battery_state
    input_topic: /battery_state_raw
    output_topic: /battery_state
    qos_depth: 10

faults:
  - id: battery_voltage_bias
    injector_id: battery
    active_on_startup: false
    config:
      voltage_bias: -1.5
```

## 8. Add Tests

At minimum, add tests for:

- schema accepts the new config keys
- validator rejects invalid values
- the injector mutates messages as expected
- service config updates work for the new keys, if they are runtime configurable

Prefer small focused tests. Use fake injectors for service and scheduler behavior, and typed injector tests only when checking message mutation.

## 9. Update Documentation

Update:

- `README.md`
- this guide if the process changes
- Doxygen comments on the new public header

Then regenerate API docs:

```bash
cd /home/reece/fault_injection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection --cmake-target generate_doxygen_api
```

## Checklist

- New injector header and source file added
- Factory creates the injector
- CMake builds the source file
- `package.xml` has any new dependencies
- config schema includes new keys
- validator checks key values where needed
- example YAML added
- README updated
- tests added
- Doxygen docs regenerate successfully
