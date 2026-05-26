# Adding an Injector

An injector is the transport-specific part of the framework. Topic injectors subscribe to a raw input topic, apply active faults, and republish the result on the consumer-facing output topic. Service injectors proxy a ROS service and can alter the response path.

Use this guide when adding support for a new ROS message type, service type, or other injectable ROS interface.

## 1. Pick the Injector Type

Choose a short `type` string for YAML scenarios. Existing examples are:

```text
odom
scan
joint_state
imu
tf
trigger_service
```

The type should describe the ROS interface being faulted, not the fault itself. For example, prefer `battery_state` over `battery_voltage_drop`.

## 2. Create the Typed Injector Class

Create a header in:

```text
include/ros2_fault_injection/injectors/
```

and a source file in:

```text
src/injectors/
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

The constructor should create the subscription and publisher using `config.topic->input_topic`, `config.topic->output_topic`, and `config.topic->qos_depth`.

For service injectors, use the service-specific configuration instead of topic fields. For example, a trigger-service injector uses `config.trigger_service->proxy_service` and `config.trigger_service->target_service`.

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

## 5. Add a Plugin Wrapper

Injectors are discovered through `pluginlib`. The factory does not need a new `if` branch for each injector type.

For a built-in injector, add a small wrapper class to:

```text
include/ros2_fault_injection/core/builtin_fault_injector_plugin.hpp
src/core/builtin_fault_injector_plugin.cpp
```

The wrapper is default-constructible for `pluginlib`, then creates the real injector when the framework passes in the node and injector configuration:

```cpp
class BatteryStateFaultInjectorPlugin : public FaultInjectorPlugin {
public:
  std::string type() const override { return "battery_state"; }

  std::shared_ptr<FaultInjector> create(
    rclcpp::Node& node, const InjectorConfig& config) override {
    return std::make_shared<BatteryStateFaultInjector>(node, config);
  }
};
```

Export the wrapper in the plugin source file:

```cpp
PLUGINLIB_EXPORT_CLASS(
  ros2_fault_injection::BatteryStateFaultInjectorPlugin,
  ros2_fault_injection::FaultInjectorPlugin)
```

Then register it in `fault_injector_plugins.xml`:

```xml
<class
  name="ros2_fault_injection/BatteryStateFaultInjectorPlugin"
  type="ros2_fault_injection::BatteryStateFaultInjectorPlugin"
  base_class_type="ros2_fault_injection::FaultInjectorPlugin">
  <description>Battery state fault injector.</description>
</class>
```

External packages can provide their own wrappers too. They should depend on `ros2_fault_injection`, implement `FaultInjectorPlugin`, and export their own plugin XML with:

```cmake
pluginlib_export_plugin_description_file(ros2_fault_injection your_plugins.xml)
```

## 6. Update CMake

Add the new source file to the core library:

```cmake
add_library(${PROJECT_NAME}_core
  ...
  src/injectors/battery_state_fault_injector.cpp
)
```

If you add the plugin wrapper to a new source file, add that source file to the core library too.

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
    topic:
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
- `FaultInjectorFactory` can create the injector through pluginlib
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
- Plugin wrapper added
- Plugin XML exports the wrapper
- Factory creates the injector through pluginlib
- CMake builds the source file
- `package.xml` has any new dependencies
- config schema includes new keys
- validator checks key values where needed
- example YAML added
- README updated
- tests added
- Doxygen docs regenerate successfully
