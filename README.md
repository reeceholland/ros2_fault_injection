# ros2_fault_injection

`ros2_fault_injection` is a C++ ROS 2 framework for inserting controlled faults into robot data streams.

The package is designed around proxy injectors. A real publisher is remapped to a `*_raw` topic, the injector subscribes to that raw topic, applies configured faults, and republishes on the normal topic used by the rest of the ROS graph.

```text
producer -> /topic_raw -> fault injector -> /topic -> consumers
```

This keeps the framework understandable and testable without doing DDS packet manipulation.

## Supported Injectors

| Injector type | Message type | Example input | Example output |
| --- | --- | --- | --- |
| `odom` | `nav_msgs/msg/Odometry` | `/odom_raw` | `/odom` |
| `scan` | `sensor_msgs/msg/LaserScan` | `/scan_raw` | `/scan` |
| `joint_state` | `sensor_msgs/msg/JointState` | `/platform/motors/feedback_raw` | `/platform/motors/feedback` |
| `imu` | `sensor_msgs/msg/Imu` | `/sensors/imu_raw` | `/sensors/imu` |

## Fault Types

The supported config keys depend on the injector type.

### Odom

| Key | Meaning |
| --- | --- |
| `x_bias` | Add a constant offset to odometry X position. |
| `y_bias` | Add a constant offset to odometry Y position. |
| `x_noise_stddev` | Add Gaussian noise to odometry X position. |
| `y_noise_stddev` | Add Gaussian noise to odometry Y position. |
| `yaw_bias_deg` | Add a constant yaw offset, in degrees. |
| `yaw_noise_stddev_deg` | Add Gaussian noise to yaw, in degrees. |
| `drop_probability` | Randomly drop odometry messages. Range: `0.0` to `1.0`. |
| `delay_ms` | Delay odometry messages by this many milliseconds. |

### Scan

| Key | Meaning |
| --- | --- |
| `range_bias` | Add a constant offset to laser scan ranges. |
| `range_noise_stddev` | Add Gaussian noise to laser scan ranges. |
| `sector_min_deg` | Start angle for a sector dropout, in degrees. |
| `sector_max_deg` | End angle for a sector dropout, in degrees. |
| `sector_value` | Replacement range for the sector. Supports numbers, `inf`, `-inf`, and `nan`. Defaults to `inf`. |
| `drop_probability` | Randomly drop scan messages. Range: `0.0` to `1.0`. |
| `delay_ms` | Delay scan messages by this many milliseconds. |

### Joint State

| Key | Meaning |
| --- | --- |
| `velocity_bias` | Add a constant offset to every joint velocity. |
| `velocity_noise_stddev` | Add Gaussian noise to every joint velocity. |
| `drop_probability` | Randomly drop joint state messages. Range: `0.0` to `1.0`. |
| `delay_ms` | Delay joint state messages by this many milliseconds. |

### IMU

| Key | Meaning |
| --- | --- |
| `angular_velocity_z_bias` | Add a constant yaw-rate offset in `rad/s`. |
| `angular_velocity_z_noise_stddev` | Add Gaussian yaw-rate noise in `rad/s`. |
| `linear_acceleration_x_bias` | Add a constant X acceleration offset in `m/s^2`. |
| `linear_acceleration_y_bias` | Add a constant Y acceleration offset in `m/s^2`. |
| `linear_acceleration_z_bias` | Add a constant Z acceleration offset in `m/s^2`. |
| `linear_acceleration_x_noise_stddev` | Add Gaussian X acceleration noise in `m/s^2`. |
| `linear_acceleration_y_noise_stddev` | Add Gaussian Y acceleration noise in `m/s^2`. |
| `linear_acceleration_z_noise_stddev` | Add Gaussian Z acceleration noise in `m/s^2`. |
| `drop_probability` | Randomly drop IMU messages. Range: `0.0` to `1.0`. |
| `delay_ms` | Delay IMU messages by this many milliseconds. |

## Scenario YAML

A scenario defines one or more injectors and the faults assigned to them.

```yaml
injectors:
  - id: odom
    type: odom
    input_topic: /odom_raw
    output_topic: /odom
    qos_depth: 10

  - id: scan
    type: scan
    input_topic: /scan_raw
    output_topic: /scan
    qos_depth: 10

faults:
  - id: odom_bias
    injector_id: odom
    active_on_startup: false
    start: 5.0
    duration: 10.0
    config:
      x_bias: 1.0
      y_bias: -0.25

  - id: scan_noise
    injector_id: scan
    active_on_startup: true
    config:
      range_noise_stddev: 0.05

  - id: imu_accel_noise
    injector_id: imu
    active_on_startup: false
    config:
      linear_acceleration_x_noise_stddev: 0.15
      linear_acceleration_y_noise_stddev: 0.15
      linear_acceleration_z_noise_stddev: 0.05
```

### Fault Activation Rules

| YAML fields | Behavior |
| --- | --- |
| `active_on_startup: true` | Fault is active as soon as the framework starts. |
| `active_on_startup: false` with `start` | Fault is scheduled to activate after `start` seconds. |
| `duration` with `start` | Fault is deactivated after the duration expires. |
| `active_on_startup: false` without `start` | Fault starts inactive and can be enabled with the service API. |

`start` and `duration` are written in seconds in YAML and stored internally as milliseconds.

## Build

From the workspace root:

```bash
cd /home/reece/fault_injection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection
source install/setup.bash
```

## Run

Launch the injector with a scenario file:

```bash
ros2 launch ros2_fault_injection fault_injector.launch.py \
  scenario_file:=/home/reece/fault_injection_ws/install/ros2_fault_injection/share/ros2_fault_injection/config/multi_injector_faults.yaml
```

When using `--symlink-install`, editing files in `config/` is usually reflected through the install space symlink. If in doubt, rebuild and source the workspace again.

## Runtime Services

### List Faults

```bash
ros2 service call /fault_injection/list_faults ros2_fault_injection/srv/ListFaults {}
```

Returns all known fault IDs and the currently active fault IDs.

### Get Fault Status

```bash
ros2 service call /fault_injection/get_fault_status ros2_fault_injection/srv/GetFaultStatus {}
```

Returns structured status records:

```yaml
faults:
- fault_id: odom_bias
  injector_id: odom
  state: inactive
  details: "scheduled, start: 5000ms, duration: 10000ms, config_keys={x_bias=1.0, y_bias=-0.25}"
```

### Activate Or Deactivate A Fault

Activate:

```bash
ros2 service call /fault_injection/set_fault_state ros2_fault_injection/srv/SetFaultState \
  "{fault_id: odom_bias, active: true}"
```

Deactivate:

```bash
ros2 service call /fault_injection/set_fault_state ros2_fault_injection/srv/SetFaultState \
  "{fault_id: odom_bias, active: false}"
```

Successful state changes publish a `FaultEvent` with `state` set to `active` or `inactive` and `source` set to `manual`.

### Update A Fault Config Value

```bash
ros2 service call /fault_injection/set_fault_config ros2_fault_injection/srv/SetFaultConfig \
  "{fault_id: odom_bias, key: x_bias, value: '2.0'}"
```

This updates one entry inside the fault's `config` map at runtime. It does not change top-level scheduling fields such as `start`, `duration`, `active_on_startup`, `id`, or `injector_id`.

The key must be valid for that injector's fault config schema. For example, `x_bias` is valid for an `odom` fault, while `range_bias` is valid for a `scan` fault.

Successful config updates publish a `FaultEvent` with:

```yaml
state: config_updated
source: manual
details: updated config key 'x_bias' to '2.0'
```

## Events

Fault changes are published as typed events:

```bash
ros2 topic echo /fault_injection/events
```

The event message is `ros2_fault_injection/msg/FaultEvent`:

```text
builtin_interfaces/Time stamp
string fault_id
string injector_id
string state
string source
string details
```

Events are published when scheduled faults activate/deactivate and when faults are changed through the service API.

## Rover Integration Pattern

For the rover stack, use fault injection as a boundary between raw producer topics and consumer-facing topics:

```text
diff_drive_controller -> /odom_raw -> odom injector -> /odom -> Nav2
lidar or simulator    -> /scan_raw -> scan injector -> /scan -> Nav2/SLAM
Unity motor feedback  -> /platform/motors/feedback_raw -> joint_state injector -> /platform/motors/feedback -> ros2_control
Unity or IMU driver  -> /sensors/imu_raw -> imu injector -> /sensors/imu -> consumers
```

This keeps the normal ROS topic names stable for Nav2, SLAM, and controllers while letting fault injection sit in the middle.

## Config Schema

Allowed fault config keys are centralized in `fault_config_schema`.

When adding a new fault config key, update:

- `fault_config_schema` so the key is recognized for the correct injector type
- `scenario_validator` if the key needs value checks such as numeric range or non-negative validation
- the typed injector implementation that applies the behavior
- the example YAML files in `config/`
- this README
- tests for the schema, validator, or injector behavior as appropriate

Keeping config-key ownership centralized avoids drift between validation and runtime behavior.

## Tests

Run package tests:

```bash
cd /home/reece/fault_injection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select ros2_fault_injection --event-handlers console_direct+
```

Current tests cover scenario validation, scenario parsing, config schema, scheduler behavior, shared `FaultInjectorBase` behavior, and service/event behavior in `FaultServiceManager`.

## Development Notes

- Keep message-specific mutation in typed injectors such as `OdomFaultInjector`, `ScanFaultInjector`, and `JointStateFaultInjector`.
- Keep shared state management in `FaultInjectorBase`.
- Keep ROS service handling thin; put behavior in normal C++ helpers where possible.
- Prefer adding new injector types behind the existing `FaultInjector` interface.
