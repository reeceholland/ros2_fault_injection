# ros2_fault_injection

[![CI](https://github.com/reeceholland/ros2_fault_injection/actions/workflows/ci.yml/badge.svg)](https://github.com/reeceholland/ros2_fault_injection/actions/workflows/ci.yml)
[![Documentation Status](https://readthedocs.org/projects/ros2-fault-injection/badge/?version=latest)](https://ros2-fault-injection.readthedocs.io/en/latest/?badge=latest)

`ros2_fault_injection` is a C++ ROS 2 framework for inserting controlled faults into robot data streams.

Documentation is available at [ros2-fault-injection.readthedocs.io](https://ros2-fault-injection.readthedocs.io/).

The package is designed around proxy injectors. For topics, a real publisher is remapped to a `*_raw` topic, the injector subscribes to that raw topic, applies configured faults, and republishes on the normal topic used by the rest of the ROS graph.

```text
producer -> /topic_raw -> fault injector -> /topic -> consumers
```

Service injectors use the same proxy idea:

```text
client -> /service -> fault injector -> /service_raw -> real server
```

This keeps the framework understandable and testable while covering both data streams and command/control interactions.

Injectors are discovered through `pluginlib`. The built-in injector types are registered in `fault_injector_plugins.xml`, and additional packages can provide their own `FaultInjectorPlugin` wrappers without editing the core factory.

## Example Plugin Package

An external example plugin package is available here:

[ros2_fault_injection_example_plugins](https://github.com/reeceholland/ros2_fault_injection_example_plugins)

It demonstrates how to add a custom injector type in a separate ROS 2 package using `pluginlib`, without modifying the core `ros2_fault_injection` package.

## Supported Injectors

| Injector type | Message type | Example input | Example output |
| --- | --- | --- | --- |
| `odom` | `nav_msgs/msg/Odometry` | `/odom_raw` | `/odom` |
| `scan` | `sensor_msgs/msg/LaserScan` | `/scan_raw` | `/scan` |
| `joint_state` | `sensor_msgs/msg/JointState` | `/platform/motors/feedback_raw` | `/platform/motors/feedback` |
| `imu` | `sensor_msgs/msg/Imu` | `/sensors/imu_raw` | `/sensors/imu` |
| `tf` | `tf2_msgs/msg/TFMessage` | `/tf_raw` | `/tf` |
| `trigger_service` | `std_srvs/srv/Trigger` | `/enable_motors_raw` | `/enable_motors` |

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
| `pose_covariance_scale` | Multiply every pose covariance entry by this value. |
| `pose_covariance_floor` | Clamp every pose covariance entry to at least this value after scaling. |
| `twist_covariance_scale` | Multiply every twist covariance entry by this value. |
| `twist_covariance_floor` | Clamp every twist covariance entry to at least this value after scaling. |
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

### TF

TF faults target individual transforms inside a `tf2_msgs/msg/TFMessage`. Each TF fault must include `parent_frame` and `child_frame` so faults do not accidentally apply to every transform in the tree.

| Key | Meaning |
| --- | --- |
| `parent_frame` | Required parent frame to match against `transform.header.frame_id`. |
| `child_frame` | Required child frame to match against `transform.child_frame_id`. |
| `x_bias` | Add a translation offset in metres to the matching transform's X value. |
| `y_bias` | Add a translation offset in metres to the matching transform's Y value. |
| `z_bias` | Add a translation offset in metres to the matching transform's Z value. |
| `roll_bias_deg` | Add a roll offset, in degrees, to the matching transform rotation. |
| `pitch_bias_deg` | Add a pitch offset, in degrees, to the matching transform rotation. |
| `yaw_bias_deg` | Add a yaw offset, in degrees, to the matching transform rotation. |
| `drop_probability` | Randomly drop matching transforms. Range: `0.0` to `1.0`. |
| `delay_ms` | Delay TF messages by this many milliseconds before republishing. |

When using TF injection, remap the original TF publisher to `/tf_raw` and let the injector publish the consumer-facing `/tf`. Avoid leaving multiple publishers producing the same `parent_frame -> child_frame` transform on `/tf`, because TF consumers may receive conflicting transforms.

### Trigger Service

| Key | Meaning |
| --- | --- |
| `force_failure` | Return `success: false` from the proxy service without calling the target service. |
| `failure_message` | Response message used when `force_failure` is active. |
| `delay_ms` | Delay the service response by this many milliseconds. |

## Scenario YAML

A scenario defines one or more injectors and the faults assigned to them.

```yaml
injectors:
  - id: odom
    type: odom
    topic:
      input_topic: /odom_raw
      output_topic: /odom
      qos_depth: 10

  - id: scan
    type: scan
    topic:
      input_topic: /scan_raw
      output_topic: /scan
      qos_depth: 10

  - id: enable_motors
    type: trigger_service
    trigger_service:
      proxy_service: /enable_motors
      target_service: /enable_motors_raw

  - id: tf_odom_base
    type: tf
    topic:
      input_topic: /tf_raw
      output_topic: /tf
      qos_depth: 50

faults:
  - id: odom_bias
    injector_id: odom
    active_on_startup: false
    start: 5.0
    duration: 10.0
    config:
      x_bias: 1.0
      y_bias: -0.25

  - id: odom_covariance_inflation
    injector_id: odom
    active_on_startup: false
    config:
      pose_covariance_scale: 10.0
      pose_covariance_floor: 0.05
      twist_covariance_scale: 10.0
      twist_covariance_floor: 0.05

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

  - id: enable_motors_failure
    injector_id: enable_motors
    active_on_startup: false
    config:
      force_failure: true
      failure_message: "Injected enable_motors failure"

  - id: tf_yaw_bias
    injector_id: tf_odom_base
    active_on_startup: false
    config:
      parent_frame: odom
      child_frame: base_link
      yaw_bias_deg: 10.0
```

### Fault Activation Rules

| YAML fields | Behavior |
| --- | --- |
| `active_on_startup: true` | Fault is active as soon as the framework starts. |
| `active_on_startup: false` with `start` | Fault is scheduled to activate after `start` seconds. |
| `duration` with `start` | Fault is deactivated after the duration expires. |
| `active_on_startup: false` without `start` | Fault starts inactive and can be enabled with the service API. |

`start` and `duration` are written in seconds in YAML and stored internally as milliseconds.

Topic injectors use a `topic` block containing `input_topic`, `output_topic`, and `qos_depth`. Trigger service injectors use a `trigger_service` block containing `proxy_service` and `target_service`.

## Build

From the workspace root:

```bash
cd /path/to/fault_injection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection
source install/setup.bash
```

## Run

Launch the injector with a scenario file:

```bash
ros2 launch ros2_fault_injection fault_injector.launch.py \
  scenario_file:=/path/to/fault_injection_ws/install/ros2_fault_injection/share/ros2_fault_injection/config/multi_injector_faults.yaml
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
  details: "scheduled, start=5000ms, duration=10000ms, config={x_bias=1.0, y_bias=-0.25}"
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
details: config_update={x_bias=2.0}
```

### Reload The Scenario File

```bash
ros2 service call /fault_injection/reload_scenario ros2_fault_injection/srv/ReloadScenario {}
```

Reloads the same scenario file that was passed to `fault_injector.launch.py`.

The reload path is intentionally conservative: it can update fault definitions, config values,
startup state, and schedules, but it cannot change the running injector layout. Injector IDs,
injector types, topic endpoints, service endpoints, and QoS depth must stay the same. If validation
or compatibility checks fail, the currently running scenario is left unchanged.

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
TF publisher         -> /tf_raw -> tf injector -> /tf -> TF consumers
client               -> /enable_motors -> trigger_service injector -> /enable_motors_raw -> real server
```

This keeps the normal ROS topic and service names stable for Nav2, SLAM, controllers, and callers while letting fault injection sit in the middle.

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

For full walkthroughs, see:

- `docs/adding_an_injector.md`
- `docs/adding_a_fault_type.md`


## API Documentation

API docs are generated with Doxygen when Doxygen is installed:

```bash
cd /path/to/fault_injection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection --cmake-target generate_doxygen_api
```

The generated HTML is written under:

```text
/path/to/fault_injection_ws/build/ros2_fault_injection/docs/html/index.html
```

The docs target is optional. Normal package builds and tests do not require Doxygen.

## Tests

Run package tests:

```bash
cd /path/to/fault_injection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select ros2_fault_injection --event-handlers console_direct+
```

Current tests cover scenario validation, scenario parsing, config schema, scheduler behavior, shared `FaultInjectorBase` behavior, service/event behavior in `FaultServiceManager`, odom covariance behavior, trigger service faults, and TF transform faults.

## Development Notes

- Keep message-specific mutation in typed injectors such as `OdomFaultInjector`, `ScanFaultInjector`, and `JointStateFaultInjector`.
- Keep shared state management in `FaultInjectorBase`.
- Keep ROS service handling thin; put behavior in normal C++ helpers where possible.
- Prefer adding new injector types behind the existing `FaultInjector` interface.
