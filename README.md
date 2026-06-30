# ros2_fault_injection

[![CI](https://github.com/reeceholland/ros2_fault_injection/actions/workflows/ci.yml/badge.svg)](https://github.com/reeceholland/ros2_fault_injection/actions/workflows/ci.yml)
[![Documentation Status](https://readthedocs.org/projects/ros2-fault-injection/badge/?version=latest)](https://ros2-fault-injection.readthedocs.io/en/latest/?badge=latest)

`ros2_fault_injection` is a C++ ROS 2 framework for inserting controlled faults into robot topics, transforms, and services, then checking expected fault outcomes with scenario assertions.

Documentation is available at [ros2-fault-injection.readthedocs.io](https://ros2-fault-injection.readthedocs.io/).

The package is designed around proxy injectors. For topics, a real publisher is remapped to a `*_raw` topic, the injector subscribes to that raw topic, applies configured faults, and republishes on the normal topic used by the rest of the ROS graph.

```text
producer -> /topic_raw -> fault injector -> /topic -> consumers
```

Service injectors use the same proxy idea:

```text
client -> /service -> fault injector -> /service_raw -> real server
```

This keeps the framework understandable and testable while covering both data streams and command/control interactions. Scenarios can also include assertions that watch fault events and publish pass/fail results for expected outcomes.

Injectors are discovered through `pluginlib`. The built-in injector types are registered in `fault_injector_plugins.xml`, and additional packages can provide their own `FaultInjectorPlugin` wrappers without editing the core factory.

Each injector owns its editable fault config schema. Scenario validation, runtime config updates, and RViz schema display all use the same `FaultConfigField` metadata, so config keys and values are checked consistently whether they come from YAML or from `/fault_injection/set_fault_config`.

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

A scenario defines one or more injectors, the faults assigned to them, and optional assertions for expected outcomes.

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

assertions:
  - id: odom_bias_activates
    type: fault_event
    fault_id: odom_bias
    state: active
    within: 6.0

  - id: odom_bias_deactivates
    type: fault_event
    fault_id: odom_bias
    state: inactive
    within: 17.0

  - id: odom_stays_above_10hz
    type: topic_hz
    topic: /odom
    message_type: nav_msgs/msg/Odometry
    min_hz: 10.0
    window: 3.0
    within: 8.0
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

### Assertions

Assertions describe expected outcomes from a scenario. They do not inject faults themselves; they observe framework events and publish pass/fail results.

Currently supported assertion types:

| Type | Required fields | Behavior |
| --- | --- | --- |
| `fault_event` | `id`, `type`, `fault_id`, `state` | Passes when the named fault publishes the requested state. |
| `topic_hz` | `id`, `type`, `topic`, `message_type`, `min_hz`, `window` | Passes when the topic publishes at or above `min_hz` over the configured window. |

Optional assertion fields:

| Field | Meaning |
| --- | --- |
| `within` | Fails the assertion if the expected condition is not observed within this many seconds. |

`fault_event` assertions support `state: active` and `state: inactive`. `topic_hz` assertions use ROS 2 generic subscriptions, so the runner counts serialized messages without needing a typed callback for every message type. The `message_type` field must use the ROS interface name, such as `nav_msgs/msg/Odometry` or `sensor_msgs/msg/LaserScan`.

The validator rejects assertions that reference unknown fault IDs, unsupported states, missing topic rate fields, duplicate assertion IDs, or negative timing values.

## Build

From the workspace root:

```bash
cd /path/to/fault_injection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection
source install/setup.bash
```

## Run

There are two common ways to run the framework.

### Operator/RViz Mode

Use `fault_injector_node` when you want the framework to stay alive for RViz, service calls, manual fault control, scenario reloads, and report requests.

```bash
ros2 launch ros2_fault_injection fault_injector.launch.py \
  scenario_file:=/path/to/fault_injection_ws/install/ros2_fault_injection/share/ros2_fault_injection/config/multi_injector_faults.yaml
```

This mode keeps the `/fault_injection/*` services available. RViz can request the current scenario, reload the scenario, edit fault config values, and request an in-memory markdown report through `/fault_injection/request_report`.

When using `--symlink-install`, editing files in `config/` is usually reflected through the install space symlink. If in doubt, rebuild and source the workspace again.

### Headless Scenario Runner Mode

Use `fault_scenario_runner_node` for CI, smoke tests, or terminal-driven runs where the process should finish with a pass/fail exit code.

```bash
ros2 run ros2_fault_injection fault_scenario_runner_node \
  --ros-args \
  -p scenario_file:=/path/to/fault_injection_ws/src/ros2_fault_injection/config/multi_injector_faults.yaml \
  -p timeout:=30.0 \
  -p report_file:=/tmp/fault_injection_report.md
```

The runner starts the scenario, waits until all assertions pass/fail or the timeout expires, writes a markdown report to `report_file` when provided, and exits with:

| Exit code | Meaning |
| --- | --- |
| `0` | All assertions passed. |
| `1` | At least one assertion failed, the scenario timed out, config loading failed, or ROS shut down before completion. |

Unlike `fault_injector_node`, the scenario runner exits when the run is complete. That means RViz cannot request services from it after completion. Use operator/RViz mode when you want to inspect or request reports interactively.

Reports are only written to disk by `fault_scenario_runner_node` when `report_file` is set. The long-running injector node returns report markdown through `/fault_injection/request_report`; it does not save a report file by default.

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

### Get Fault Config Schema

```bash
ros2 service call /fault_injection/get_fault_schema ros2_fault_injection/srv/GetFaultSchema \
  "{fault_id: odom_bias}"
```

Returns the editable config keys for the selected fault's injector type:

```yaml
success: true
message: retrieved schema for fault_id: odom_bias
injector_id: odom
injector_type: odom
keys:
- delay_ms
- drop_probability
- x_bias
- x_noise_stddev
- y_bias
- y_noise_stddev
```

Use this when building UI controls or scripts that need to know which config keys are valid before calling `SetFaultConfig`.

### Get Fault Config

```bash
ros2 service call /fault_injection/get_fault_config ros2_fault_injection/srv/GetFaultConfig \
  "{fault_id: odom_bias}"
```

Returns the current runtime config for one fault:

```yaml
success: true
message: retrieved config for fault_id: odom_bias
injector_id: odom
injector_type: odom
keys:
- drop_probability
- x_bias
values:
- '0.0'
- '1.0'
```

`keys` and `values` are parallel arrays: `keys[i]` matches `values[i]`. This shape is used because ROS 2 service definitions do not support map fields directly.

### Update A Fault Config Value

```bash
ros2 service call /fault_injection/set_fault_config ros2_fault_injection/srv/SetFaultConfig \
  "{fault_id: odom_bias, key: x_bias, value: '2.0'}"
```

This updates one entry inside the fault's `config` map at runtime. It does not change top-level scheduling fields such as `start`, `duration`, `active_on_startup`, `id`, or `injector_id`.

The key and value must be valid for that injector's fault config schema. For example, `x_bias` is valid for an `odom` fault, while `range_bias` is valid for a `scan` fault. Numeric fields, booleans, probabilities, and non-negative values are validated before the stored config is changed.

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

## Assertions

Assertion results are published as typed events:

```bash
ros2 topic echo /fault_injection/assertion_events
```

The assertion event message is `ros2_fault_injection/msg/AssertionEvent`:

```text
builtin_interfaces/Time stamp
string assertion_id
string assertion_type
string state
string message
```

Common assertion states:

| State | Meaning |
| --- | --- |
| `passed` | The expected condition was observed. |
| `failed` | The expected condition was not observed before its deadline, or otherwise failed. |

Assertions are useful for automated scenario checks. For example, a scheduled `odom_bias` fault can have one assertion that expects an `active` event shortly after startup and another that expects an `inactive` event after its configured duration.

The scenario monitor also publishes a continuously updated summary:

```bash
ros2 topic echo /fault_injection/scenario_status
```

The status message is `ros2_fault_injection/msg/ScenarioStatus`:

```text
builtin_interfaces/Time stamp
string state
uint32 pending_count
uint32 passed_count
uint32 failed_count
string[] failed_assertion_ids
string[] failed_messages
```

Example running status:

```yaml
state: running
pending_count: 2
passed_count: 1
failed_count: 0
failed_assertion_ids: []
failed_messages: []
```

If any assertion fails, the scenario state becomes `failed` and the failed assertion IDs and messages are included.

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

Current tests cover scenario validation, scenario parsing, config schema, scheduler behavior, shared `FaultInjectorBase` behavior, service/event behavior in `FaultServiceManager`, fault event assertions, topic Hz assertions, scenario status monitoring, odom covariance behavior, trigger service faults, TF transform faults, and a launch/service integration path for runtime config reads and updates.

## Development Notes

- Keep message-specific mutation in typed injectors such as `OdomFaultInjector`, `ScanFaultInjector`, and `JointStateFaultInjector`.
- Keep shared state management in `FaultInjectorBase`.
- Keep ROS service handling thin; put behavior in normal C++ helpers where possible.
- Prefer adding new injector types behind the existing `FaultInjector` interface.
