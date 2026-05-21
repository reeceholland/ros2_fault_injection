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

## Fault Types

The supported config keys depend on the injector type.

### Odom

| Key | Meaning |
| --- | --- |
| `x_bias` | Add a constant offset to odometry X position. |
| `y_bias` | Add a constant offset to odometry Y position. |
| `x_noise_stddev` | Add Gaussian noise to odometry X position. |
| `y_noise_stddev` | Add Gaussian noise to odometry Y position. |
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
    active: false
    start: 5.0
    duration: 10.0
    config:
      x_bias: 1.0
      y_bias: -0.25

  - id: scan_noise
    injector_id: scan
    active: true
    config:
      range_noise_stddev: 0.05
```

### Fault Activation Rules

| YAML fields | Behavior |
| --- | --- |
| `active: true` | Fault starts active immediately. |
| `active: false` with `start` | Fault is scheduled to activate after `start` seconds. |
| `duration` with `start` | Fault is deactivated after the duration expires. |
| `active: false` without `start` | Fault is manual-only and can be enabled with the service API. |

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
  details: scheduled, start=5000ms, duration=10000ms, config={x_bias=1.0, y_bias=-0.25}
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

## Events

Fault state changes are published as string events:

```bash
ros2 topic echo /fault_injection/events
```

Events are published when scheduled faults activate/deactivate and when faults are changed through the service API.

## Rover Integration Pattern

For the rover stack, use fault injection as a boundary between raw producer topics and consumer-facing topics:

```text
diff_drive_controller -> /odom_raw -> odom injector -> /odom -> Nav2
lidar or simulator    -> /scan_raw -> scan injector -> /scan -> Nav2/SLAM
Unity motor feedback  -> /platform/motors/feedback_raw -> joint_state injector -> /platform/motors/feedback -> ros2_control
```

This keeps the normal ROS topic names stable for Nav2, SLAM, and controllers while letting fault injection sit in the middle.

## Tests

Run package tests:

```bash
cd /home/reece/fault_injection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select ros2_fault_injection --event-handlers console_direct+
```

Current tests cover scenario validation, scenario parsing, and shared `FaultInjectorBase` behavior.

## Development Notes

- Keep message-specific mutation in typed injectors such as `OdomFaultInjector`, `ScanFaultInjector`, and `JointStateFaultInjector`.
- Keep shared state management in `FaultInjectorBase`.
- Keep ROS service handling thin; put behavior in normal C++ helpers where possible.
- Prefer adding new injector types behind the existing `FaultInjector` interface.
