# Fault Configuration

Scenarios are YAML files that describe one or more injectors, the faults each injector can apply, and optional assertions for expected outcomes.

At runtime, each fault is registered against an `injector_id`. The injector type decides which keys are valid inside that fault's `config` map.

Each injector exposes a config schema made of `FaultConfigField` entries. Scenario loading and runtime config updates both use that schema, so YAML files and `/fault_injection/set_fault_config` follow the same key and value validation rules. The schema also feeds `/fault_injection/get_fault_schema`, which UI tools can use to display field types, descriptions, defaults, and numeric limits.

## Common Fault Fields

Every fault supports these top-level fields:

| Field | Type | Description |
| --- | --- | --- |
| `id` | string | Unique fault name used by services, logs, and events. |
| `injector_id` | string | ID of the injector that owns this fault. |
| `active_on_startup` | bool | Starts the fault active as soon as the scenario is loaded. |
| `start` | seconds | Schedules the fault to activate after this many seconds. |
| `duration` | seconds | Schedules the fault to deactivate after this many seconds. Must be non-negative. |
| `config` | map | Injector-specific fault parameters. |

Faults without `active_on_startup` or `start` remain inactive until manually activated through `/fault_injection/set_fault_state`.

## Odometry Faults

Injector type: `odom`

Message type: `nav_msgs/msg/Odometry`

| Config Key | Description |
| --- | --- |
| `drop_probability` | Probability from `0.0` to `1.0` that an odom message is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `x_bias` | Adds a fixed offset to `pose.pose.position.x`. |
| `y_bias` | Adds a fixed offset to `pose.pose.position.y`. |
| `x_noise_stddev` | Adds Gaussian noise to `pose.pose.position.x`. |
| `y_noise_stddev` | Adds Gaussian noise to `pose.pose.position.y`. |
| `yaw_bias_deg` | Adds a fixed yaw offset in degrees. |
| `yaw_noise_stddev_deg` | Adds Gaussian yaw noise in degrees. |
| `pose_covariance_scale` | Multiplies every pose covariance entry by this value. |
| `twist_covariance_scale` | Multiplies every twist covariance entry by this value. |
| `pose_covariance_floor` | Applies a minimum value to every pose covariance entry. |
| `twist_covariance_floor` | Applies a minimum value to every twist covariance entry. |

Example:

```yaml
- id: odom_bias
  injector_id: odom
  active_on_startup: false
  start: 5.0
  duration: 10.0
  config:
    x_bias: 1.0
    y_bias: -0.25
    yaw_bias_deg: 15.0
```

## LaserScan Faults

Injector type: `scan`

Message type: `sensor_msgs/msg/LaserScan`

| Config Key | Description |
| --- | --- |
| `drop_probability` | Probability from `0.0` to `1.0` that a scan message is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `range_bias` | Adds a fixed offset to each range reading. |
| `range_noise_stddev` | Adds Gaussian noise to range readings. |
| `sector_min_deg` | Start angle of a sector fault in degrees. |
| `sector_max_deg` | End angle of a sector fault in degrees. |
| `sector_value` | Replacement range value for readings inside the configured sector. |

Example:

```yaml
- id: scan_sector_dropout
  injector_id: scan
  active_on_startup: false
  config:
    sector_min_deg: -20.0
    sector_max_deg: 20.0
    sector_value: .inf
```

## Joint State Faults

Injector type: `joint_state`

Message type: `sensor_msgs/msg/JointState`

This is used for motor feedback style topics such as `/platform/motors/feedback`.

| Config Key | Description |
| --- | --- |
| `drop_probability` | Probability from `0.0` to `1.0` that a joint state message is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `velocity_bias` | Adds a fixed offset to each velocity entry. |
| `velocity_noise_stddev` | Adds Gaussian noise to each velocity entry. |

Example:

```yaml
- id: motor_velocity_bias
  injector_id: motor_feedback
  active_on_startup: false
  config:
    velocity_bias: 0.1
```

## IMU Faults

Injector type: `imu`

Message type: `sensor_msgs/msg/Imu`

| Config Key | Description |
| --- | --- |
| `drop_probability` | Probability from `0.0` to `1.0` that an IMU message is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `angular_velocity_z_bias` | Adds a fixed offset to `angular_velocity.z`. |
| `angular_velocity_z_noise_stddev` | Adds Gaussian noise to `angular_velocity.z`. |
| `linear_acceleration_x_bias` | Adds a fixed offset to `linear_acceleration.x`. |
| `linear_acceleration_x_noise_stddev` | Adds Gaussian noise to `linear_acceleration.x`. |
| `linear_acceleration_y_bias` | Adds a fixed offset to `linear_acceleration.y`. |
| `linear_acceleration_y_noise_stddev` | Adds Gaussian noise to `linear_acceleration.y`. |
| `linear_acceleration_z_bias` | Adds a fixed offset to `linear_acceleration.z`. |
| `linear_acceleration_z_noise_stddev` | Adds Gaussian noise to `linear_acceleration.z`. |

Example:

```yaml
- id: imu_accel_bias
  injector_id: imu
  active_on_startup: false
  config:
    linear_acceleration_x_bias: 0.1
    linear_acceleration_z_noise_stddev: 0.05
```

## TF Faults

Injector type: `tf`

Message type: `tf2_msgs/msg/TFMessage`

| Config Key | Description |
| --- | --- |
| `parent_frame` | Optional parent frame filter. If set, only matching transforms are mutated. |
| `child_frame` | Optional child frame filter. If set, only matching transforms are mutated. |
| `drop_probability` | Probability from `0.0` to `1.0` that a matching transform is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `x_bias` | Adds a fixed offset to transform translation `x`. |
| `y_bias` | Adds a fixed offset to transform translation `y`. |
| `z_bias` | Adds a fixed offset to transform translation `z`. |
| `roll_bias_deg` | Adds a fixed roll offset in degrees. |
| `pitch_bias_deg` | Adds a fixed pitch offset in degrees. |
| `yaw_bias_deg` | Adds a fixed yaw offset in degrees. |

Example:

```yaml
- id: base_link_yaw_bias
  injector_id: tf
  active_on_startup: false
  config:
    parent_frame: odom
    child_frame: base_link
    yaw_bias_deg: 10.0
```

## Twist Faults

Injector type: `twist`

Message type: `geometry_msgs/msg/Twist`

Twist faults are useful for command paths such as `/cmd_vel`. Remap the command producer to an input topic such as `/cmd_vel_raw`, then let the injector publish the normal controller-facing `/cmd_vel` topic.

| Config Key | Description |
| --- | --- |
| `drop_probability` | Probability from `0.0` to `1.0` that a command message is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `stale_replay_enabled` | Replays the previously received command instead of forwarding the newest command. |
| `stale_replay_duration_ms` | Maximum age of the stored command that may be replayed, in milliseconds. |
| `linear_x_scale` | Multiplies the `linear.x` command by this value. |
| `angular_z_scale` | Multiplies the `angular.z` command by this value. |
| `max_linear_x` | Clamps `linear.x` symmetrically to `[-max_linear_x, max_linear_x]`. |
| `max_angular_z` | Clamps `angular.z` symmetrically to `[-max_angular_z, max_angular_z]`. |
| `force_stop` | Publishes a zero `Twist` command while active. |

Example:

```yaml
- id: cmd_vel_stale_replay
  injector_id: cmd_vel
  active_on_startup: false
  config:
    stale_replay_enabled: true
    stale_replay_duration_ms: 1000

- id: cmd_vel_clamp
  injector_id: cmd_vel
  active_on_startup: false
  config:
    max_linear_x: 0.25
    max_angular_z: 0.5
```

## Trigger Service Faults

Injector type: `trigger_service`

Service type: `std_srvs/srv/Trigger`

| Config Key | Description |
| --- | --- |
| `delay_ms` | Delays the service response by this many milliseconds. |
| `force_failure` | When true, returns a failed response without calling the target service. |
| `failure_message` | Response message used when `force_failure` is active. |

Example:

```yaml
- id: enable_motors_failure
  injector_id: enable_motors
  active_on_startup: false
  config:
    force_failure: true
    failure_message: injected enable motors failure
```

## Point Cloud Faults

Injector type: `point_cloud`

Message type: `sensor_msgs/msg/PointCloud2`

Point cloud faults are useful for 3D lidar streams. For an Ouster-style setup,
publish the sensor output on `/ouster/points_raw`, then consume the injected
output from `/ouster/points`.

| Config Key | Description |
| --- | --- |
| `drop_probability` | Probability from `0.0` to `1.0` that a complete point cloud message is dropped. |
| `delay_ms` | Delays forwarding by this many milliseconds. |
| `point_dropout_probability` | Probability from `0.0` to `1.0` that each individual point is invalidated with `NaN` coordinates. |
| `range_noise_stddev` | Standard deviation of Gaussian range noise applied along each point ray. |
| `dust_return_probability` | Probability from `0.0` to `1.0` that each point is converted into a short-range dust return. |
| `dust_min_range` | Minimum range, in metres, for generated dust returns. |
| `dust_max_range` | Maximum range, in metres, for generated dust returns. |
| `dust_intensity_scale` | Intensity multiplier applied only to points converted into dust returns. |
| `intensity_scale` | Intensity multiplier applied to every point in the cloud. Use it for broad signal attenuation. |

Dust returns and broad attenuation are separate effects. Use
`dust_intensity_scale` when only the generated near dust returns should have
lower intensity. Leave `intensity_scale` unset, or set it to `1.0`, when
background points should retain their original intensity.

Example:

```yaml
injectors:
  - id: ouster_points
    type: point_cloud
    topic:
      input_topic: /ouster/points_raw
      output_topic: /ouster/points
      qos_depth: 10

faults:
  - id: ouster_dust_returns
    injector_id: ouster_points
    active_on_startup: false
    start: 5.0
    duration: 10.0
    config:
      dust_return_probability: 0.35
      dust_min_range: 0.2
      dust_max_range: 1.5
      dust_intensity_scale: 0.2
```

## Runtime Updates

Fault config values can be inspected and changed while the node is running:

```bash
ros2 service call /fault_injection/get_fault_config ros2_fault_injection/srv/GetFaultConfig \
  "{fault_id: odom_bias}"
```

The response returns parallel `keys` and `values` arrays containing the current runtime config for that fault.

```bash
ros2 service call /fault_injection/set_fault_config ros2_fault_injection/srv/SetFaultConfig \
  "{fault_id: odom_bias, key: x_bias, value: '2.0'}"
```

Only keys listed by the owning injector schema are accepted, and values are validated before being stored. Scheduling fields such as `start`, `duration`, and `active_on_startup` are part of the scenario and are not changed through `SetFaultConfig`.


## Assertions

Assertions are optional scenario entries that check whether expected outcomes happen while the scenario runs. They do not mutate messages or services. They observe framework output and publish pass/fail results.

Supported assertion types:

| Type | Description |
| --- | --- |
| `fault_event` | Passes when a named fault publishes an expected state. |
| `topic_hz` | Passes when a topic publishes at or above a minimum frequency. |

Common fields:

| Field | Type | Description |
| --- | --- | --- |
| `id` | string | Unique assertion name used in assertion events and scenario status. |
| `type` | string | Assertion type, such as `fault_event` or `topic_hz`. |
| `within` | seconds | Optional deadline. The assertion fails if the expected condition is not observed before this time. |

`fault_event` fields:

| Field | Type | Description |
| --- | --- | --- |
| `fault_id` | string | Fault event to watch. Must reference an existing fault ID. |
| `state` | string | Expected fault state. Supported values: `active`, `inactive`. |

`topic_hz` fields:

| Field | Type | Description |
| --- | --- | --- |
| `topic` | string | Topic to monitor. |
| `message_type` | string | ROS interface type, such as `nav_msgs/msg/Odometry`. |
| `min_hz` | number | Minimum acceptable publish frequency. Must be greater than `0.0`. |
| `window` | seconds | Rolling measurement window used to estimate topic frequency. Must be greater than `0.0`. |

Example:

```yaml
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

The validator rejects duplicate assertion IDs, unsupported assertion types, unknown fault IDs, unsupported states, missing topic rate fields, invalid timing values, and non-positive `min_hz` or `window` values. Assertion state changes are published on `/fault_injection/assertion_events`; the current scenario summary is published on `/fault_injection/scenario_status`.
