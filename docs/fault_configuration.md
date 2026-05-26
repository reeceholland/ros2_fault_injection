# Fault Configuration

Scenarios are YAML files that describe one or more injectors and the faults each injector can apply.

At runtime, each fault is registered against an `injector_id`. The injector type decides which keys are valid inside that fault's `config` map.

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

Only keys listed for that injector type are accepted, and values are validated before being stored. Scheduling fields such as `start`, `duration`, and `active_on_startup` are part of the scenario and are not changed through `SetFaultConfig`.
