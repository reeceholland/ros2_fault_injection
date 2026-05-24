# Runtime Services

`ros2_fault_injection` exposes runtime services under the `/fault_injection` namespace.

Use these services to list faults, inspect state, activate or deactivate faults, update config values, and reload the scenario file without restarting the node.

## List Faults

Service:

```text
/fault_injection/list_faults
```

Type:

```text
ros2_fault_injection/srv/ListFaults
```

Request:

```text
---
```

Response:

```text
string[] fault_ids
string[] active_fault_ids
```

Example:

```bash
ros2 service call /fault_injection/list_faults ros2_fault_injection/srv/ListFaults {}
```

Use this when you only need the known fault IDs and which of those are currently active.

## Get Fault Status

Service:

```text
/fault_injection/get_fault_status
```

Type:

```text
ros2_fault_injection/srv/GetFaultStatus
```

Request:

```text
---
```

Response:

```text
ros2_fault_injection/FaultStatus[] faults
```

Each `FaultStatus` contains:

```text
string fault_id
string injector_id
string state
string details
```

Example:

```bash
ros2 service call /fault_injection/get_fault_status ros2_fault_injection/srv/GetFaultStatus {}
```

Example response:

```yaml
faults:
- fault_id: odom_bias
  injector_id: odom
  state: inactive
  details: "scheduled, start=5000ms, duration=10000ms, config={x_bias=1.0}"
```

Use this for UI panels, debugging, and logs where the injector ID and human-readable fault details matter.

## Set Fault State

Service:

```text
/fault_injection/set_fault_state
```

Type:

```text
ros2_fault_injection/srv/SetFaultState
```

Request:

```text
string fault_id
bool active
---
```

Response:

```text
bool success
string message
```

Activate a fault:

```bash
ros2 service call /fault_injection/set_fault_state ros2_fault_injection/srv/SetFaultState \
  "{fault_id: odom_bias, active: true}"
```

Deactivate a fault:

```bash
ros2 service call /fault_injection/set_fault_state ros2_fault_injection/srv/SetFaultState \
  "{fault_id: odom_bias, active: false}"
```

Successful calls publish a `FaultEvent` with `state` set to `active` or `inactive` and `source` set to `manual`.

## Set Fault Config

Service:

```text
/fault_injection/set_fault_config
```

Type:

```text
ros2_fault_injection/srv/SetFaultConfig
```

Request:

```text
string fault_id
string key
string value
---
```

Response:

```text
bool success
string message
```

Example:

```bash
ros2 service call /fault_injection/set_fault_config ros2_fault_injection/srv/SetFaultConfig \
  "{fault_id: odom_bias, key: x_bias, value: '2.0'}"
```

This changes one entry in the fault's `config` map while the node is running.

Only keys valid for that injector type are accepted. For example, `x_bias` is valid for an `odom` injector, while `range_bias` is valid for a `scan` injector.

This service does not change top-level scheduling fields such as `start`, `duration`, `active_on_startup`, `id`, or `injector_id`.

Successful calls publish a `FaultEvent` with `state` set to `config_updated` and `source` set to `manual`.

## Reload Scenario

Service:

```text
/fault_injection/reload_scenario
```

Type:

```text
ros2_fault_injection/srv/ReloadScenario
```

Request:

```text
---
```

Response:

```text
bool success
string message
```

Example:

```bash
ros2 service call /fault_injection/reload_scenario ros2_fault_injection/srv/ReloadScenario {}
```

Reloads the same scenario file that was passed to `fault_injector.launch.py`.

Reload can update fault definitions, config values, startup state, and schedules for existing injectors.

Reload cannot change:

- injector IDs
- injector types
- topic input/output endpoints
- trigger service proxy/target endpoints
- QoS depth
- the number of configured injectors

If validation or compatibility checks fail, the currently running scenario remains unchanged.

## Fault Events

Topic:

```text
/fault_injection/events
```

Type:

```text
ros2_fault_injection/msg/FaultEvent
```

Message:

```text
builtin_interfaces/Time stamp
string fault_id
string injector_id
string state
string source
string details
```

Example:

```bash
ros2 topic echo /fault_injection/events
```

Events are published when scheduled faults activate or deactivate and when faults are changed through the service API.

Common event states:

| State | Meaning |
| --- | --- |
| `active` | Fault became active. |
| `inactive` | Fault became inactive. |
| `config_updated` | A runtime config value changed. |

Common event sources:

| Source | Meaning |
| --- | --- |
| `startup` | Fault was activated during scenario startup. |
| `scheduled` | Fault changed because of a scheduled start or stop. |
| `manual` | Fault changed because of a service call. |
