# Runtime Services

`ros2_fault_injection` exposes runtime services under the `/fault_injection` namespace.

Use these services to list faults, inspect state, read current config values, activate or deactivate faults, update config values, and reload the scenario file without restarting the node.

These services are available from the long-running `fault_injector_node` path, for example when started through `fault_injector.launch.py`. The headless `fault_scenario_runner_node` is intended for CI/smoke-test execution and exits once the scenario passes, fails, or times out, so it is not suitable for RViz service requests after completion.

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

## Get Fault Config Schema

Service:

```text
/fault_injection/get_fault_schema
```

Type:

```text
ros2_fault_injection/srv/GetFaultSchema
```

Request:

```text
string fault_id
---
```

Response:

```text
bool success
string message
string injector_id
string injector_type
string[] keys
```

Example:

```bash
ros2 service call /fault_injection/get_fault_schema ros2_fault_injection/srv/GetFaultSchema \
  "{fault_id: odom_bias}"
```

Use this service when a UI or script needs to discover which config keys are valid for a fault before sending a runtime update.

## Get Fault Config

Service:

```text
/fault_injection/get_fault_config
```

Type:

```text
ros2_fault_injection/srv/GetFaultConfig
```

Request:

```text
string fault_id
---
```

Response:

```text
bool success
string message
string injector_id
string injector_type
string[] keys
string[] values
```

Example:

```bash
ros2 service call /fault_injection/get_fault_config ros2_fault_injection/srv/GetFaultConfig \
  "{fault_id: odom_bias}"
```

Example response:

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

`keys` and `values` are parallel arrays because ROS 2 service definitions do not support map fields directly. Use this service when a UI or script needs the current runtime value for a fault config key.

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

Only keys and values valid for that injector type are accepted. For example, `x_bias` is valid for an `odom` injector, while `range_bias` is valid for a `scan` injector. Numeric fields, probabilities, booleans, and non-negative values are validated before the stored config changes.

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

## Get Scenario

Service:

```text
/fault_injection/get_scenario
```

Type:

```text
ros2_fault_injection/srv/GetScenario
```

Request:

```text
---
```

Response:

```text
bool success
string message
string scenario_file
string content
```

Example:

```bash
ros2 service call /fault_injection/get_scenario ros2_fault_injection/srv/GetScenario {}
```

This returns the path to the active scenario file and the current YAML file content. RViz uses this to show the loaded scenario without requiring direct filesystem access.

## Request Report

Service:

```text
/fault_injection/request_report
```

Type:

```text
ros2_fault_injection/srv/RequestReport
```

Request:

```text
---
```

Response:

```text
bool success
string message
string final_result
string markdown
```

Example:

```bash
ros2 service call /fault_injection/request_report ros2_fault_injection/srv/RequestReport {}
```

This creates a report from the currently running scenario state and returns the markdown in the service response. The long-running injector node does not save this report to disk by default; callers such as RViz can display or save the returned markdown.

For headless runs, use `fault_scenario_runner_node` with the `report_file` parameter instead:

```bash
ros2 run ros2_fault_injection fault_scenario_runner_node \
  --ros-args \
  -p scenario_file:=/path/to/scenario.yaml \
  -p timeout:=30.0 \
  -p report_file:=/tmp/fault_injection_report.md
```

In runner mode, the report is written to `report_file` when provided and the process exits with `0` on pass or `1` on failure/timeout.

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

## Assertion Events

Topic:

```text
/fault_injection/assertion_events
```

Type:

```text
ros2_fault_injection/msg/AssertionEvent
```

Message:

```text
builtin_interfaces/Time stamp
string assertion_id
string assertion_type
string state
string message
```

Example:

```bash
ros2 topic echo /fault_injection/assertion_events
```

Assertion events are edge-triggered. They are published when an assertion changes from pending to `passed` or `failed`.

## Scenario Status

Topic:

```text
/fault_injection/scenario_status
```

Type:

```text
ros2_fault_injection/msg/ScenarioStatus
```

Message:

```text
builtin_interfaces/Time stamp
string state
uint32 pending_count
uint32 passed_count
uint32 failed_count
string[] failed_assertion_ids
string[] failed_messages
```

Example:

```bash
ros2 topic echo /fault_injection/scenario_status
```

Example output:

```yaml
state: failed
pending_count: 0
passed_count: 2
failed_count: 1
failed_assertion_ids:
- odom_stays_above_10hz
failed_messages:
- Timed out waiting for topic /odom to reach 10.000000 Hz
```

Scenario status is a continuously published snapshot. Use it when a UI, script, or log needs the current scenario health instead of only assertion state-change events.

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


## Assertion Events

Topic:

```text
/fault_injection/assertion_events
```

Type:

```text
ros2_fault_injection/msg/AssertionEvent
```

Message:

```text
builtin_interfaces/Time stamp
string assertion_id
string assertion_type
string state
string message
```

Example:

```bash
ros2 topic echo /fault_injection/assertion_events
```

Assertion events are published by the assertion runner when an assertion changes from pending to a terminal state.

Common assertion states:

| State | Meaning |
| --- | --- |
| `passed` | The expected scenario outcome was observed. |
| `failed` | The expected scenario outcome was not observed before its deadline, or otherwise failed. |

For a `fault_event` assertion, `assertion_type` is `fault_event`, and the message describes the expected fault ID and state.
