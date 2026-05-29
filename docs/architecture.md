# Architecture

`ros2_fault_injection` is organized around a small runtime pipeline:

```text
scenario YAML
  -> ScenarioConfig parser
  -> ScenarioValidator
  -> FaultController
  -> FaultInjectorFactory / pluginlib
  -> typed injectors
  -> FaultScheduler
  -> services and events
  -> optional RViz panel
```

The goal is to keep message-specific fault behavior isolated while keeping scheduling, services, validation,
and UI integration generic.

## Scenario Loading

A scenario YAML file describes the runtime layout:

- `injectors` define the proxy endpoints and injector type.
- `faults` define named fault configs owned by an injector.
- scheduling fields such as `active_on_startup`, `start`, and `duration` describe when faults should be active.

`ScenarioConfig` parses the YAML into typed C++ structs. `ScenarioValidator` checks the parsed scenario before
the node starts or before a reload is accepted.

## Controller

`FaultController` owns the high-level runtime setup:

- loads and validates the scenario
- creates injectors through `FaultInjectorFactory`
- registers faults with their owning injector
- starts the scheduler
- exposes the injector map used by runtime services

It is intentionally the place where the scenario becomes a running fault injection system.

## Injector Factory And Pluginlib

`FaultInjectorFactory` creates injectors by type. Built-in injector types are provided through a pluginlib
wrapper, and external packages can add new injector types by exporting their own `FaultInjectorPlugin`.

This lets the framework grow without editing the core factory every time a downstream project adds a custom
message type.

## Injectors

Every injector implements the `FaultInjector` interface. Most injectors derive from `FaultInjectorBase`, which
stores common state:

- registered fault configs
- active fault IDs
- runtime config value updates

Typed injectors own the ROS communication, message mutation, and editable config schema for one domain. For
example, the odom injector subscribes to raw odometry, applies active odom faults, and republishes the mutated
odometry.

Each built-in injector exposes a `FaultConfigField` schema. The same schema is used by scenario validation,
runtime config updates, and the RViz panel. This keeps YAML loading and `/fault_injection/set_fault_config`
consistent: a value accepted in a scenario should follow the same rules as a value changed at runtime.

Topic injectors follow this proxy pattern:

```text
producer -> /topic_raw -> injector -> /topic -> consumers
```

Service injectors follow the same idea:

```text
client -> /service -> injector -> /service_raw -> real server
```

## Scheduler

`FaultScheduler` activates and deactivates faults according to the scenario:

- startup-active faults are enabled immediately
- scheduled faults are enabled after `start`
- faults with `duration` are disabled after the duration expires
- manual-only faults remain inactive until changed through the service API

Scheduler actions publish fault events so UI tools and logs can show what happened.

## Runtime Services

`FaultServiceManager` provides the runtime control API under `/fault_injection`:

- list faults
- get fault status
- get editable config schema
- get current fault config
- set one config value
- set active/inactive state
- reload the current scenario file

The services are intentionally thin. They validate requests against the owning injector schema, call
injector/controller behavior, and publish events when runtime state changes.

## Events

`FaultEventPublisher` emits structured events on `/fault_injection/events`.

Events are published for scheduled changes, startup activation, manual state changes, config updates, and
other runtime actions. The RViz panel uses this topic to show recent activity.

## RViz Panel

`ros2_fault_injection_rviz` is a separate RViz plugin package. It talks to the framework only through public
services and events:

- status table from `/fault_injection/get_fault_status`
- recent events from `/fault_injection/events`
- scenario reload through `/fault_injection/reload_scenario`
- config key dropdown from `/fault_injection/get_fault_schema`
- current config values from `/fault_injection/get_fault_config`
- runtime edits through `/fault_injection/set_fault_config`

Keeping RViz on the public API is useful: if the panel works, a script or another UI can use the same interfaces.

## Extension Points

To add behavior, prefer one of these extension points:

- add a new config key to an existing injector
- add a new typed injector
- add an external plugin package
- add UI support in the RViz panel

Keep validation, schema, examples, docs, and tests updated with the behavior. This keeps the framework predictable
as it grows.
