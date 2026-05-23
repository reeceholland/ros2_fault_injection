# ros2_fault_injection API

`ros2_fault_injection` is a C++ ROS 2 framework for injecting controlled faults into robot topic streams.

The public API is organized around a small set of framework classes:

- `ros2_fault_injection::FaultInjector` defines the common injector interface.
- `ros2_fault_injection::FaultInjectorBase` stores shared fault state and config.
- `ros2_fault_injection::FaultController` creates injectors, registers faults, and starts scheduling.
- `ros2_fault_injection::FaultScheduler` activates and deactivates scheduled faults.
- `ros2_fault_injection::FaultServiceManager` exposes runtime service control.
- `ros2_fault_injection::FaultEventPublisher` publishes structured fault events.

Typed injectors implement message-specific mutation for odometry, laser scans, joint states, and IMU data.

See the project README for runtime usage, YAML scenario format, and service examples.


## Contributor Guides

- `docs/adding_an_injector.md`
- `docs/adding_a_fault_type.md`
