^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_fault_injection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-10)
------------------
* Added pluginlib-based injector discovery.
* Added Twist, PointCloud2, and service fault support.
* Added PointCloud2 dust, dropout, range noise, and intensity fault configuration.
* Added scenario assertions, scenario status publishing, and markdown report generation.
* Added dynamic scenario reload and scenario inspection services.
* Added RViz integration support for fault status, configuration, assertions, and reports.
* Added Read the Docs documentation.

0.1.0 (2026-05-24)
------------------
* Added C++ fault injection framework.
* Added odom, scan, joint state, IMU, TF, and trigger service injectors.
* Added YAML scenario parsing and validation.
* Added runtime services for listing faults, changing fault state, and updating fault configuration.
* Added structured fault event publishing.
* Added unit tests for scheduler, services, injectors, config schema, validation, and event formatting.
* Added Doxygen API documentation support.
* Added uncrustify formatting and copyright checks.
