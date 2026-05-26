cmake_minimum_required(VERSION 3.16)
project(ros2_fault_injection VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Doxygen)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFaultState.srv"
  "srv/ListFaults.srv"
  "srv/GetFaultStatus.srv"
  "srv/SetFaultConfig.srv"
  "srv/ReloadScenario.srv"
  "msg/FaultStatus.msg"
  "msg/FaultEvent.msg"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_library(${PROJECT_NAME}_core
  src/core/fault_injector_base.cpp
  src/injectors/odom_fault_injector.cpp
  src/injectors/scan_fault_injector.cpp
  src/injectors/joint_state_fault_injector.cpp
  src/injectors/imu_fault_injector.cpp
  src/config/scenario_config.cpp
  src/core/fault_scheduler.cpp
  src/core/fault_event_publisher.cpp
  src/config/scenario_validator.cpp
  src/core/fault_injector_factory.cpp
  src/core/builtin_fault_injector_plugin.cpp
  src/core/fault_service_manager.cpp
  src/core/fault_controller.cpp
  src/config/fault_config_schema.cpp
  src/utils/fault_descriptions.cpp
  src/injectors/trigger_service_fault_injector.cpp
  src/injectors/tf_fault_injector.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  yaml-cpp
  "${cpp_typesupport_target}"
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  pluginlib
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  tf2_msgs
)

add_executable(fault_injector_node
  src/fault_injector_node.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(fault_injector_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(fault_injector_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  fault_injector_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(ros2_fault_injection fault_injector_plugins.xml)

install(FILES
  CHANGELOG.rst
  LICENSE
  README.md
  fault_injector_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}
)


if(DOXYGEN_FOUND)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    @ONLY
  )

  set(DOXYGEN_DOCS_COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  set(DOXYGEN_DOCS_COMMENT "Generating API documentation with Doxygen")
else()
  set(DOXYGEN_DOCS_COMMAND
    ${CMAKE_COMMAND} -E echo "Doxygen not found. Install doxygen to generate API documentation."
    COMMAND ${CMAKE_COMMAND} -E false
  )
  set(DOXYGEN_DOCS_COMMENT "Doxygen is required to generate API documentation")
endif()

add_custom_target(generate_doxygen_api
  COMMAND ${DOXYGEN_DOCS_COMMAND}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT ${DOXYGEN_DOCS_COMMENT}
  VERBATIM
)

if(BUILD_TESTING)
  ament_add_gtest(test_scenario_validator
    test/config/test_scenario_validator.cpp
    test/config/test_scenario_config.cpp
    test/config/test_scenario_examples.cpp
  )
  target_compile_definitions(test_scenario_validator
    PRIVATE
      ROS2_FAULT_INJECTION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(test_scenario_validator
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_injector_base
    test/core/test_fault_injector_base.cpp
  )
  target_link_libraries(test_fault_injector_base
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_config_schema
    test/config/test_fault_config_schema.cpp
  )
  target_link_libraries(test_fault_config_schema
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_fault_scheduler
    test/core/test_fault_scheduler.cpp
  )
  target_link_libraries(test_fault_scheduler
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_service_manager
    test/core/test_fault_service_manager.cpp
  )
  target_link_libraries(test_fault_service_manager
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_description
    test/utils/test_fault_description.cpp
  )
  target_link_libraries(test_fault_description
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_odom_fault_injector
    test/injectors/test_odom_fault_injector.cpp
  )
  target_link_libraries(test_odom_fault_injector
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_trigger_service_fault_injector
    test/injectors/test_trigger_service_fault_injector.cpp
  )
  target_link_libraries(test_trigger_service_fault_injector
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_tf_fault_injector
    test/injectors/test_tf_fault_injector.cpp
  )
  target_link_libraries(test_tf_fault_injector
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_controller
    test/core/test_fault_controller.cpp
  )
  target_link_libraries(test_fault_controller
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_injector_factory
    test/core/test_fault_injector_factory.cpp
  )
  target_link_libraries(test_fault_injector_factory
    ${PROJECT_NAME}_core
  )

  ament_uncrustify(
    include
    src
    test
  )

  ament_copyright()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  pluginlib
  nav_msgs
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  yaml-cpp
)

ament_package()
