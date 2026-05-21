cmake_minimum_required(VERSION 3.16)
project(ros2_fault_injection VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFaultState.srv"
  "srv/ListFaults.srv"
  "srv/GetFaultStatus.srv"
  "msg/FaultStatus.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_library(${PROJECT_NAME}_core
  src/fault_injector_base.cpp
  src/odom_fault_injector.cpp
  src/scan_fault_injector.cpp
  src/joint_state_fault_injector.cpp
  src/scenario_config.cpp
  src/fault_scheduler.cpp
  src/fault_event_publisher.cpp
  src/scenario_validator.cpp
  src/fault_injector_factory.cpp
  src/fault_service_manager.cpp
  src/fault_controller.cpp
  src/fault_config_schema.cpp
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
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
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
  fault_injector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

if(BUILD_TESTING)
  ament_add_gtest(test_scenario_validator
    test/test_scenario_validator.cpp
    test/test_scenario_config.cpp
  )
  target_link_libraries(test_scenario_validator
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_injector_base
    test/test_fault_injector_base.cpp
  )
  target_link_libraries(test_fault_injector_base
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_fault_config_schema
    test/test_fault_config_schema.cpp
  )
  target_link_libraries(test_fault_config_schema
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_fault_scheduler
    test/test_fault_scheduler.cpp
  )
  target_link_libraries(test_fault_scheduler
    ${PROJECT_NAME}_core
  )
endif()

ament_package()
