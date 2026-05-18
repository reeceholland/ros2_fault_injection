cmake_minimum_required(VERSION 3.16)
project(ros2_fault_injection VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/odom_fault_injector.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  nav_msgs
)

add_executable(odom_fault_injector_node
  src/odom_fault_injector_node.cpp
)

target_link_libraries(odom_fault_injector_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(odom_fault_injector_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  odom_fault_injector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
