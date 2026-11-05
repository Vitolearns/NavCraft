cmake_minimum_required(VERSION 3.20)
project(navcraft_diagnostics)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navcraft_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(navcraft_diagnostics INTERFACE)
target_include_directories(navcraft_diagnostics INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS navcraft_diagnostics
  EXPORT export_navcraft_diagnostics)

add_executable(system_monitor_node src/system_monitor_node.cpp)
ament_target_dependencies(system_monitor_node
  rclcpp diagnostic_msgs diagnostic_updater nav_msgs navcraft_interfaces sensor_msgs std_msgs)

add_executable(fault_supervisor_node src/fault_supervisor_node.cpp)
ament_target_dependencies(fault_supervisor_node
  rclcpp navcraft_interfaces std_srvs)

install(TARGETS system_monitor_node fault_supervisor_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fault_supervisor.launch.py TIMEOUT 30)
endif()

ament_export_targets(export_navcraft_diagnostics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs diagnostic_msgs diagnostic_updater nav_msgs navcraft_interfaces sensor_msgs std_msgs std_srvs)
ament_package()
