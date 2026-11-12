cmake_minimum_required(VERSION 3.20)
project(navcraft_evaluation)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(navcraft_interfaces REQUIRED)

add_executable(experiment_runner_node src/experiment_runner_node.cpp)
ament_target_dependencies(experiment_runner_node
  rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs navcraft_interfaces)

install(TARGETS experiment_runner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY paths
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_tracking_statistics.launch.py TIMEOUT 60)
endif()

ament_package()
