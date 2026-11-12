cmake_minimum_required(VERSION 3.20)
project(navcraft_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateSequence.action"
  "srv/SetChassisMode.srv"
  "msg/TrackingError.msg"
  "msg/TrackingStatistics.msg"
  "msg/RobotState.msg"
  "msg/ChassisState.msg"
  "msg/NavigationStatistics.msg"
  "msg/MapZone.msg"
  "msg/SafetyState.msg"
  "srv/ListMaps.srv"
  "srv/SetActiveMap.srv"
  "srv/ListMapZones.srv"
  "srv/EvaluateMapPosition.srv"
  "srv/SetControlMode.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
