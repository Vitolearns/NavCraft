#!/usr/bin/env bash
set -euo pipefail

ros2 service type /list_maps | rg "navcraft_interfaces/srv/ListMaps"
ros2 service type /set_active_map | rg "navcraft_interfaces/srv/SetActiveMap"
ros2 topic type /map_manager/state | rg "navcraft_interfaces/msg/RobotState"
ros2 topic type /map_manager/catalog | rg "std_msgs/msg/String"
ros2 topic type /map_manager/zones | rg "std_msgs/msg/String"
ros2 service call /list_maps navcraft_interfaces/srv/ListMaps "{}" | rg "indoor_room"
ros2 service call /set_active_map navcraft_interfaces/srv/SetActiveMap "{name: indoor_room}" | rg "success=True|success: true"
ros2 topic echo --once /map_manager/state navcraft_interfaces/msg/RobotState --field state | rg "OK|WARN|ERROR"
