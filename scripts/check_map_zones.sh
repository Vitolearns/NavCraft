#!/usr/bin/env bash
set -euo pipefail

ros2 service type /list_map_zones | rg "navcraft_interfaces/srv/ListMapZones"
ros2 service type /evaluate_map_position | rg "navcraft_interfaces/srv/EvaluateMapPosition"
ros2 topic type /map_manager/zones | rg "std_msgs/msg/String"
ros2 service call /list_map_zones navcraft_interfaces/srv/ListMapZones "{map_name: indoor_room, include_disabled: false}" | rg "precision_zone|restricted_zone"
ros2 service call /evaluate_map_position navcraft_interfaces/srv/EvaluateMapPosition "{map_name: indoor_room, point: {x: -1.0, y: -1.0, z: 0.0}}" | rg "speed_limit_mps=0.18|speed_limit_mps: 0.18"
ros2 service call /evaluate_map_position navcraft_interfaces/srv/EvaluateMapPosition "{map_name: indoor_room, point: {x: -2.0, y: 1.8, z: 0.0}}" | rg "allowed=False|allowed: false"
