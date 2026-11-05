#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="navcraft-${ROS_DOMAIN_ID}"
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
case "${1:-up}" in
  up) shift || true; exec python3 scripts/launch_demo.py "$@" ;;
  run) shift; exec python3 scripts/mission.py "$@" ;;
  warehouse) exec python3 scripts/mission.py --route warehouse ;;
  obstacle) shift; exec python3 scripts/warehouse_obstacle.py "$@" ;;
  pause|resume|cancel) exec ros2 service call "/${1}_mission" std_srvs/srv/Trigger '{}' ;;
  status) exec ros2 topic echo /execution_state std_msgs/msg/String ;;
  estop) exec ros2 service call /emergency_stop std_srvs/srv/SetBool '{data: true}' ;;
  release) exec ros2 service call /emergency_stop std_srvs/srv/SetBool '{data: false}' ;;
  *) echo 'Usage: bash scripts/demo.sh {up [gui:=false use_rviz:=false]|run|warehouse|obstacle {block|clear}|pause|resume|cancel|status|estop|release}' >&2; exit 2 ;;
esac
