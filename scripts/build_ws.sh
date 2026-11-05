#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# Build against the ROS underlay without workspace overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
python3 scripts/generate_map.py
colcon --log-base log-navcraft build --base-paths src --build-base build-navcraft \
  --install-base install-navcraft --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
