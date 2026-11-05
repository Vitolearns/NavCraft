#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
colcon --log-base log-navcraft test --base-paths src --build-base build-navcraft --install-base install-navcraft --return-code-on-test-failure
colcon --log-base log-navcraft test-result --test-result-base build-navcraft --verbose
python3 scripts/test_executor.py
