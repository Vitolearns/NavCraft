#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
test_status=0
colcon --log-base log-navcraft test --base-paths src --build-base build-navcraft --install-base install-navcraft --return-code-on-test-failure || test_status=$?
# Keep detailed failures visible even when colcon test returns nonzero.
result_status=0
colcon --log-base log-navcraft test-result --test-result-base build-navcraft --verbose || result_status=$?
if (( test_status != 0 )); then exit "$test_status"; fi
if (( result_status != 0 )); then exit "$result_status"; fi
python3 scripts/test_executor.py
