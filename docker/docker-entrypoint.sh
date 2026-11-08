#!/bin/bash
# Runs after /dds_check.sh. Sources the ROS overlay, then replaces itself with the command (PID 1 gets SIGTERM).
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
