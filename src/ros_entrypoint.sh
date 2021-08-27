#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/galactic/setup.bash"
source "/ros2_ws/install/setup.bash"

exec "$@"