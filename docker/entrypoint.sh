#!/bin/bash
# Carrega o ROS 2 (e o workspace do grupo, se existir) antes de qualquer comando
set -e
source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
exec "$@"
