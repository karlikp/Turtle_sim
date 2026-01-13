#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DISTRO="${ROS_DISTRO:-humble}"

# ROS środowisko
source /opt/ros/humble/setup.bash

# Jeśli masz workspace colcon w repo, odkomentuj i dostosuj:
# sudo rosdep init 2>/dev/null || true
cd "$SCRIPT_DIR/../turtlebot_sim"
rosdep update
rosdep install --from-paths . -y --ignore-src --rosdistro "$ROS_DISTRO"

cd "$SCRIPT_DIR/.."
colcon build 
source /opt/ros/humble/setup.bash
