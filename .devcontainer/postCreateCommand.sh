#!/usr/bin/env bash
set -e

# ROS środowisko
source /opt/ros/humble/setup.bash

# Jeśli masz workspace colcon w repo, odkomentuj i dostosuj:
# sudo rosdep init 2>/dev/null || true
# rosdep update
# rosdep install --from-paths src -i -y --rosdistro humble

# colcon build --symlink-install
