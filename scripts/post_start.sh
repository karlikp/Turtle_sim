#!/usr/bin/env bash
set -e

source /home/karol/ws/install/setup.bash # Point(.) is equel (source)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix turtlebot_sim)/share/turtlebot_sim/models_sdf

mkdir -p /home/karol/.config/QGroundControl
chown -R karol:karol /home/karol/.config/QGroundControl
chmod -R u+rwX /home/karol/.config/QGroundControl
chmod -R a+rw /home/karol/ws/src/turtlebot_sim/worlds || true



colcon build
