#!/bin/bash

set -e

WS=~/ros2_ws
PKG=mie443_contest2
NODE=contest2

echo " Killing old Gazebo + contest2"

pkill -f "ros2 run $PKG $NODE" || true
pkill -f gazebo || true
pkill -f gz || true

sleep 1

echo "Fast rebuild: $PKG"
cd $WS
colcon build --packages-select $PKG

echo "Sourcing workspace"
source install/setup.bash

#######################################
# Terminator 1: Gazebo
# ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite rviz:=true SLAM:=true;
#######################################
terminator \
  --title="Gazebo Simulator" \
  -e "bash -c '
    source $WS/install/setup.bash;
    echo \"Launching Gazebo\";
    ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite world:=maze rviz:=true slam:=true;
    exec bash
  '" &

#######################################
# Terminator 2: contest2 node
#######################################
terminator \
  --title="contest2 Node" \
  -e "bash -c '
    source $WS/install/setup.bash;
    echo \"Waiting for /odom topic\";
    until ros2 topic list | grep -q /odom; do sleep 1; done;
    echo \"Running contest2 node\";
    ros2 run $PKG $NODE;
    exec bash
  '" &

echo "Launch complete (separate Terminator windows)"

