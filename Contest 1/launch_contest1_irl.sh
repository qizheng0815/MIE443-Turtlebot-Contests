#!/bin/bash

# 1. Launch SLAM Toolbox (Terminal 1)
echo "Launching SLAM Toolbox..."
terminator -e "ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=false" &

# 2. Launch Rviz Visualization (Terminal 2)
echo "Launching Rviz..."
terminator -e "ros2 launch turtlebot4_viz view_navigation.launch.py" &

# 3. Wait for SLAM/Rviz to initialize (5 seconds)
echo "Waiting for systems to come online..."
sleep 5

# 4. Build and Run the C++ Node (Terminal 3)
# We chain the commands: Build -> Source -> (Optional) Run
CMD_BUILD="bash -c 'WS=~/ros2_ws; \
PKG=mie443_contest1; \
NODE=contest1; \
cd \$WS; \
colcon build --packages-select \$PKG --symlink-install && \
source install/setup.bash && \
echo \"--- Running Contest 1 Node ---\" && \
ros2 run \$PKG \$NODE; \
exec bash'"

echo "Opening Build/Control Terminal..."
terminator -e "$CMD_BUILD" &

exit 0