#!/bin/bash
# MIE 443 Contest 2 — Simulation Launcher
# Usage: ./run_contest2_sim.sh [WORLD]
#   WORLD: maze (default), warehouse, depot

# Note: change the map directory
# Note: ./run_contest2_sim.sh    run in terminal


WORLD=${1:-maze}
WS=~/ros2_ws
ROS_DISTRO=jazzy
MAP=${2:-/home/admin1/ros2_ws/src/mie443_contest2/mie443_contest2/mie443_contest2/maps/Contest2MapPractice.yaml}
SOURCE="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash"

# --- Cleanup previous session ---
echo "Cleaning up previous processes..."
pkill -f "turtlebot4_gz.launch" 2>/dev/null
pkill -f "localization.launch" 2>/dev/null
pkill -f "nav2.launch" 2>/dev/null
pkill -f "view_navigation.launch" 2>/dev/null
pkill -f "yolo_detector.py" 2>/dev/null
pkill -f "ros2 run mie443_contest2 contest2" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
pkill -f "ruby" 2>/dev/null
sleep 3
echo "Done."

# --- Build ---
echo "Building mie443_contest2..."
cd ${WS} && colcon build --packages-select mie443_contest2
if [ $? -ne 0 ]; then
    echo "ERROR: Build failed. Fix errors before launching."
    exit 1
fi
source ${WS}/install/setup.bash
echo "Build complete."
echo ""

# Check for YOLO venv
YOLO_ACTIVATE=""
if [ -f ~/contest2/bin/activate ]; then
    YOLO_ACTIVATE="source ~/contest2/bin/activate &&"
else
    echo "WARNING: Python venv not found at ~/contest2 — YOLO will run without it."
fi

echo "Launching Contest 2 Simulation | world: $WORLD"
echo "Map: $MAP"
echo ""

# 1. Gazebo
gnome-terminal --title="1 Gazebo" -- bash -c "${SOURCE} && ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite world:=${WORLD}; exec bash" &

# 2. Localization (wait for Gazebo)
sleep 1
gnome-terminal --title="2 Localization" -- bash -c "sleep 15 && ${SOURCE} && ros2 launch turtlebot4_navigation localization.launch.py map:=${MAP}; exec bash" &

# 3. Nav2
gnome-terminal --title="3 Nav2" -- bash -c "sleep 20 && ${SOURCE} && ros2 launch turtlebot4_navigation nav2.launch.py; exec bash" &

# 4. RViz
gnome-terminal --title="4 RViz" -- bash -c "sleep 22 && ${SOURCE} && ros2 launch turtlebot4_viz view_navigation.launch.py; exec bash" &

# 5. YOLO Detector
gnome-terminal --title="5 YOLO Detector" -- bash -c "sleep 15 && ${YOLO_ACTIVATE} ${SOURCE} && ros2 run mie443_contest2 yolo_detector.py; exec bash" &

# 6. Contest2 (manual trigger)
gnome-terminal --title="6 Contest2 (RUN LAST)" -- bash -c "echo 'Wait for all other windows to finish starting, then press Enter to launch contest2...' && read && ${SOURCE} && ros2 run mie443_contest2 contest2; exec bash" &

echo "All windows launched. Press Enter in window 6 when ready to start contest2."
