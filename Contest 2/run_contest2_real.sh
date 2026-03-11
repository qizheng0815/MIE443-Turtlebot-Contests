#!/bin/bash
# MIE 443 Contest 2 — Real Robot Launcher
# Usage: ./run_contest2_real.sh [MAP_YAML]
#   Connects to the physical TurtleBot4 and launches the full contest2 stack.
#   Gazebo is NOT launched — the real robot runs its own base nodes.
#
# Prerequisites:
#   - PC must be on the same network as the TurtleBot4
#   - Robot IP: 100.69.127.98 (set in cyclonedds_real.xml)
#   - ROS_DOMAIN_ID=14 must match the robot's domain
#
# Pi-side commands (run via SSH BEFORE this script):
#   ssh ubuntu@<ip>
#   source contest2/bin/activate
#   ros2 launch lerobot_moveit so101_turtlebot.launch.py   # arm bridge + TF
#   ros2 run mie443_contest2 image_capture_server           # OAK-D capture service
#   ros2 launch apriltag_ros camera_36h11.launch.yml        # AprilTag detection

WS=~/ros2_ws
ROS_DISTRO=jazzy
MAP=${1:-/home/turtlebot/ros2_ws/src/mie443_contest2/mie443_contest2/maps/Contest2MapPractice.yaml}
SOURCE="export CYCLONEDDS_URI=${WS}/cyclonedds_real.xml && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=14 && source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash"

# ── Cleanup previous session ──────────────────────────────────────────────────
echo "Cleaning up previous processes..."
pkill -f "localization.launch"    2>/dev/null
pkill -f "nav2.launch"            2>/dev/null
pkill -f "view_navigation.launch" 2>/dev/null
pkill -f "yolo_detector.py"       2>/dev/null
pkill -f "move_group"             2>/dev/null
pkill -f "ros2 run mie443_contest2 contest2" 2>/dev/null
sleep 2
echo "Done."

# ── Fix coords.xml filename (repository typo: cords.xml → coords.xml) ────────
COORDS_SRC="${WS}/src/mie443_contest2/mie443_contest2/boxes_database/cords.xml"
COORDS_DST="${WS}/src/mie443_contest2/mie443_contest2/boxes_database/coords.xml"
if [ -f "${COORDS_SRC}" ] && [ ! -f "${COORDS_DST}" ]; then
    cp "${COORDS_SRC}" "${COORDS_DST}"
    echo "Fixed: copied cords.xml → coords.xml"
fi

# ── Build ─────────────────────────────────────────────────────────────────────
echo "Building mie443_contest2..."
cd ${WS} && colcon build --packages-select mie443_contest2
if [ $? -ne 0 ]; then
    echo "ERROR: Build failed. Fix errors before launching."
    exit 1
fi
source ${WS}/install/setup.bash
echo "Build complete."
echo ""

# ── Verify robot is reachable ─────────────────────────────────────────────────
echo "Checking robot connectivity (100.69.127.98)..."
if ! ping -c 1 -W 2 100.69.127.98 &>/dev/null; then
    echo "WARNING: Robot at 100.69.127.98 is not reachable. Check network connection."
    echo "         Continuing anyway — press Ctrl+C to cancel."
    sleep 3
else
    echo "Robot reachable."
fi
echo ""

# ── YOLO model check (yolov8s.pt required by yolo_detector.py) ───────────────
YOLO_MODEL="${WS}/yolov8s.pt"
if [ ! -f "${YOLO_MODEL}" ]; then
    echo "WARNING: ${YOLO_MODEL} not found."
    echo "         yolo_detector.py will auto-download it on first run (requires internet)."
    echo "         Or pre-download: wget -q -O ${YOLO_MODEL} \\"
    echo "           https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt"
fi

# ── YOLO venv check ───────────────────────────────────────────────────────────
YOLO_ACTIVATE=""
if [ -f ~/contest2/bin/activate ]; then
    YOLO_ACTIVATE="source ~/contest2/bin/activate &&"
else
    echo "WARNING: Python venv not found at ~/contest2 — YOLO will run without it."
fi

echo "Launching Contest 2 — Real Robot | Map: $(basename ${MAP})"
echo ""

# 1. Localization
gnome-terminal --title="1 Localization" -- bash -c \
    "${SOURCE} && ros2 launch turtlebot4_navigation localization.launch.py map:=${MAP}; exec bash" &
sleep 1

# 2. Nav2
gnome-terminal --title="2 Nav2" -- bash -c \
    "sleep 10 && ${SOURCE} && ros2 launch turtlebot4_navigation nav2.launch.py; exec bash" &
sleep 1

# 3. RViz (navigation)
gnome-terminal --title="3 RViz Nav" -- bash -c \
    "sleep 12 && ${SOURCE} && ros2 launch turtlebot4_viz view_navigation.launch.py; exec bash" &
sleep 1

# 4. MoveIt move_group + RViz (arm — laptop side; requires arm bridge on Pi)
gnome-terminal --title="4 MoveIt (arm)" -- bash -c \
    "sleep 5 && ${SOURCE} && ros2 launch lerobot_moveit so101_laptop.launch.py; exec bash" &
sleep 1

# 5. YOLO Detector
gnome-terminal --title="5 YOLO Detector" -- bash -c \
    "sleep 5 && ${YOLO_ACTIVATE} ${SOURCE} && ros2 run mie443_contest2 yolo_detector.py; exec bash" &
sleep 1

# 6a. OAK-D annotated camera viewer
gnome-terminal --title="6a OAK-D YOLO View" -- bash -c \
    "sleep 6 && ${SOURCE} && ros2 run rqt_image_view rqt_image_view /yolo/oakd/compressed; exec bash" &
sleep 1


# 6b. Wrist annotated camera viewer
gnome-terminal --title="6b Wrist YOLO View" -- bash -c \
    "sleep 6 && ${SOURCE} && ros2 run rqt_image_view rqt_image_view /yolo/wrist/compressed; exec bash" &
sleep 1

# 7. Contest2 (manual trigger — wait for all other windows to be ready)
gnome-terminal --title="7 Contest2 (RUN LAST)" -- bash -c \
    "echo 'Wait for all other windows to finish starting, then press Enter to launch contest2...' && read && ${SOURCE} && ros2 run mie443_contest2 contest2; exec bash" &

echo "All windows launched."
echo ""
echo "REMINDER — Run these on the Pi (ssh ubuntu@<ip>) BEFORE pressing Enter in window 7:"
echo "  source contest2/bin/activate"
echo "  ros2 launch lerobot_moveit so101_turtlebot.launch.py     # arm bridge + TF"
echo "  ros2 run mie443_contest2 image_capture_server             # OAK-D capture service"
echo "  ros2 launch apriltag_ros camera_36h11.launch.yml          # AprilTag detection"
echo ""
echo "In RViz (window 3), use '2D Pose Estimate' to set the robot's initial position."
echo "Then press Enter in window 7 to start contest2."
