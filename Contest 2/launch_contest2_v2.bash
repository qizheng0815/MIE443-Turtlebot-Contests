#!/bin/bash
###############################################################################
# MIE443 Contest 2 - Full Launch Script
# Builds the workspace, sources the environment, then launches all necessary
# ROS 2 nodes in separate terminal windows.
#
# Usage:
#   chmod +x launch_contest2.bash
#   ./launch_contest2.bash
#
# Optional override:
#   ./launch_contest2.bash /custom/path/to/map.yaml
###############################################################################

set -e

# ── Configuration ──────────────────────────────────────────────────────────────
ROS2_WS="/home/turtlebot/ros2_ws"
MAP_DIR="$ROS2_WS/src/mie443_contest2/mie443_contest2/maps"
YOLO_SCRIPT_PATH="$(dirname "$(readlink -f "$0")")/yolo_detector.py"
CONTEST2_PKG="mie443_contest2"
CONTEST2_EXEC="contest2"

DELAY_SHORT=3
DELAY_LONG=10

# Source command that each terminal will run before its main process
SOURCE_CMD="source ~/.bashrc && source $ROS2_WS/install/setup.bash"

# ── Helper ─────────────────────────────────────────────────────────────────────
launch_in_terminal() {
    local title="$1"
    shift
    local cmd="$*"

    # Prepend sourcing into every terminal so ROS 2 packages are found
    local full_cmd="$SOURCE_CMD && $cmd"

    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="$title" -- bash -c "$full_cmd; echo ''; echo '[$title] Process exited. Press Enter to close.'; read" &
    elif command -v xterm &>/dev/null; then
        xterm -T "$title" -e bash -c "$full_cmd; echo ''; echo '[$title] Process exited. Press Enter to close.'; read" &
    else
        echo "[WARN] No graphical terminal found. Launching in background: $title"
        bash -c "$full_cmd" &
    fi
}

# ══════════════════════════════════════════════════════════════════════════════
# PRE-FLIGHT: Source & Build
# ══════════════════════════════════════════════════════════════════════════════
echo "=============================================="
echo "  MIE443 Contest 2 - Launch Script"
echo "=============================================="
echo ""

# --- Step 0a: Source bashrc ---
echo "[0a] Sourcing ~/.bashrc ..."
source ~/.bashrc
echo "     Done."
echo ""

# --- Step 0b: colcon build + source install ---
echo "[0b] Running colcon build in $ROS2_WS ..."
cd "$ROS2_WS"
colcon build --symlink-install
echo "     Build complete."
echo ""

echo "[0c] Sourcing install/setup.bash ..."
source install/setup.bash
echo "     Done."
echo ""

# ── Resolve map file ──────────────────────────────────────────────────────────
if [ -n "$1" ]; then
    MAP_PATH="$1"
else
    # Auto-detect: pick the first .yaml file in the maps directory
    if [ -d "$MAP_DIR" ]; then
        MAP_PATH="$(find "$MAP_DIR" -maxdepth 1 -name '*.yaml' | head -n 1)"
    fi

    if [ -z "$MAP_PATH" ]; then
        echo "[WARN] No .yaml map found in $MAP_DIR"
        echo "       Falling back to default warehouse map."
        MAP_PATH="/opt/ros/jazzy/share/turtlebot4_navigation/maps/warehouse.yaml"
    fi
fi

# ── Resolve YOLO script ──────────────────────────────────────────────────────
if [ ! -f "$YOLO_SCRIPT_PATH" ]; then
    ALT_YOLO="$ROS2_WS/src/$CONTEST2_PKG/$CONTEST2_PKG/yolo_detector.py"
    if [ -f "$ALT_YOLO" ]; then
        YOLO_SCRIPT_PATH="$ALT_YOLO"
    fi
fi

echo "  Workspace   : $ROS2_WS"
echo "  Map path    : $MAP_PATH"
echo "  YOLO script : $YOLO_SCRIPT_PATH"
echo ""

if [ ! -f "$MAP_PATH" ]; then
    echo "[WARN] Map file does not exist at $MAP_PATH — navigation may fail."
fi
if [ ! -f "$YOLO_SCRIPT_PATH" ]; then
    echo "[WARN] YOLO script not found at $YOLO_SCRIPT_PATH"
fi

echo ""
echo "Launching all nodes in 3 seconds... (Ctrl+C to cancel)"
sleep 3

# ══════════════════════════════════════════════════════════════════════════════
# STEP 1 — Gazebo: Simulated TurtleBot 4
# ══════════════════════════════════════════════════════════════════════════════
echo "[1/9] Launching TurtleBot 4 in Gazebo..."
launch_in_terminal "Gazebo - TurtleBot4" \
    "ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite"
sleep $DELAY_LONG

# ══════════════════════════════════════════════════════════════════════════════
# STEP 2 — AMCL Localization
# ══════════════════════════════════════════════════════════════════════════════
echo "[2/9] Launching AMCL Localization..."
launch_in_terminal "AMCL Localization" \
    "ros2 launch turtlebot4_navigation localization.launch.py map:=$MAP_PATH"
sleep $DELAY_LONG

# ══════════════════════════════════════════════════════════════════════════════
# STEP 3 — Nav2 Navigation Stack
# ══════════════════════════════════════════════════════════════════════════════
echo "[3/9] Launching Nav2 Navigation Stack..."
launch_in_terminal "Nav2" \
    "ros2 launch turtlebot4_navigation nav2.launch.py"
sleep $DELAY_LONG

# ══════════════════════════════════════════════════════════════════════════════
# STEP 4 — RViz2 Visualization
# ══════════════════════════════════════════════════════════════════════════════
echo "[4/9] Launching RViz2 (Navigation View)..."
launch_in_terminal "RViz2 - Navigation" \
    "ros2 launch turtlebot4_viz view_navigation.launch.py"
sleep $DELAY_SHORT

# ══════════════════════════════════════════════════════════════════════════════
# STEP 5 — SO-ARM 101: Gazebo Simulation
# ══════════════════════════════════════════════════════════════════════════════
echo "[5/9] Launching SO-ARM 101 in Gazebo..."
launch_in_terminal "SO-ARM Gazebo" \
    "ros2 launch lerobot_description so101_gazebo.launch.py"
sleep $DELAY_LONG

# ══════════════════════════════════════════════════════════════════════════════
# STEP 6 — SO-ARM 101: Joint Controllers
# ══════════════════════════════════════════════════════════════════════════════
echo "[6/9] Launching SO-ARM 101 Controller..."
launch_in_terminal "SO-ARM Controller" \
    "ros2 launch lerobot_controller so101_controller.launch.py"
sleep $DELAY_SHORT

# ══════════════════════════════════════════════════════════════════════════════
# STEP 7 — MoveIt 2 (Motion Planning for SO-ARM 101)
# ══════════════════════════════════════════════════════════════════════════════
echo "[7/9] Launching MoveIt 2 for SO-ARM 101..."
launch_in_terminal "MoveIt2 - SO-ARM" \
    "ros2 launch lerobot_moveit so101_moveit.launch.py"
sleep $DELAY_LONG

# ══════════════════════════════════════════════════════════════════════════════
# STEP 8 — YOLO Object Detection Service
# ══════════════════════════════════════════════════════════════════════════════
echo "[8/9] Launching YOLO Detector..."
launch_in_terminal "YOLO Detector" \
    "python3 $YOLO_SCRIPT_PATH"
sleep $DELAY_SHORT

# ══════════════════════════════════════════════════════════════════════════════
# STEP 9 — Contest 2 Main Node
# ══════════════════════════════════════════════════════════════════════════════
echo "[9/9] Launching Contest 2 Node..."
launch_in_terminal "Contest2 Node" \
    "ros2 run $CONTEST2_PKG $CONTEST2_EXEC"

# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "=============================================="
echo "  All 9 nodes launched successfully!"
echo "=============================================="
echo ""
echo "  Terminals opened:"
echo "    1. Gazebo   — TurtleBot 4 Simulation"
echo "    2. AMCL     — Localization (particle filter)"
echo "    3. Nav2     — Navigation Stack"
echo "    4. RViz2    — Visualization"
echo "    5. SO-ARM   — Gazebo Arm Simulation"
echo "    6. SO-ARM   — Joint Controllers"
echo "    7. MoveIt 2 — Motion Planning (SO-ARM 101)"
echo "    8. YOLO     — Object Detection Service"
echo "    9. Contest2 — Main State Machine"
echo ""
echo "  REMINDERS:"
echo "    - In RViz2, click '2D Pose Estimate' to initialize AMCL."
echo "    - Wait for the particle filter to converge before navigation."
echo "    - In MoveIt RViz2: Add 'MotionPlanning', set planner to 'ompl'."
echo "    - Arm poses in contest2.cpp are placeholders — tune for your setup."
echo ""
echo "  To stop everything:"
echo "    pkill -f 'ros2|gazebo|rviz2|yolo_detector'"
echo ""
