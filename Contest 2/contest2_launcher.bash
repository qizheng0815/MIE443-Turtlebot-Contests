#!/bin/bash
# ==============================================================================
# Contest 2 Simulation Launcher
#
# Builds contest2, then opens each component in its own Terminator window.
#
# What this script does:
#   1. colcon build  (mie443_contest2)
#   2. Gazebo              (empty world + SO-ARM101 model)
#   3. ROS 2 Controllers   (joint_state_broadcaster, arm, gripper in Gazebo)
#   4. MoveIt + RViz       (use_sim_time:=True)
#   5. Nav2 localisation   (map server + AMCL, use_sim_time:=True)
#   6. Nav2 navigation     (planners + controllers + bt_navigator, use_sim_time:=True)
#   7. YOLO detector
#   8. contest2            (use_sim_time:=true)
#
# Usage:
#   bash ~/ros2_ws/contest2_launcher.bash
#   bash ~/ros2_ws/contest2_launcher.bash --no-rviz
#   bash ~/ros2_ws/contest2_launcher.bash --map /path/to/your_map.yaml
#   bash ~/ros2_ws/contest2_launcher.bash --skip-build   # skip colcon build step
# ==============================================================================

WORKSPACE_DIR="$HOME/ros2_ws"
LAUNCH_RVIZ=true
SKIP_BUILD=false
MAP_FILE="${WORKSPACE_DIR}/src/mie443_contest1/mie443_contest1/worlds/practice_scene1.yaml"

# ── Parse flags ────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-rviz)    LAUNCH_RVIZ=false;  shift ;;
        --skip-build) SKIP_BUILD=true;    shift ;;
        --map)        MAP_FILE="$2";      shift 2 ;;
        *) echo "Unknown argument: $1  (valid: --no-rviz  --skip-build  --map <path>)"; exit 1 ;;
    esac
done

# ── Detect ROS distro ──────────────────────────────────────────────────────────
if [ -n "$ROS_DISTRO" ]; then
    DISTRO="$ROS_DISTRO"
elif [ -d "/opt/ros/jazzy" ]; then
    DISTRO="jazzy"
elif [ -d "/opt/ros/humble" ]; then
    DISTRO="humble"
else
    echo "ERROR: Could not detect ROS distro. Source your ROS setup first:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

SOURCE_CMD="source /opt/ros/${DISTRO}/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash"

# ── Sanity checks ──────────────────────────────────────────────────────────────
if ! command -v terminator &>/dev/null; then
    echo "ERROR: terminator not found.  sudo apt install terminator"
    exit 1
fi

if ! command -v colcon &>/dev/null; then
    echo "ERROR: colcon not found.  sudo apt install python3-colcon-common-extensions"
    exit 1
fi

if [ ! -f "${MAP_FILE}" ]; then
    echo "ERROR: Map file not found: ${MAP_FILE}"
    echo "  Override with:  --map /path/to/map.yaml"
    exit 1
fi

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║              Contest 2 – Simulation Launcher                   ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
printf  "║  ROS distro : %-51s║\n" "${DISTRO}"
printf  "║  Workspace  : %-51s║\n" "${WORKSPACE_DIR}"
printf  "║  Map        : %-51s║\n" "$(basename ${MAP_FILE})"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

# ── Step 1: colcon build ───────────────────────────────────────────────────────
if [ "$SKIP_BUILD" = false ]; then
    echo "──────────────────────────────────────────────────────────────────"
    echo "  Building mie443_contest2..."
    echo "──────────────────────────────────────────────────────────────────"

    cd "${WORKSPACE_DIR}" || { echo "ERROR: Cannot cd to ${WORKSPACE_DIR}"; exit 1; }

    source "/opt/ros/${DISTRO}/setup.bash"
    colcon build --packages-select mie443_contest2

    if [ $? -ne 0 ]; then
        echo ""
        echo "ERROR: colcon build failed. Fix the errors above and try again."
        exit 1
    fi

    echo ""
    echo "  Build successful. Sourcing install..."
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo ""
else
    echo "  Skipping build (--skip-build flag set)."
    if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
        echo "ERROR: Workspace install not found. Remove --skip-build and rebuild."
        exit 1
    fi
    echo ""
fi

# ── Verify contest2 binary exists after build ──────────────────────────────────
if [ ! -f "${WORKSPACE_DIR}/install/mie443_contest2/lib/mie443_contest2/contest2" ]; then
    echo "ERROR: contest2 executable not found after build."
    echo "  Check for compile errors above."
    exit 1
fi

read -rp "  Press ENTER to launch all components, or Ctrl+C to cancel..."
echo ""

# ── Helper: open a named Terminator window ─────────────────────────────────────
open_term() {
    local title="$1"
    local cmd="$2"
    terminator \
        --title="[Contest2] ${title}" \
        -e "bash -c '${SOURCE_CMD} && echo && echo === ${title} === && echo && ${cmd}; echo; echo --- ${title} has exited \(press Enter to close\) ---; read'" &
    sleep 0.4
}

# ── Step 2: Gazebo ────────────────────────────────────────────────────────────
echo "  [1/7] Starting Gazebo (SO-ARM101 in empty world)..."
open_term "Gazebo" \
    "ros2 launch lerobot_description so101_gazebo.launch.py"

echo "  Waiting 8 seconds for Gazebo to start..."
sleep 8

# ── Step 3: ROS 2 Controllers ─────────────────────────────────────────────────
echo "  [2/7] Spawning ROS 2 controllers into Gazebo..."
open_term "ROS2 Controllers" \
    "ros2 launch lerobot_controller so101_controller.launch.py is_sim:=True"

sleep 3

# ── Step 4: MoveIt move_group (+ optional RViz) ───────────────────────────────
echo "  [3/7] Starting MoveIt (sim time)..."
if [ "$LAUNCH_RVIZ" = true ]; then
    open_term "MoveIt + RViz" \
        "ros2 launch lerobot_moveit so101_moveit.launch.py is_sim:=True"
else
    open_term "MoveIt move_group" \
        "ros2 run moveit_ros_move_group move_group \
         --ros-args --log-level info -p use_sim_time:=true"
fi

echo "  Waiting 6 seconds for MoveIt to initialise..."
sleep 6

# ── Step 5: Nav2 localisation (map server + AMCL) ────────────────────────────
echo "  [4/7] Starting Nav2 localisation..."
open_term "Nav2 Localisation" \
    "ros2 launch nav2_bringup localization_launch.py \
     map:=${MAP_FILE} use_sim_time:=True"

echo "  Waiting 5 seconds for AMCL and map server..."
sleep 5

# ── Step 6: Nav2 navigation stack ────────────────────────────────────────────
echo "  [5/7] Starting Nav2 navigation stack..."
open_term "Nav2 Navigation" \
    "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True"

sleep 5

# ── Step 7: YOLO detector ────────────────────────────────────────────────────
echo "  [6/7] Starting YOLO detector..."
open_term "YOLO Detector" \
    "ros2 run mie443_contest2 yolo_detector.py"

sleep 1

# ── Step 8: contest2 ─────────────────────────────────────────────────────────
echo "  [7/7] Starting contest2..."
open_term "Contest2" \
    "ros2 run mie443_contest2 contest2 --ros-args -p use_sim_time:=true"

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "  All components launched in separate Terminator windows."
echo ""
echo "  REMINDER: Use RViz or the Nav2 2D Pose Estimate tool to set the"
echo "  robot's initial position on the map before running the contest."
echo ""
echo "  To stop everything: Ctrl+C in each Terminator window."
echo ""
