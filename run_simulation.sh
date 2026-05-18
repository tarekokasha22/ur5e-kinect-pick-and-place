#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════════
#  run_simulation.sh  —  Kinect + UR5e Vision Pick-and-Place  (Simulation)
# ════════════════════════════════════════════════════════════════════════════
#  One command to start the full simulation:
#
#    bash run_simulation.sh
#
#  What it does
#  ────────────
#  Terminal 1  (this script):
#    • Sources ROS 2 Humble
#    • Builds kinect_picking (if not already built)
#    • Launches UR5e fake hardware + MoveIt2 + RViz
#        ros2 launch kinect_picking ur_fake.launch.py
#
#  Terminal 2  (auto-opened in a new gnome-terminal tab):
#    • Waits 15 s for MoveIt to come up
#    • Launches the vision pipeline (mock Kinect + detection + robot commander)
#        ros2 launch kinect_picking kinect_picking_sim.launch.py
#
#  After both are running, send pick commands with:
#    ros2 topic pub --once /voice_command std_msgs/msg/String "data: 'circle_1'"
#    ros2 topic pub --once /voice_command std_msgs/msg/String "data: 'square_1'"
#    ros2 topic pub --once /voice_command std_msgs/msg/String "data: 'triangle_1'"
#
# ════════════════════════════════════════════════════════════════════════════

set -e

# ── Paths ──────────────────────────────────────────────────────────────── #
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/kinect_ws_final (1)/kinect_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

# ── Python 3.13/3.10 fix ───────────────────────────────────────────────── #
# ROS 2 Humble requires Python 3.10; system python3 may be 3.13+.
# Use python3.10 explicitly so ros2 CLI and colcon work correctly.
ROS2="python3.10 /opt/ros/humble/bin/ros2"

# ── Colour helpers ─────────────────────────────────────────────────────── #
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'
YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'

info()    { echo -e "${CYAN}[SIM]${NC} $*"; }
success() { echo -e "${GREEN}[SIM]${NC} $*"; }
warn()    { echo -e "${YELLOW}[SIM]${NC} $*"; }
err()     { echo -e "${RED}[SIM] ERROR:${NC} $*"; exit 1; }

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║   Kinect 360 + UR5e  ·  Full Simulation Launcher    ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════╝${NC}"
echo ""

# ── Sanity checks ──────────────────────────────────────────────────────── #
[[ -f "${ROS_SETUP}" ]]  || err "ROS 2 Humble not found at ${ROS_SETUP}"
[[ -d "${WS_DIR}" ]]     || err "Workspace not found at:\n  ${WS_DIR}"

# ── Source ROS 2 ───────────────────────────────────────────────────────── #
# shellcheck disable=SC1090
source "${ROS_SETUP}"
info "ROS 2 Humble sourced"

# ── Build workspace (only if install/ is missing or package changed) ───── #
info "Building kinect_picking …"
cd "${WS_DIR}"
python3.10 -m colcon build --packages-select kinect_picking \
    --symlink-install 2>&1 | tail -5
success "Build complete"

# ── Source the workspace overlay ───────────────────────────────────────── #
# shellcheck disable=SC1090
source "${WS_SETUP}"
info "Workspace overlay sourced"

# ── Cleanup on Ctrl-C ─────────────────────────────────────────────────── #
cleanup() {
    echo ""
    warn "Caught interrupt — shutting down all nodes …"
    # Kill the vision pipeline terminal if we opened it
    if [[ -n "${VISION_PID}" ]]; then
        kill "${VISION_PID}" 2>/dev/null || true
    fi
    # Kill any stray ros2 nodes
    pkill -f "ros2 launch kinect_picking" 2>/dev/null || true
    pkill -f "ros2 launch ur_robot_driver" 2>/dev/null || true
    pkill -f "ros2 launch ur_moveit_config" 2>/dev/null || true
    pkill -f "kinect_picking" 2>/dev/null || true
    pkill -f "mock_kinect_node\|detection_node\|depth_extractor\|coord_transform\|robot_commander" 2>/dev/null || true
    success "Cleanup done. Goodbye!"
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── Launch vision pipeline in a second terminal ────────────────────────── #
VISION_CMD="bash -c \"\
    source ${ROS_SETUP} && \
    source ${WS_SETUP} && \
    echo '' && \
    echo '  ⏳  Waiting 20 s for MoveIt2 + controllers to start …' && \
    sleep 20 && \
    echo '  🚀  Starting vision pipeline …' && \
    python3.10 /opt/ros/humble/bin/ros2 launch kinect_picking kinect_picking_sim.launch.py; \
    exec bash\""

# Try gnome-terminal first, fall back to xterm, then just background
if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="Vision Pipeline" -- bash -c "${VISION_CMD}"
    info "Vision pipeline terminal opened (gnome-terminal)"
elif command -v xterm &>/dev/null; then
    xterm -title "Vision Pipeline" -e "${VISION_CMD}" &
    VISION_PID=$!
    info "Vision pipeline terminal opened (xterm, PID ${VISION_PID})"
else
    warn "No terminal emulator found — launching vision pipeline in background"
    bash -c "source ${ROS_SETUP} && source ${WS_SETUP} && sleep 15 && \
             python3.10 /opt/ros/humble/bin/ros2 launch kinect_picking kinect_picking_sim.launch.py" &
    VISION_PID=$!
    info "Vision pipeline running in background (PID ${VISION_PID})"
fi

echo ""
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  Launching UR5e (fake hardware) + MoveIt2 + RViz …${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}RViz will open in ~10-15 seconds.${NC}"
echo -e "  ${CYAN}Vision pipeline starts automatically in a new window.${NC}"
echo ""
echo -e "  ${BOLD}Send pick commands (new terminal, after setup):${NC}"
echo -e "  source ${WS_SETUP}"
echo -e "  ${YELLOW}python3.10 /opt/ros/humble/bin/ros2 topic pub --once /voice_command std_msgs/msg/String \"data: 'circle_1'\"${NC}"
echo -e "  ${YELLOW}python3.10 /opt/ros/humble/bin/ros2 topic pub --once /voice_command std_msgs/msg/String \"data: 'square_1'\"${NC}"
echo -e "  ${YELLOW}python3.10 /opt/ros/humble/bin/ros2 topic pub --once /voice_command std_msgs/msg/String \"data: 'triangle_1'\"${NC}"
echo ""
echo -e "  Press ${BOLD}Ctrl-C${NC} to stop everything."
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# ── Start UR control + MoveIt2 (blocks until Ctrl-C) ──────────────────── #
${ROS2} launch kinect_picking ur_fake.launch.py
