#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
#  run_full_system.sh — Kinect 360 + UR5e Pick-and-Place
# ═══════════════════════════════════════════════════════════════════════════
#  Usage:  ./run_full_system.sh [ROBOT_IP]
#  Default robot IP: 192.168.1.102
# ═══════════════════════════════════════════════════════════════════════════

ROBOT_IP="${1:-192.168.1.102}"
ROBOT_TYPE="ur5e"
CAM_TX="0.0"
CAM_TY="0.0"
CAM_TZ="0.0"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UR_WS="$SCRIPT_DIR/ur_driver"
KINECT_WS="$SCRIPT_DIR/kinect_ws_final (1)/kinect_ws"

# Always use gnome-terminal.real (the working binary, not the broken Python wrapper)
TERM="/usr/bin/gnome-terminal.real"

echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║         Kinect 360 + UR5e  Pick-and-Place System                ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  Robot IP     : $ROBOT_IP"
echo "║  Terminal     : $TERM"
echo "║  Camera offset: X=$CAM_TX  Y=$CAM_TY  Z=$CAM_TZ"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

if [ ! -f "$UR_WS/install/setup.bash" ]; then
    echo "[ERROR] ur_driver not built. Run: cd \"$UR_WS\" && python3.10 -m colcon build"
    exit 1
fi
if [ ! -f "$KINECT_WS/install/setup.bash" ]; then
    echo "[ERROR] kinect_ws not built. Run: cd \"$KINECT_WS\" && python3.10 -m colcon build"
    exit 1
fi

# Shared source string used in every terminal
SRC="source /opt/ros/humble/setup.bash \
  && source '$UR_WS/install/setup.bash' \
  && source '$KINECT_WS/install/setup.bash'"

ROS2="python3.10 /opt/ros/humble/bin/ros2"

# ── Terminal 1: UR5e Robot Driver ────────────────────────────────────────
echo "[1/3] Opening UR5e Robot Driver terminal..."
"$TERM" --title="[1] UR5e Driver" -- bash -c \
  "eval $SRC
   echo ''
   echo '=== Terminal 1: UR5e Robot Driver ==='
   echo \"Connecting to robot at $ROBOT_IP ...\"
   echo ''
   $ROS2 launch ur_robot_driver ur_control.launch.py \
       ur_type:=$ROBOT_TYPE robot_ip:=$ROBOT_IP launch_rviz:=false
   exec bash" &

echo "    Waiting 8 s for UR driver to come up..."
sleep 8

# ── Terminal 2: MoveIt2 ──────────────────────────────────────────────────
echo "[2/3] Opening MoveIt2 terminal..."
"$TERM" --title="[2] MoveIt2" -- bash -c \
  "eval $SRC
   echo ''
   echo '=== Terminal 2: MoveIt2 Motion Planning ==='
   echo ''
   $ROS2 launch ur_moveit_config ur_moveit.launch.py \
       ur_type:=$ROBOT_TYPE launch_rviz:=true
   exec bash" &

echo "    Waiting 10 s for MoveIt2 to be ready..."
sleep 10

# ── Terminal 3: Kinect Vision Pipeline + Robot Commander ─────────────────
echo "[3/3] Opening Kinect + Commander terminal..."
"$TERM" --title="[3] Kinect + Commander" -- bash -c \
  "eval $SRC
   echo ''
   echo '=== Terminal 3: Kinect Vision + Robot Commander ==='
   echo ''
   $ROS2 launch kinect_picking kinect_picking.launch.py \
       cam_tx:=$CAM_TX cam_ty:=$CAM_TY cam_tz:=$CAM_TZ
   exec bash" &

echo ""
echo "✅  All 3 terminals opened successfully."
echo ""
echo "┌─── How to use ──────────────────────────────────────────────────┐"
echo "│ 1. Place pink/green shapes in front of the Kinect.              │"
echo "│ 2. Wait for the Detection window to label them.                 │"
echo "│ 3. Say: 'circle one' / 'square two' / 'triangle three' etc.    │"
echo "│    → Target highlights cyan. UR5e picks & drops it.            │"
echo "│ 4. Say a new command to pick another shape.                     │"
echo "└─────────────────────────────────────────────────────────────────┘"
wait
