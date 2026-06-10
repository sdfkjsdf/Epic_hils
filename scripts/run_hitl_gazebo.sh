#!/usr/bin/env bash
# =============================================================================
# run_hitl_gazebo.sh - Launch the Gazebo HITL world that connects to the real
#                      PX4 board over USB serial.
#
# The mavlink_interface plugin in the iris_hitl model:
#   - owns the board serial  : /dev/ttyACM0 @ 921600  (hil_mode=1)
#   - forwards MAVLink to QGC : udp 14550
#   - forwards MAVLink to SDK : udp 14540   <-- connect MAVROS to udp://:14540@
# So no mavlink-router is needed.
#
# Usage:
#   ./run_hitl_gazebo.sh            # GUI
#   HEADLESS=1 ./run_hitl_gazebo.sh # gzserver only (no GUI)
# =============================================================================
set -e

PX4=/root/poongsan_hils/PX4-Autopilot
SG="$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
HERE="$(cd "$(dirname "$0")" && pwd)"

export GAZEBO_PLUGIN_PATH="$SG/build:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$SG/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$SG/worlds:${GAZEBO_RESOURCE_PATH:-}"
source /usr/share/gazebo/setup.sh 2>/dev/null || true

# Auto-detect the PX4 board device (works regardless of ttyACM number) + confirm HITL.
BOARD_DEV="$(python3 "$HERE/px4_detect.py" --quiet 2>/dev/null)"; rc=$?
if [ -z "$BOARD_DEV" ] || [ "$rc" = "2" ]; then
    echo "[ERROR] PX4 board not found. Plug in USB."; exit 1
fi
if [ "$rc" = "3" ]; then
    echo "[ERROR] HITL not enabled. Run: python3 $HERE/px4_detect.py --enable-hitl"; exit 1
fi
BOARD_DEV="$(readlink -f "$BOARD_DEV")"   # resolve by-id symlink -> /dev/ttyACMx

# Patch the HITL model's serial device to the detected port (port-agnostic).
SDF="$SG/models/iris_hitl/iris_hitl.sdf"
sed -i -E "s#<serialDevice>[^<]*</serialDevice>#<serialDevice>${BOARD_DEV}</serialDevice>#" "$SDF"

WORLD="$SG/worlds/hitl_iris.world"
echo "[*] Gazebo HITL: world=$WORLD  board=${BOARD_DEV}@921600  (MAVROS->udp:14540, QGC->udp:14550)"

# Load the gazebo_ros api plugin so /gazebo/model_states is published (ground-truth
# for SIM mode). Requires roscore to be running already.
source /opt/ros/noetic/setup.bash 2>/dev/null || true
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
ROS_PLUGIN="-s libgazebo_ros_api_plugin.so"

if [ "${HEADLESS:-0}" = "1" ]; then
    exec gzserver --verbose $ROS_PLUGIN "$WORLD"
else
    exec gazebo --verbose $ROS_PLUGIN "$WORLD"
fi
