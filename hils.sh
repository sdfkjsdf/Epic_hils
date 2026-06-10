#!/usr/bin/env bash
# =============================================================================
# hils.sh - EPIC HILS manager / launcher (robust lifecycle).
#
# Orchestrates the HITL stack across two containers + the real PX4 board:
#   A = epic_onboard      : roscore + MAVROS + onboard adapter (+ EPIC)
#   B = poongsan_hils_dev : Gazebo HITL (mavlink_interface -> board serial)
#
# Automates the fiddly parts so any user gets a clean run:
#   - kill stale processes first (clean slate)
#   - reboot the board for a FRESH EKF (avoids long-session drift/divergence)
#   - start each stage only after the previous is *really* ready (heartbeat-gated)
#   - gate on EKF height convergence before declaring READY
#
# Usage:
#   ./hils.sh            interactive menu
#   ./hils.sh up         clean bring-up
#   ./hils.sh down       stop everything
#   ./hils.sh status     health of all components
#   ./hils.sh enable-hitl
#   ./hils.sh restart-sim
# =============================================================================
set -uo pipefail

A=epic_onboard
B=poongsan_hils_dev
WS=/root/poongsan_hils
UDP=14540
BOARD_DEV=/dev/ttyACM0
ROS='source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash 2>/dev/null'

green(){ printf '\033[1;32m%s\033[0m' "$*"; }
red(){   printf '\033[1;31m%s\033[0m' "$*"; }
yellow(){ printf '\033[1;33m%s\033[0m' "$*"; }

ea(){ docker exec "$A" bash -lc "$ROS; $*"; }     # exec in onboard (ROS sourced)
eb(){ docker exec "$B" bash -lc "$*"; }           # exec in sim box

ensure_up(){ docker start "$A" >/dev/null 2>&1; docker start "$B" >/dev/null 2>&1; }

# ---- functional health checks (robust against zombie processes) ----
sim_up(){ eb "python3 $WS/scripts/wait_ready.py udp $UDP 3" >/dev/null 2>&1; }
mavros_connected(){ ea 'timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q "connected: True"'; }
adapter_up(){ ea 'pgrep -f [p]x4_setpoint_adapter >/dev/null'; }
roscore_up(){ ea 'pgrep -x rosmaster >/dev/null'; }

# ---- teardown ----
stop_all(){
  echo "[down] stopping onboard (adapter/mavros/hover) + sim (gazebo) ..."
  ea 'pkill -f [t]est_hover_cmd 2>/dev/null; pkill -f [p]x4_setpoint_adapter 2>/dev/null; pkill -x mavros_node 2>/dev/null; pkill -x roslaunch 2>/dev/null' 2>/dev/null || true
  eb 'pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null' 2>/dev/null || true
  sleep 2
  echo "[down] done."
}

# ---- board ----
check_hitl(){
  if eb "python3 $WS/scripts/px4_detect.py --quiet" >/dev/null 2>&1; then return 0; fi
  echo "  $(red 'HITL not enabled on board.') run: ./hils.sh enable-hitl"; return 1
}
reboot_board(){
  echo -n "[board] rebooting for a fresh EKF ... "
  eb "python3 - <<'PY'
from pymavlink import mavutil
m=mavutil.mavlink_connection('$BOARD_DEV',baud=115200)
m.wait_heartbeat(timeout=8) and m.reboot_autopilot()
PY" >/dev/null 2>&1 || true
  sleep 3
  if eb "python3 $WS/scripts/wait_ready.py serial $BOARD_DEV 40" >/dev/null 2>&1; then
    echo "$(green ready)"; else echo "$(red TIMEOUT)"; return 1; fi
}

# ---- sim ----
start_sim(){
  echo -n "[sim] starting Gazebo HITL, waiting for udp:$UDP ... "
  eb "(HEADLESS=\${HEADLESS:-1} nohup $WS/scripts/run_hitl_gazebo.sh >$WS/.gz_hitl.log 2>&1 &)" >/dev/null 2>&1
  if eb "python3 $WS/scripts/wait_ready.py udp $UDP 45" >/dev/null 2>&1; then
    echo "$(green ok)"; else echo "$(red 'TIMEOUT (see .gz_hitl.log)')"; return 1; fi
}

# ---- onboard ----
start_onboard(){
  ea 'pgrep -x rosmaster >/dev/null || (nohup roscore >'"$WS"'/.A_roscore.log 2>&1 &)'; sleep 4
  echo -n "[onboard] mavros (udp:$UDP), waiting for connection ... "
  ea 'pgrep -x mavros_node >/dev/null || (nohup roslaunch mavros px4.launch fcu_url:="udp://:'"$UDP"'@" >'"$WS"'/.A_mavros.log 2>&1 &)'
  for i in $(seq 1 30); do mavros_connected && { echo "$(green connected)"; return 0; }; sleep 1; done
  echo "$(red TIMEOUT)"; return 1
}

wait_ekf(){
  echo -n "[ekf] waiting for height estimate to converge ... "
  for try in $(seq 1 20); do
    vals=$(ea 'for k in 1 2 3 4 5 6; do timeout 2 rostopic echo -n1 /mavros/local_position/odom/pose/pose/position/z 2>/dev/null | head -1; sleep 0.5; done')
    spread=$(echo "$vals" | awk 'NR==1{mn=mx=$1} {if($1<mn)mn=$1; if($1>mx)mx=$1} END{print mx-mn}')
    if echo "$spread" | awk '{exit !($1<0.4 && $1>=0)}'; then echo "$(green "converged (spread=${spread}m)")"; return 0; fi
    sleep 1
  done
  echo "$(red 'not converged')"; return 1
}

start_adapter(){
  echo -n "[onboard] adapter ... "
  ea 'pgrep -f px4_setpoint_adapter >/dev/null || (nohup rosrun onboard_adapter px4_setpoint_adapter.py _auto_offboard:=true _auto_arm:=false >'"$WS"'/.A_adapter.log 2>&1 &)'
  sleep 3
  adapter_up && echo "$(green up)" || { echo "$(red failed)"; return 1; }
}

cmd_up(){
  ensure_up
  stop_all
  check_hitl || return 1
  reboot_board || return 1
  start_sim || return 1
  start_onboard || return 1
  wait_ekf || echo "  $(yellow 'WARNING: EKF not converged - inspect before arming')"
  start_adapter || return 1
  echo; echo "===== $(green 'HILS READY') ====="
  cmd_status
}

cmd_status(){
  ensure_up
  echo "----- HILS status -----"
  sim_up           && echo " Sim (gazebo) : $(green up)"        || echo " Sim (gazebo) : $(red down)"
  roscore_up       && echo " roscore      : $(green up)"        || echo " roscore      : $(red down)"
  mavros_connected && echo " MAVROS       : $(green connected)" || echo " MAVROS       : $(red 'not connected')"
  adapter_up       && echo " Adapter      : $(green up)"        || echo " Adapter      : $(red down)"
  if mavros_connected; then
    ea 'timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep -E "armed:|mode:" | sed "s/^/ /"'
    z=$(ea 'timeout 3 rostopic echo -n1 /mavros/local_position/odom/pose/pose/position/z 2>/dev/null | head -1')
    echo " z(odom)      : ${z:-?}"
  fi
  echo "-----------------------"
}

cmd_enable_hitl(){ ensure_up; stop_all; eb "python3 $WS/scripts/px4_detect.py --enable-hitl" || true; }

restart_sim(){ ensure_up; eb 'pkill -x gzserver 2>/dev/null'; sleep 2; reboot_board && start_sim; }

menu(){
  while true; do
    echo; echo "========== EPIC HILS Manager =========="
    cmd_status
    echo " 1) Up      (clean bring-up: board reboot -> sim -> mavros -> adapter)"
    echo " 2) Down    (stop all)"
    echo " 3) Status"
    echo " 4) Enable HITL on board"
    echo " 5) Restart sim only"
    echo " q) Quit"
    read -rp "select> " sel
    case "$sel" in
      1) cmd_up ;; 2) stop_all ;; 3) cmd_status ;;
      4) cmd_enable_hitl ;; 5) restart_sim ;;
      q|Q) break ;; *) echo "?" ;;
    esac
  done
}

case "${1:-menu}" in
  up) cmd_up ;;
  down) ensure_up; stop_all ;;
  status) cmd_status ;;
  enable-hitl) cmd_enable_hitl ;;
  restart-sim) restart_sim ;;
  menu|"") menu ;;
  *) echo "usage: $0 {up|down|status|enable-hitl|restart-sim|menu}" ;;
esac
