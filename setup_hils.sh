#!/usr/bin/env bash
# =============================================================================
# poongsan HILS (container B) setup script
#  - Base: osrf/ros:noetic-desktop-full (Ubuntu 20.04 + ROS Noetic + Gazebo 11)
#  - Run inside the dev container poongsan_hils_dev; accumulate steps here so this
#    file can be transcribed directly into a Dockerfile (RUN lines).
#  - Idempotent: safe to re-run.
#  - Scope: trajectory-tracking tuning only. No Livox LiDAR sim / no exploration
#    world (point cloud is reused from poongsan's MARSIM in live mode).
# =============================================================================
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# [1] Base tools                                              [done 2026-06-07]
# -----------------------------------------------------------------------------
apt-get update
apt-get install -y --no-install-recommends \
    git wget curl python3-pip build-essential

# -----------------------------------------------------------------------------
# [2] MAVROS (Noetic) + GeographicLib geoid datasets          [done 2026-06-07]
#     Core of EPIC /position_cmd -> setpoint and /mavros/.../odom feedback
# -----------------------------------------------------------------------------
apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras
wget -q -O /tmp/install_geographiclib_datasets.sh \
    https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
bash /tmp/install_geographiclib_datasets.sh   # egm96 etc.; skips if already present

# -----------------------------------------------------------------------------
# [3] PX4 (firmware v1.16) - PC-side gazebo-classic sim plugins/models
#     Real-board HITL & PX4 SITL share the same gazebo-classic plugins/models.
# -----------------------------------------------------------------------------
# 3a) clone (shallow, with submodules)                        [done 2026-06-07]
cd /root/poongsan_hils
[ -d PX4-Autopilot ] || git clone -b v1.16.0 --recursive --depth 1 --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git

# 3b) dependencies (needs sudo / --no-nuttx skips board firmware toolchain;
#     on focal this installs gazebo11 + libgazebo11-dev)
apt-get install -y sudo
cd /root/poongsan_hils/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx

# 3c) build only the gazebo-classic plugins (HITL direct -> skip px4_sitl)  [done 2026-06-07]
SG=/root/poongsan_hils/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
mkdir -p "$SG/build"
cd "$SG/build"
cmake ..
make -j"$(nproc)"
# Output: 40 .so in build/ (libgazebo_mavlink_interface.so etc.).
# HITL assets: worlds/hitl_iris.world, models/iris_hitl
# Runtime env (exported by scripts/run_hitl_gazebo.sh):
#   export GAZEBO_PLUGIN_PATH=$SG/build:$GAZEBO_PLUGIN_PATH
#   export GAZEBO_MODEL_PATH=$SG/models:$GAZEBO_MODEL_PATH

# -----------------------------------------------------------------------------
# [4] Gazebo dynamics plant
#     Gazebo 11 is already in the base image (desktop-full).
#     Vehicle = iris (PX4 default). Mass/thrust/inertia are SDF numbers, swap later.
#     NOTE: Livox LiDAR sim is out of scope (tracking tuning needs no point cloud).
# -----------------------------------------------------------------------------
# (no extra install)

# -----------------------------------------------------------------------------
# [5] HITL launch + bridge node
# -----------------------------------------------------------------------------
# 5a) HITL Gazebo launch = scripts/run_hitl_gazebo.sh  [HIL link verified 2026-06-07]
#     - mavlink_interface plugin owns the board serial (/dev/ttyACM0@921600, hil_mode=1)
#       and forwards MAVLink to udp:14540 (MAVROS) and udp:14550 (QGC) -> no mavlink-router.
#     - Board HITL check/enable: scripts/px4_detect.py (needs SYS_HITL=1)
#     - Verified: PX4 heartbeat (HIL=True) received on udp:14540.
#     NOTE: use 'docker run --init' for the final image to reap zombies.
# 5b) MAVROS connect: roslaunch mavros px4.launch fcu_url:="udp://:14540@"   [verified 2026-06-07]
#     -> /mavros/state (connected:True), /mavros/local_position/odom publishing.
# 5c) bridge node (/position_cmd -> /mavros/setpoint_raw/local) + offboard/arm        [TODO]
# 5d) odom remap (/mavros/local_position/odom -> /lidar_slam/odom) + frame align      [TODO]

# -----------------------------------------------------------------------------
# [6] HITL actuator-output module guard (pwm_out_sim)         [diagnosed 2026-06-08]
# -----------------------------------------------------------------------------
# Root cause of "arms in OFFBOARD but the sim drone never moves":
#   PX4 sends HIL_ACTUATOR_CONTROLS from the actuator_outputs_sim topic
#   (src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp), which ONLY the
#   pwm_out_sim module publishes (PWMSim.hpp). That module is a build-time option
#   and was NOT in the micoair_h743-v2 firmware -> nsh `pwm_out_sim` = command not
#   found -> no actuator output -> Gazebo motors never spin. (Official PX4 HITL
#   guide documents exactly this: docs.px4.io/main/en/simulation/hitl)
#
#   Self-service guard added: scripts/check_hil_output.py
#     - auto-detects the board, runs `pwm_out_sim status` over MAVLink nsh,
#     - if missing, prints the official remedy + doc/GitHub links,
#     - `--fix` patches the board .px4board and (if arm-none-eabi-gcc present)
#       rebuilds + flashes.
#     - wired into hils_menu.py as a preflight step (before the simulator starts).
#
#   FIRMWARE FIX (official, PX4-documented):
#     add the key to the board config:
#       boards/micoair/h743-v2/default.px4board:  CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
#     then rebuild + flash:
#       cd PX4-Autopilot && make micoair_h743-v2 upload   [needs ARM toolchain]
#     airframe must be a HIL airframe (SYS_AUTOSTART=1001, "HIL Quadcopter X").
# -----------------------------------------------------------------------------
# [7] PX4 NuttX firmware build toolchain (for micoair_h743-v2) [done 2026-06-08]
# -----------------------------------------------------------------------------
# Base image (Ubuntu 20.04) ships no arm-none-eabi-gcc, and the focal apt one
# (9.2.1) is too old for PX4 v1.16 (cmake wants >9.3). Use the exact toolchain
# PX4/NuttX CI pins (platforms/nuttx/NuttX/nuttx/tools/ci/cibuild.sh): 10.3-2021.10.
apt-get install -y --no-install-recommends genromfs
pip3 install pyros-genmsg              # provides the 'genmsg' module the build imports
cd /opt && \
  wget --quiet https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 && \
  tar jxf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 && \
  mv gcc-arm-none-eabi-10.3-2021.10 gcc-arm-none-eabi && \
  rm gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
export PATH=/opt/gcc-arm-none-eabi/bin:$PATH   # add to dev_bashrc.sh for the image
# The NuttX submodule shipped with NO git tags -> px_update_git_header.py throws
# IndexError (re.findall(nuttx-X.Y.Z)[-1]). Fetch the tags once:
git -C /root/poongsan_hils/PX4-Autopilot/platforms/nuttx/NuttX/nuttx fetch --tags
# Build the HIL-enabled firmware (board config already has pwm_out_sim, see [6]):
#   cd PX4-Autopilot && make micoair_h743-v2
#   -> build/micoair_h743-v2_default/micoair_h743-v2_default.px4  (flash this)

# -----------------------------------------------------------------------------
# [8] Flash + airframe -> DRONE FLIES IN HITL                 [SOLVED 2026-06-08]
# -----------------------------------------------------------------------------
# Flash the built firmware (board on USB). px_uploader can't auto-enter the
# bootloader on this board (57600 reboot + MAVLink reboot-to-bootloader both
# leave it in app mode). Reliable method: run upload, then PHYSICALLY unplug+
# replug the USB while it prints "Attempting reboot..." (container is privileged
# + /dev:/dev so re-enumeration is seen) -> it catches the BL window:
#   cd PX4-Autopilot && make micoair_h743-v2 upload   # then unplug/replug USB
#   -> Program 100% / Verify 100% / Reboot.  pwm_out_sim now exists on the board.
#
# Then the OUTPUT-stage mapping (HIL_ACT_FUNCx) must point to the motors, else
# pwm_out_sim has nothing to emit ("cycle: 0 events"). Do it the official way by
# selecting the HIL airframe (sets HIL_ACT_FUNC1..4=101..104 + SYS_HITL=1 +
# CA_ROTOR geometry + CBRK_SUPPLY_CHK):
#   nsh> param set SYS_AUTOSTART 1001   # HIL Quadcopter X
#   nsh> param save ; reboot
# Verify after reboot: HIL_ACT_FUNC1..4 = 101..104, SYS_AUTOSTART=1001.
# RESULT: hils_menu.py gazebo_truth -> Takeoff -> Gazebo drone climbs to ~1.6 m
# and hovers (gz_z tracks the 1.5 m setpoint). Blocker resolved.

echo "[setup_hils] [1][2][3] done, [5a] verified, [6] HIL guard, [7] toolchain+build, [8] flash+1001=FLIES."
