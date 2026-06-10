# EPIC HILS - Session Checkpoint (2026-06-07)

Goal: validate that the drone tracks EPIC's output trajectory before real flight,
using HITL (real PX4 board + Gazebo). Separate guidance error (EPIC) from
estimation error (LiDAR odom, done by another institution).

## Architecture
- **A = epic_onboard** (docker, --network host): roscore + MAVROS + onboard adapter (+ EPIC).
  The real onboard stack; deploys to the drone. ROS side = ENU.
- **B = poongsan_hils_dev** (docker, --network host): Gazebo HITL (gazebo-classic) + PX4
  gazebo plugins. The simulator/plant. MAVLink/UDP only (+ gazebo_ros for ground-truth).
- **Real board**: MicoAir743v2 (STM32H743), PX4 v1.16, USB serial
  (/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00, = some /dev/ttyACMx).
- Data path: adapter -> /mavros/setpoint_raw/local -> MAVROS -> udp:14540 ->
  gazebo mavlink_interface -> board (HITL). Gazebo forwards MAVLink to udp:14540 (MAVROS)
  and udp:14550 (QGC/tools). No mavlink-router needed.

## Odometry modes (variable separation)
- **gazebo_truth**: Gazebo ground-truth pose -> board EKF (external vision) + EPIC.
  No estimation error -> isolates EPIC guidance + PX4 tracking.
- **onboard_odom**: external LiDAR odom (other institution) -> same topics. Full chain.
External odom is injected because the real EPIC drone is GPS-denied (LiDAR-inertial odom).

## What works (verified)
- GPU recovered (kernel 6.17.0-35 + matching nvidia 580.159.03; GUI/GL in container OK).
- Board auto-detect + HITL enable (SYS_HITL=1, HIL flag confirmed).
- hils_menu.py: 10-step bring-up all OK in gazebo_truth mode.
- EKF estimation FIXED via external-odom (EKF2_EV_CTRL=11, HGT_REF=3 vision, GPS_CTRL=0,
  BARO_CTRL=0). odom_z = gz_z (anchored), ENU frames consistent (step-9 frame check passes).
- PX4 control chain fully live (verified via nsh console): commander Armed/Offboard/no-failsafe;
  vehicle_local_position_setpoint, attitude_setpoint, rates_setpoint all fresh with full up-thrust;
  actuator_motors = [1.0, 0.0, 0.21, 0.19] produced in real time by control allocation.

## THE BLOCKER (precise)
The drone does not actually fly: armed + OFFBOARD + correct EKF + correct setpoint (z=1.6),
but the physical Gazebo drone never moves.
Root cause: **no actuator OUTPUT driver runs on the board in HITL**, so actuator_motors
(fresh) is never converted to actuator_outputs / HIL_ACTUATOR_CONTROLS -> the gazebo
motor_speed topic gets zero messages -> motors never spin.
nsh evidence:
- `actuator_outputs`: STALE (~714 s old), Instance0=[0...], Instance1=[500...].
- `dshot status` -> not running ; `pwm_out status` -> not running ;
  `pwm_out_sim` -> command not found (SITL-only module, absent on real-board firmware) ;
  `uart_esc` -> command not found.
- `pwm_out start` did not unblock.
So: control allocation OK, but the actuator output stage to the simulator is empty.
This is purely a PX4-HITL-on-real-board output-path issue. Our guidance/adapter/EKF/frames are fine.

## NEXT SESSION - direction (A): fix real-board HITL actuator output
Investigate how PX4 v1.16 routes actuator outputs to the simulator on a REAL board in HITL:
1. `mavlink status` (in nsh) - is the instance on the gazebo serial link sending HIL_ACTUATOR_CONTROLS?
   What mode is that mavlink instance (onboard/normal)? HIL streams active?
2. The board HIL startup (rcS / ROMFS): in HITL, what should publish actuator_outputs on a
   real board? (SITL uses pwm_out_sim; real-board HITL mechanism may differ / may be broken
   in this firmware build.)
3. Output function assignment for the sim (which outputs are the 4 motors). `param show` for
   the relevant output module (this firmware has no PWM_MAIN_FUNCx set).
4. Possibly: build/flash a PX4 firmware that includes pwm_out_sim, or configure the HIL output.
5. Fallback if blocked: PX4 SITL (pure software) gives a working flight loop with the same PX4
   controller -> validates EPIC guidance immediately; pursue real-board HITL output in parallel.

## How to run / inspect
- Launcher (host): `cd ~/poongsan_hils && python3 hils_menu.py gazebo_truth`
  (auto bring-up, then TEST menu: takeoff/land/frame-check/status).
- PX4 nsh console (run from B, which has pymavlink; gazebo must be up forwarding udp:14550):
  `cd ~/poongsan_hils/PX4-Autopilot && printf 'commander status\n' | \
   python3 Tools/mavlink_shell.py 0.0.0.0:14550`
- Containers must both be --network host. epic_onboard entrypoint overridden to `sleep`
  (original image entrypoint was bash -> dies with -d).

## Gotchas / learnings
- pgrep/pkill -f self-matches the launching shell -> use bracket trick `[g]z_truth_bridge` or -x.
- Background procs in docker exec: use `(nohup ... &)` subshell to survive exec exit.
- Container PID1 = sleep (no zombie reaping) -> use `docker run --init` for the final image.
- PX4 int params over MAVLink: reinterpret int32 bytes into the float field (sending 1.0 stores
  1065353216). See scripts/px4_detect.py / configure_ekf_extodom.py.
- EKF z divergence was a long-session drift / local-origin offset; external-odom aiding fixed it.
- In HITL the board's physical orientation is irrelevant (uses simulated HIL sensors).
- Files: hils_menu.py (primary launcher), hils.sh (bash variant), setup_hils.sh (B image recipe),
  run_dev.sh, dev_bashrc.sh, scripts/{px4_detect,run_hitl_gazebo,wait_ready,gz_truth_bridge,
  configure_ekf_extodom,test_hover_cmd}, onboard_adapter/ (px4_setpoint_adapter.py),
  docs/coordinate_alignment.md.
