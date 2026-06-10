# poongsan_hils - EPIC HILS container B (real PX4 board HITL + Gazebo)

Validate that the drone tracks EPIC's output trajectory well before real flight.
- A (epic_onboard, separate): EPIC planner + bridge + MAVROS. Deploys onto the real drone.
- B (this folder): Gazebo 11 (dynamics) + PX4 HITL (real board) + MAVROS + bridge.

## Scope
Trajectory-tracking tuning only. No Livox LiDAR sim / no exploration world
(point cloud is reused from poongsan's MARSIM in live mode).
Base image: osrf/ros:noetic-desktop-full (host 24.04 cannot run Noetic natively).

## Interface (A <-> B boundary)
- EPIC -> HILS: /position_cmd (quadrotor_msgs::PositionCommand) -> /mavros/setpoint_raw/local
- HILS -> EPIC: odom (/lidar_slam/odom), point cloud (cloud_topic, PointCloud2)

## Board / HITL transport
- Board: MicoAir743v2 (PX4), USB serial /dev/ttyACM0 (stable: /dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00)
- The Gazebo mavlink_interface plugin owns the serial and forwards MAVLink to
  udp:14540 (MAVROS) and udp:14550 (QGC) -> no mavlink-router needed.

## Run the HILS (hils_menu.py)  <-- primary entry point

`hils_menu.py` runs on the HOST. It brings up the whole HITL stack (board +
Gazebo + MAVROS + adapter) with one command, verifies every step, and then opens
a TEST menu (takeoff / land / frame-check / status).

```bash
cd ~/poongsan_hils
python3 hils_menu.py gazebo_truth gui      # watch in the Gazebo GUI
python3 hils_menu.py gazebo_truth          # headless (faster, no window)
python3 hils_menu.py onboard_odom gui      # use external (LiDAR) odom instead of Gazebo truth
```
Args (any order): `gazebo_truth` | `onboard_odom` (odom source), optional `gui`.

### Before you run (once per power-on)
- Containers up: `./run_dev.sh` (B) and the epic_onboard container (A) running.
- Board on USB (MicoAir743v2).
- **Cold power-cycle the board once at the start of a session** (unplug+replug
  USB). This guarantees a clean flight-controller state. After that you can run
  the menu as many times as you like - it keeps the sim/sensor stream alive.

### What you'll see
12-step bring-up, each printed OK/FAIL, then:
```
========== HILS READY ==========
===== TEST MENU =====
 1) Takeoff & hover (1.5 m)
 2) Land & disarm
 3) Frame check (Gazebo vs board vs EPIC)
 4) Status
 5) Stop all (down)
 6) Trajectory test - circle (path tracking)   <-- synthetic moving /position_cmd
 7) Trajectory test - figure8
 8) EPIC trajectory replay (recorded epic_traj.bag)  <-- real EPIC path (route A)
 q) Quit            <-- leaves the stack running (sim stays up for the next run)
```
A successful takeoff: the Gazebo drone climbs gz_z 0.1 -> ~1.6 m and holds.
Trajectory test (6/7): publishes a moving /position_cmd (the same topic EPIC
drives) via scripts/test_traj_cmd.py, flies the shape, and prints the tracking
error (commanded vs Gazebo truth) - RMS/max each lap. This is the metric to tune
PX4 gains against. It does its own climb, so you can pick 6 right after READY.
Tighter tracking: set the adapter's _use_vel:=true _use_accel:=true (vel/acc
feed-forward; EPIC's PositionCommand already carries them).

### EPIC trajectory replay (route A) - option 8
Replays a REAL EPIC trajectory (recorded from EPIC's native MARSIM exploration)
onto the HITL drone, so you validate tracking of EPIC's actual output - not just
synthetic shapes. scripts/epic_replay.py reads epic_traj.bag (/planning/pos_cmd),
translates it so the start maps to the HITL drone's current point, and republishes
on /position_cmd. Prints tracking RMS/max. (z stays ~constant; the path spans
~30x24 m, so it flies a real exploration segment.)

How the bag was recorded (one-time, already done - epic_traj.bag is in this dir):
```
# in epic_onboard, EPIC's native demo (MARSIM garage world):
roslaunch epic_planner garage.launch            # MARSIM + planner + rviz (DISPLAY=:1)
rosrun topic_tools throttle messages /quad_0/lidar_slam/odom 50 \
       /quad_0/lidar_slam/odom_throttled        # exploration_node needs *_throttled
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '...'  # trigger
rosbag record -O epic_traj.bag /planning/pos_cmd /quad_0/lidar_slam/odom --duration=40
```
Notes/gotchas found bringing EPIC's demo up here (first time it was run):
- exploration_manager/garage.launch passed unused args (odom_publish_rate,
  sensing_rate) to MARSIM garage.launch -> removed them.
- the exploration_node subscribes /quad_0/lidar_slam/odom_throttled, which nothing
  published -> add the throttle node above ("no odom" spam until then).
- traj_server's /position_cmd is remapped to /planning/pos_cmd in the demo (record
  that topic).

### If it asks you to power-cycle  (flight TERMINATION)
If a bring-up step "Verify board flyable (not terminated)" fails, the board has
latched flight TERMINATION (a HIL-sensor-stream gap; only a cold boot clears it).
The menu will print **"ACTION NEEDED: cold power-cycle"** and wait - just UNPLUG
the board USB, wait ~3 s, and PLUG it back in; the menu detects it and continues
automatically. (Why this happens: see docs/CHECKPOINT_2026-06-08.md.)

### Between runs: use `q`, not `Stop all`
- **`q` (Quit)** leaves gazebo + the board running -> the HIL_SENSOR stream stays
  continuous -> the next `python3 hils_menu.py ...` REUSES it and flies with no
  power-cycle. Use this between iterations.
- **`5` (Stop all)** kills gazebo. With the sim gone, the board's EKF drifts (no
  sensors) and can't be recovered live -> you must **cold power-cycle the board**
  before the next run. Only use Stop all when you're truly done for the session.
- Same applies if gazebo ever crashes: cold power-cycle, then run.

### Design notes that keep it reliable (sensor-stream continuity)
- Re-runs REUSE the running gazebo + roscore and DO NOT reboot the board, so the
  gazebo->board HIL_SENSOR stream is never interrupted (an interruption ->
  gyro timeout -> flight TERMINATION). When gazebo owns the board serial, board
  ops go over its udp:14550 forward.
- Backstops: EKF EV-reset on divergence (scripts/ekf_ev_reset.py), a 3D
  convergence gate before arming, RC/GCS failsafes disabled for HITL, and the
  power-cycle prompt above.

## Workflow (the key discipline)
1. ./run_dev.sh - start the long-lived dev container (workspace bind-mounted, GPU/host-net/X11/USB).
2. Install/build inside the container; transcribe every command into setup_hils.sh immediately.
3. Occasionally run setup_hils.sh in a fresh container to catch missing deps.
4. When done, transcribe setup_hils.sh into a Dockerfile and ship the image (use `docker run --init`).

## Files
- run_dev.sh - dev container launcher / re-attach
- setup_hils.sh - accumulated install steps (= future Dockerfile)
- dev_bashrc.sh - dev shell (rec/note/recall discipline tools)
- scripts/px4_detect.py - auto-detect PX4 board on USB + check/enable HITL (SYS_HITL)
    - `python3 scripts/px4_detect.py` : detect + status; prompts y/N to enable HITL if needed
    - `--enable-hitl` : set SYS_HITL=1 without prompt ; `--quiet` : print device path only
- scripts/run_hitl_gazebo.sh - launch the Gazebo HITL world that connects to the board
    - `HEADLESS=1 ./scripts/run_hitl_gazebo.sh` for gzserver only (no GUI)
- hils_menu.py - **main launcher** (host): one-shot bring-up + TEST menu (see "Run the HILS" above)
- scripts/check_hil_output.py - verify the HITL actuator-output module (pwm_out_sim) is in the firmware
    - `--conn <dev|udp>` ; `--fix` patches board config + rebuilds/flashes ; prints PX4 doc links
- scripts/configure_ekf_extodom.py - set EKF2 for external-vision odom + HITL failsafe params
    - `<dev|udp> [--no-reboot]` ; --no-reboot sets params live (keeps sensor stream continuous)
- scripts/gz_truth_bridge.py - /gazebo/model_states -> /mavros/vision_pose/pose (+ /lidar_slam/odom)
- scripts/test_hover_cmd.py - hover/takeoff/land setpoint publisher (_climb / _abs_z)
- scripts/test_traj_cmd.py - synthetic moving /position_cmd (circle/figure8/square/line) + tracking RMS
- scripts/epic_replay.py - replay recorded EPIC trajectory (epic_traj.bag) onto /position_cmd + tracking RMS
- epic_traj.bag - recorded EPIC /planning/pos_cmd trajectory (from native MARSIM garage demo)
- scripts/wait_ready.py - wait for a board heartbeat on serial or udp
- scripts/ekf_ev_reset.py - toggle EKF2_EV_CTRL (resets z only; NOT a reliable x/y fix - kept for reference)
- docs/coordinate_alignment.md - ENU world <-> Gazebo/PX4 origin/yaw alignment (required in live mode)
- docs/CHECKPOINT_2026-06-08.md - full root-cause + fix history (pwm_out_sim build, airframe 1001,
  flight-termination/HIL-sensor-gap, sensor-continuity hardening). Read this first next session.

## Firmware note (one-time, already done)
The board runs a CUSTOM build of PX4 v1.16 (`micoair_h743-v2`) that ADDS the
`pwm_out_sim` module (CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y) - the stock MicoAir
build omits it, so HITL produced no motor output. Airframe = SYS_AUTOSTART 1001
(HIL Quadcopter X). Build toolchain (arm-none-eabi-gcc 10.3-2021.10) + deps are in
setup_hils.sh [7]; the built .px4 is build/micoair_h743-v2_default/. To reflash:
`cd PX4-Autopilot && make micoair_h743-v2 upload`, then UNPLUG+REPLUG the USB while
it prints "Attempting reboot...".
