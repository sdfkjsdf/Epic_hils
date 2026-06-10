# EPIC HILS - Session Checkpoint (2026-06-08)  [BLOCKER SOLVED - DRONE FLIES]

Continues CHECKPOINT_2026-06-07.md. Goal unchanged: validate that the drone
tracks EPIC's output trajectory in HITL (real MicoAir743v2 board + Gazebo).
STATUS: the 06-07 "arms but never moves" blocker is fully resolved - the HITL
drone now takes off and hovers (see SOLVED section below).

## THE BLOCKER FROM 06-07 IS NOW SOLVED (root cause + fix in hand)
Symptom: armed + OFFBOARD + correct EKF + correct setpoint, but the sim drone
never moved.
Root cause (confirmed): PX4 sends HIL_ACTUATOR_CONTROLS from the
`actuator_outputs_sim` uORB topic
(src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp:60), which is published
ONLY by the `pwm_out_sim` module (PWMSim.hpp:87). That module is a BUILD-TIME
option and was NOT compiled into the micoair_h743-v2 firmware:
  - nsh `pwm_out_sim status` -> "command not found" (verified live)
  - `param show HIL_ACT*` -> empty (module's params absent)
  - boards/micoair/h743-v2/default.px4board had no CONFIG key
rcS:451-457 tries `pwm_out_sim start -m hil` in HITL but only plays an error tune
when it fails. -> no actuator_outputs_sim -> empty stream -> motors never spin.
This is exactly the case in the official PX4 HITL guide
(docs.px4.io/main/en/simulation/hitl).

## WHAT WAS DONE THIS SESSION
1. Board config fixed (official, documented procedure - NOT a hack):
   boards/micoair/h743-v2/default.px4board now has:
       CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y   (inserted after MODULES_SENSORS)
2. Build toolchain installed in container B (recorded in setup_hils.sh [7]):
   - ARM gcc: /opt/gcc-arm-none-eabi  (gcc-arm-none-eabi-10.3-2021.10, the exact
     version PX4/NuttX CI pins). Added to dev_bashrc.sh PATH.
   - apt genromfs ; pip pyros-genmsg (provides the `genmsg` module the build uses)
   - NuttX submodule had NO git tags -> px_update_git_header.py threw IndexError;
     fixed once with: git -C platforms/nuttx/NuttX/nuttx fetch --tags
3. Firmware BUILT successfully (exit 0):
   build/micoair_h743-v2_default/micoair_h743-v2_default.px4  (1.69 MB)
   Verified pwm_out_sim symbols ARE in the .elf (pwm_out_sim_main, PWMSim.cpp).
   NOTE: this .px4 is on the host bind-mount -> persists across containers.
4. Self-service guard added so future users hit a clear message, not a silent
   no-fly: scripts/check_hil_output.py
   - auto-detects board, runs `pwm_out_sim status` over MAVLink nsh,
   - if missing, prints the official remedy + doc/GitHub URLs,
   - `--fix` patches the board config and (if toolchain present) rebuilds+flashes.
   - wired into hils_menu.py as preflight step
     "Check HITL output module (pwm_out_sim)" (runs before the sim grabs serial).
5. Param backup before flashing: param_backup_preflash.json (1031 params).
   Key values: SYS_AUTOSTART=4001 (Generic Quad, works in HITL - did NOT force
   1001), SYS_HITL=1, EKF2_EV_CTRL=11, EKF2_HGT_REF=3, MAV_TYPE=2.
6. Docker image saved: poongsan_hils_dev:hitl-2026-06-08 (= :latest), 8.57 GB
   (base osrf/ros:noetic-desktop-full + everything in setup_hils.sh incl.
   toolchain). epic_onboard unchanged this session (epic_onboard:latest current).

## ★★ SOLVED - THE DRONE FLIES IN HITL (2026-06-08) ★★
Flashed + verified end-to-end. Takeoff test (gazebo_truth, 1.5 m setpoint):
  t=2s gz_z=0.10 -> t=4s 1.24 -> t=6s 1.55 -> t=8s 1.60 -> hover ~1.61 m stable,
  ekf_z tracks gz_z. The Gazebo drone physically climbs and holds hover.

The fix was TWO parts (both required):
  (1) FIRMWARE (could NOT be a runtime param): pwm_out_sim module was not
      compiled into micoair_h743-v2 -> added CONFIG_MODULES_SIMULATION_PWM_OUT_SIM
      + rebuilt + flashed. After flash: nsh `pwm_out_sim status` works, but motors
      still didn't spin because...
  (2) OUTPUT MAPPING (a runtime param): HIL_ACT_FUNC1..4 were 0 (Disabled) ->
      pwm_out_sim had no motor->channel map -> "cycle: 0 events", empty
      HIL_ACTUATOR_CONTROLS. Control allocation was fine (airframe present,
      actuator_motors computed); only the OUTPUT stage mapping was missing.
      Fixed the clean/official way by switching the airframe to the purpose-built
      HIL airframe: `param set SYS_AUTOSTART 1001` (HIL Quadcopter X) + reboot.
      The 1001 script (ROMFS .../airframes/1001_rc_quad_x.hil) auto-sets
      HIL_ACT_FUNC1..4=101..104, SYS_HITL=1, CA_ROTOR_* geometry, and
      CBRK_SUPPLY_CHK=894281 (arm without battery). Generic 4001 does NOT set
      HIL_ACT_FUNC (that was the gap). Verified all applied after reboot.
  NB: airframe/output-function mapping is the RUNTIME-param half the user
  intuited; it just needed the module (firmware) present first so the params exist.

FLASH method that worked (container B is privileged + /dev:/dev so USB
re-enumeration is visible): `make micoair_h743-v2 upload`, then physically
UNPLUG + REPLUG the board USB while it prints "Attempting reboot..." -> uploader
catches the bootloader window -> Program 100% / Verify 100% / Reboot. (The
57600 auto-reboot and MAVLink reboot-to-bootloader param1=3 did NOT enter BL on
this board; the replug is the reliable trigger.)

### Stability hardening + Gazebo GUI (2026-06-08, later)
After the first flight, repeated takeoffs were flaky / failed. Three distinct
root causes, all now fixed (drone takes off reliably in the GUI, gz_z 0.1->1.6):
  (A) EKF settling transient: after a board reboot the EKF2 external-vision frame
      (esp. YAW) takes ~20-30 s to align; during it the estimate swings far
      (saw EKF x,y = -7, +39 while gazebo truth = 1, 1). Arming in that window =
      bad. The old anchor gate checked only Z (passed too early). FIX: s_anchor in
      hils_menu.py now requires |dx|,|dy|,|dz| < 0.5 m AND stable for 3 consecutive
      checks (full 3D convergence) before READY.
  (B) Large transient was amplified by the world spawning the vehicle at
      (1.01, 0.98, yaw=1.14 rad). FIX: world spawn changed to origin/yaw 0 in
      hitl_iris.world: `<pose>0 0 0.83 0 0 0</pose>` (matches the coordinate-
      alignment strategy; transient now tiny - anchor dx=0.01, dy=0.00).
  (C) Flight TERMINATION failsafe (the real "arms but won't climb" cause):
      failsafe_flags showed manual_control_signal_lost + global_position_invalid;
      NAV_RCL_ACT was 2 (RC-loss action = Return). HITL has no RC -> RC loss ->
      Return -> but no GPS -> RTL impossible -> escalates to flight TERMINATION
      (latched, motors killed). FIX: NAV_RCL_ACT=0 (disable RC-loss failsafe) +
      COM_RCL_EXCEPT=4 (except Offboard); added to configure_ekf_extodom.py so it
      is reapplied every bring-up (airframe changes reset these).

### Gazebo GUI
hils_menu.py now takes a `gui` arg: `python3 hils_menu.py gazebo_truth gui`.
s_sim launches `gazebo` (full GUI) instead of headless gzserver, passing DISPLAY.
Container B is privileged with /dev:/dev + X socket mounted + working NVIDIA GL
(GTX 1660 SUPER), DISPLAY=:1. NOTE: if the drone is invisible but the world
renders, it is usually stale/accumulated gazebo instances from earlier runs
(PID1=sleep doesn't reap zombies) - do a full pkill of gazebo/gzserver/gzclient
and relaunch clean.

### STABILIZATION (resolved) - the real flakiness was flight TERMINATION from HIL-sensor gaps
The "arms but won't climb" flakiness was finally root-caused via the board's
dmesg: `angular velocity no longer valid (timeout)` + `No valid data from
Gyro/Accel/Baro/Compass` + `MAG/BARO TIMEOUT` -> PX4 latches flight TERMINATION.
i.e. the gazebo->board HIL_SENSOR stream briefly stalls during bring-up (around
the step-5 board reboot, before gazebo is feeding) -> gyro timeout -> TERMINATION
LATCHES (motors killed) and survives until reboot. The estimator keeps running
(odom valid, anchor passes), so it arms but never lifts. Confirmed it was NOT RC:
with NAV_RCL_ACT=0 + COM_RC_IN_MODE=4 + COM_RCL_EXCEPT=4 it still terminated, and
failsafe_flags showed angular_velocity recovered (False) while nav_state stayed
Termination -> a latched state, not a live condition.
KEY FINDING: after ~15 software reboots/termination cycles in a session it became
DETERMINISTIC (terminated every bring-up; retries couldn't converge). A software
reboot did NOT clear it; a PHYSICAL board power-cycle (USB unplug/replug = cold
boot) DID -> then it flew reliably again. So: persistent termination => cold
power-cycle the board.

What was added to make bring-up robust (hils_menu.py):
  - s_flyable() step "Verify board flyable (not terminated)": reads nsh
    `commander status`; if "Termination" -> step fails.
  - main() retries the whole clean bring-up up to 4x (step-5 reboot clears a
    soft-latched termination between attempts).
  - s_anchor(): EKF EV-reset (scripts/ekf_ev_reset.py toggles EKF2_EV_CTRL 0->11)
    at 20 s/40 s if x,y not converged - un-sticks a diverged EKF without a reboot;
    requires 5 consecutive in-tol checks before READY.
  - configure_ekf_extodom.py also sets NAV_RCL_ACT=0, COM_RCL_EXCEPT=4,
    COM_RC_IN_MODE=4, NAV_DLL_ACT=0 every bring-up (HITL has no RC/GCS).
  - main() failure message tells the user to cold power-cycle the board if it kept
    failing at the flyable step.
Net: a clean session (or right after a cold power-cycle) flies reliably.

### SENSOR-CONTINUITY HARDENING (implemented + validated) - eliminates the gap at source
Root of the termination = the HIL_SENSOR stream (gazebo->board serial) being
interrupted during bring-up. The bring-up was doing two things that gap it every
run: (1) rebooting the board (step 5), (2) killing+restarting gazebo (cleanup ->
step 7). Reworked the bring-up to NEVER interrupt the stream:
  - s_cleanup: kills ONLY the ROS control nodes (adapter, hover, bridge, mavros).
    Does NOT kill gazebo (gzserver feeds HIL_SENSOR) or roscore (gazebo's
    ros_api_plugin is bound to it).
  - s_config_ekf: configure_ekf_extodom.py --no-reboot - sets the EV/failsafe
    params LIVE, no board reboot. (The board's params are persisted, so a cold
    boot already has them; live-set is a no-op for the init params -> safe.)
  - s_roscore / s_sim: reuse a running roscore / gazebo; start only if absent.
  - When gazebo is up it owns the board serial, so s_detect/s_check_output/
    s_config_ekf talk to the board over gazebo's udp:14550 (gz_up() switch);
    s_detect/s_hitl skip the serial probe (board already validated this session).
Result (validated): a RE-RUN now reuses gazebo (HIL_SENSOR never gaps) -> step 11
"Verify board flyable" passes (no termination) -> takeoff climbs 0.1->1.6 m, with
NO board power-cycle needed between runs. Two consecutive re-runs both flew.
Workflow now: cold power-cycle the board ONCE at session start (clean state),
then run the menu as many times as you like - it keeps the sensor stream alive.
(scripts/ekf_ev_reset.py + the 4x retry + flyable-check remain as backstops.)

### (superseded) earlier note - intermittent EKF horizontal-position divergence
The takeoff is intermittent: sometimes the EKF X,Y locks to gazebo truth and it
flies (gui4), sometimes the EKF X,Y diverges and stays stuck (gui5: gazebo truth
(0,0) but EKF (-38,-20); z & yaw fine). When stuck, the 3D anchor correctly
fails (good - it no longer arms into a bad estimate), so it just won't reach
READY. Hypothesis: the board reboots at step 5, but vision (gz_truth_bridge)
only starts at step 8 (~15-20 s later, after gazebo is up); the EKF drifts
horizontally during that no-vision window, then rejects the late vision
(innovation gate) and never resets -> stuck.
Gotchas confirmed this session:
  - `ekf2 stop` then `ekf2 start` over nsh FAILS ("start failed (-1)") and
    leaves ekf2 stopped -> needs a board reboot to recover. Don't use it.
  - Rebooting the board mid-session breaks the gazebo HIL link AND MAVROS
    (connected:False) - gazebo's mavlink_interface doesn't reopen the serial.
    So the board reboot MUST stay at step 5 (before gazebo grabs the serial);
    surgical mid-session reboots cascade-break the stack. Always recover with a
    full clean menu re-run (step-1 cleanup now also kills the `gazebo` wrapper).
Candidate fixes to try next:
  1. Ensure vision is flowing before the EKF initialises: e.g. reduce the
     reboot->vision gap, or briefly publish vision before/right after the step-5
     reboot.
  2. Make EV horizontal fusion reset-friendly so a far measurement snaps the
     estimate instead of being gated out: raise EKF2_EVP_NOISE / the EV position
     gate, or force an EKF position reset to EV once vision is confirmed.
  3. Anchor-retry: if s_anchor fails, re-run bring-up automatically (clean).

### Remaining / nice-to-have
- Params persist on the board (SYS_AUTOSTART=1001 saved), so this is one-time.
  If a fresh board / reflash happens, set SYS_AUTOSTART=1001 + reboot again.
  Consider adding an airframe check to hils_menu preflight (like the pwm_out_sim
  guard) so a wrong airframe is caught automatically.
- Now do the actual GOAL: feed EPIC /position_cmd and validate path tracking
  (not just a hover setpoint). onboard_adapter already bridges it.

## How to run / inspect (unchanged from 06-07)
- Launcher (host): `cd ~/poongsan_hils && python3 hils_menu.py gazebo_truth`
- nsh console (container B, gazebo up forwarding udp:14550):
  `cd ~/poongsan_hils/PX4-Autopilot && printf 'cmd\n' | \
   python3 Tools/mavlink_shell.py 0.0.0.0:14550`
- Build (container B): `export PATH=/opt/gcc-arm-none-eabi/bin:$PATH && \
  cd ~/poongsan_hils/PX4-Autopilot && make micoair_h743-v2 [upload]`
