#!/usr/bin/env python3
"""
hils_menu.py - EPIC HILS one-shot launcher with live progress + test menu.

Run on the HOST. Brings up the whole HITL stack with external-odometry aiding
(GPS/baro off), printing each step. Only when everything passes does it open the
TEST menu (takeoff/hover, land, frame-check, status).

Odometry source MODES (choose at start):
  gazebo_truth  - use Gazebo ground-truth pose directly (no estimation error).
                  Isolates EPIC guidance + PX4 tracking.
  onboard_odom  - use the odometry the onboard receives (external/LiDAR odom).
                  Full chain incl. estimation error (source plugged in externally).

Bring-up (board rebooted + EKF set for external vision):
  1 cleanup -> 2 detect -> 3 HITL -> 4 EKF=extodom(+reboot) -> 5 roscore ->
  6 sim(Gazebo+ros) -> 7 odom source -> 8 mavros -> 9 EKF anchor/frame-check ->
  10 adapter

Containers: A = epic_onboard (ROS), B = poongsan_hils_dev (Gazebo).
"""
import shlex
import subprocess
import sys
import time

A = "epic_onboard"
B = "poongsan_hils_dev"
WS = "/root/poongsan_hils"
UDP = "14540"
MODEL = "iris"
ROS = "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash 2>/dev/null"

G, R, Y, C, N = "\033[1;32m", "\033[1;31m", "\033[1;33m", "\033[1;36m", "\033[0m"

MODE = "gazebo_truth"   # or "onboard_odom" (set from argv)
BOARD_DEV = "/dev/ttyACM0"
HITL_ON = False
GUI = False             # True -> launch the Gazebo GUI (add "gui" arg); needs DISPLAY
GZ_OK = False           # set by s_cleanup: a HEALTHY gazebo (alive AND forwarding) is reusable
FF = False              # adapter velocity/accel feed-forward (off=position-only baseline)
EV_VEL = True           # True -> EKF2 fuses external-odom VELOCITY (EKF2_EV_CTRL=15);
                        # "pose_only" arg -> position+yaw only (=11). Bridge always sends
                        # full nav_msgs/Odometry; this just selects what EKF2 fuses.
EPIC = False            # True -> run the live EPIC closed loop (add "epic" arg)
# EPIC live closed-loop wiring (gazebo_truth): the shared drone-state topic.
EPIC_ODOM = "/quad_0/lidar_slam/odom"   # bridge publishes here; EPIC+pcl_render+FC share it
EPIC_CMD = "/planning/pos_cmd"          # EPIC traj_server output -> adapter input
# NOTE: no coordinate offset. One Gazebo-truth odom (offset 0) feeds both EPIC and
# the FC, like LIO-SAM. Gazebo origin (0,0) is garage free-space; the drone just has
# to be airborne (~2 m) before EPIC plans (z=0.1 ground = floor collision).


def sh(cmd, timeout=60):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except subprocess.TimeoutExpired:
        return 124, "", "timeout"


def ea(cmd, timeout=60):
    return sh("docker exec %s bash -lc %s" % (A, shlex.quote(ROS + "; " + cmd)), timeout)


def eb(cmd, timeout=60):
    return sh("docker exec %s bash -lc %s" % (B, shlex.quote(cmd)), timeout)


def ea_d(cmd):
    """Detached launch in A (survives - roslaunch needs a real detach, not a
    nohup-subshell which dies when the exec returns). DISPLAY set for GL nodes."""
    return sh("docker exec -d %s bash -lc %s" % (A, shlex.quote(ROS + "; export DISPLAY=:1; " + cmd)))


def ensure_up():
    sh("docker start %s >/dev/null 2>&1" % A)
    sh("docker start %s >/dev/null 2>&1" % B)


def gz_up():
    """True if a LIVE gzserver is running in B (HIL_SENSOR stream live, board
    serial owned by gazebo). Must exclude <defunct> zombies: container PID1=sleep
    doesn't reap, so dead gzservers linger as zombies and `pgrep -x gzserver`
    would match them (a stale 'gazebo is up' -> board ops misrouted to udp)."""
    out = eb("ps -eo stat=,comm= | awk '$2==\"gzserver\" && $1 !~ /Z/' | wc -l", 8)[1].strip()
    try:
        return int(out) > 0
    except Exception:
        return False


def board_present():
    """True if the MicoAir board is currently enumerated on USB (host)."""
    return sh("ls /dev/serial/by-id/*MicoAir* >/dev/null 2>&1 && echo yes", 6)[1].strip() == "yes"


def wait_powercycle():
    """A latched flight TERMINATION only clears on a COLD boot (a software reboot
    of a degraded board does not). We don't auto power-cycle (no reliable per-port
    USB power control on this host), so prompt the operator to physically replug
    the board, then detect the unplug+replug and continue automatically.
    Returns True once the board is back, False on timeout."""
    print("")
    print("%s================================================================%s" % (Y, N))
    print("%s  ACTION NEEDED: cold power-cycle the flight controller%s" % (Y, N))
    print("%s================================================================%s" % (Y, N))
    print("  The board is in a state that only a COLD boot clears: either a latched")
    print("  flight TERMINATION (HIL-sensor gap) or a diverged EKF (drifted while the")
    print("  sim was down). A software reboot / EV-reset does NOT fix it.")
    print("  >>> Please UNPLUG the board USB, wait ~3 s, then PLUG it back in. <<<")
    print("  (Other USB devices are on a different bus; only the board is affected.)")
    print("")
    # the running gazebo lost the serial on unplug -> kill it so a fresh one binds
    # the re-enumerated board cleanly on the next bring-up.
    eb("pkill -x gazebo 2>/dev/null; pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null", 15)
    # Manual confirmation (no auto-restart): the operator replugs, then presses
    # Enter. We verify the board is back before continuing.
    while True:
        try:
            ans = input("  After you REPLUG the board, press Enter to continue (q = abort): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return False
        if ans == "q":
            return False
        if board_present():
            time.sleep(2)  # let it finish booting + enumerate
            print("  board detected - continuing bring-up.")
            return True
        print("  %s(board not on USB yet - replug, then press Enter again; or q to abort)%s" % (Y, N))


def rfield(field):
    """rostopic echo one numeric leaf field -> float or None."""
    _, out, _ = ea("timeout 3 rostopic echo -n1 %s 2>/dev/null | head -1" % field, 6)
    try:
        return float(out.strip().splitlines()[0])
    except Exception:
        return None


def gz_pose():
    """Gazebo true pose of the model -> [x,y,z] or None."""
    _, out, _ = eb("source /usr/share/gazebo/setup.sh 2>/dev/null; timeout 4 gz model -m %s -p 2>/dev/null | head -1" % MODEL, 8)
    p = out.split()
    try:
        return [float(p[0]), float(p[1]), float(p[2])]
    except Exception:
        return None


# ---------- bring-up steps: return (ok, msg) ----------
def s_cleanup():
    # Kill ONLY the lightweight ROS control nodes. Do NOT kill gazebo (gzserver
    # feeds the board's HIL_SENSOR over serial) or roscore (gazebo's ros_api_plugin
    # is bound to it). Killing either gaps the HIL_SENSOR stream -> gyro timeout ->
    # flight TERMINATION (latched). Gazebo/roscore are (re)used if alive, started
    # only if absent. This is the key to sensor-stream continuity across re-runs.
    global GZ_OK
    ea("pkill -f [t]est_hover_cmd 2>/dev/null; pkill -f [p]x4_setpoint_adapter 2>/dev/null; "
       "pkill -f [g]z_truth_bridge 2>/dev/null; pkill -x mavros_node 2>/dev/null; "
       "pkill -f [e]xploration_node 2>/dev/null; pkill -f [o]pengl_render 2>/dev/null; "
       "pkill -f [m]ap_pub 2>/dev/null; pkill -f [t]raj_server 2>/dev/null; "
       "pkill -f [w]aypoint_generator 2>/dev/null; pkill -f 'throttle messages' 2>/dev/null", 25)
    time.sleep(2)
    # Reuse an existing gazebo ONLY if it is HEALTHY = alive AND actually forwarding
    # board telemetry (udp:14550). A live-but-stale gazebo (process up but serial
    # dead, e.g. after a board power-cycle or an aborted run) does NOT forward ->
    # kill it so we start fresh (otherwise s_detect fails 'board not on udp').
    GZ_OK = False
    alive = eb("ps -eo stat=,comm= | awk '$2==\"gzserver\" && $1 !~ /Z/' | wc -l", 8)[1].strip()
    try:
        if int(alive) > 0 and eb("python3 %s/scripts/wait_ready.py udp 14550 3" % WS, 8)[0] == 0:
            GZ_OK = True
    except Exception:
        GZ_OK = False
    if not GZ_OK:
        eb("pkill -x gazebo 2>/dev/null; pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null", 15)
        time.sleep(2)
    return True, ("healthy gazebo found -> reuse (sensor continuity)" if GZ_OK
                  else "fresh start (no healthy gazebo; stale one cleared if any)")


def s_detect():
    global HITL_ON, BOARD_DEV
    if GZ_OK:
        # gazebo already owns the board serial (HIL stream live); board was
        # validated earlier this session. Confirm it's alive via gazebo's udp.
        live = eb("python3 %s/scripts/wait_ready.py udp 14550 8" % WS, 12)[0] == 0
        HITL_ON = True
        return live, "board live via gazebo udp:14550 (serial owned by gazebo)" if live else "board not on udp:14550"
    rc, out, _ = eb("python3 %s/scripts/px4_detect.py" % WS, 40)
    port = ""
    for line in out.splitlines():
        if "device" in line and ":" in line:
            port = line.split(":", 1)[1].strip()
    if rc == 2:
        return False, "no board (plug in USB)"
    if port:
        BOARD_DEV = port
    HITL_ON = (rc == 0)
    return True, "port=%s HITL=%s" % (BOARD_DEV, "on" if HITL_ON else "OFF")


def s_hitl():
    if HITL_ON:
        return True, "already on"
    rc, _, _ = eb("python3 %s/scripts/px4_detect.py --enable-hitl" % WS, 40)
    return rc == 0, "SYS_HITL set" if rc == 0 else "enable failed"


def s_check_output():
    """Preflight: the board firmware must contain pwm_out_sim, else HITL produces
    no actuator output (drone arms but never moves). Checked over the board serial
    while it is still free (before the simulator grabs it)."""
    conn = "udpin:0.0.0.0:14550" if GZ_OK else BOARD_DEV  # serial is gazebo's when it runs
    rc, out, _ = eb("python3 %s/scripts/check_hil_output.py --conn %s" % (WS, conn), 45)
    if rc == 0:
        return True, "pwm_out_sim present (HIL output path OK)"
    print("\n" + out.rstrip())  # show the official remedy + doc links
    return False, "HIL output module missing -> fix: python3 scripts/check_hil_output.py --fix"


def s_config_ekf():
    # --no-reboot: the board's EKF params are persisted (cold boot loads them), so
    # we set them live and SKIP the reboot. The reboot would drop the gazebo
    # HIL_SENSOR feed -> gyro timeout -> flight TERMINATION. Use udp when gazebo
    # owns the serial.
    conn = "udpin:0.0.0.0:14550" if GZ_OK else BOARD_DEV
    ev = "" if EV_VEL else " --pose-only"   # pose_only arg -> EKF2 fuses pos+yaw only
    rc, _, _ = eb("python3 %s/scripts/configure_ekf_extodom.py %s --no-reboot%s" % (WS, conn, ev), 40)
    time.sleep(2)
    return rc == 0, "EKF=external-vision set live (no reboot -> sensor continuity)"


def s_roscore():
    # Check RESPONSIVENESS, not pgrep: container PID1=sleep doesn't reap, so dead
    # roscores linger as <defunct> zombies that pgrep matches -> a stale 'up' while
    # the master is actually dead (gazebo then can't connect -> 'No ROS master').
    if ea("timeout 5 rostopic list >/dev/null 2>&1 && echo ok", 8)[1].strip() == "ok":
        return True, "roscore up (responsive, reused)"
    ea("(nohup roscore >%s/.A_roscore.log 2>&1 &)" % WS, 15)
    for _ in range(12):
        if ea("timeout 4 rostopic list >/dev/null 2>&1 && echo ok", 6)[1].strip() == "ok":
            return True, "roscore started (responsive)"
        time.sleep(1)
    return False, "roscore not responsive (see .A_roscore.log)"


def s_sim():
    global GZ_OK
    # Reuse a running gazebo (preserves the continuous HIL_SENSOR stream -> no
    # gyro timeout -> no flight TERMINATION). Only start it if absent.
    if GZ_OK:
        # ...but a gazebo connected to a DEAD roscore passes udp yet has no
        # /gazebo/model_states on this roscore -> the truth bridge gets no pose.
        # roscore is up by now (prev step), so verify; if stale, restart gazebo.
        ms = ea("timeout 5 rostopic hz /gazebo/model_states 2>&1 | grep -q 'average rate' && echo y", 8)[1].strip() == "y"
        if ms:
            return True, "gazebo reused (HIL_SENSOR + model_states OK)"
        eb("pkill -x gazebo 2>/dev/null; pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null", 15)
        time.sleep(2)
        GZ_OK = False  # falling through to a fresh start; board ops already done via udp
    headless = "0" if GUI else "1"
    eb("(HEADLESS=%s DISPLAY=${DISPLAY:-:1} ROS_MASTER_URI=http://localhost:11311 "
       "nohup %s/scripts/run_hitl_gazebo.sh >%s/.gz_hitl.log 2>&1 &)" % (headless, WS, WS), 15)
    if eb("python3 %s/scripts/wait_ready.py udp %s %d" % (WS, UDP, 90 if GUI else 50), 100)[0] == 0:
        return True, "udp:%s forwarding%s (started)" % (UDP, " GUI" if GUI else "")
    return False, "no udp (see .gz_hitl.log)"


def s_odom_source():
    if MODE == "gazebo_truth":
        # Gazebo truth -> ONE odom (offset 0) feeding both EPIC (shared topic) and the
        # FC vision, mirroring LIO-SAM (single SLAM odom = EPIC = FC). SLAM stand-in.
        otopic = EPIC_ODOM if EPIC else "/lidar_slam/odom"
        ea("(nohup python3 %s/scripts/gz_truth_bridge.py _model_name:=%s _odom_topic:=%s "
           ">%s/.A_gtbridge.log 2>&1 &)" % (WS, MODEL, otopic, WS), 15)
        time.sleep(3)
        ok1 = ea("timeout 5 rostopic hz /gazebo/model_states 2>&1 | grep -q 'average rate' && echo y", 8)[1].strip() == "y"
        ok2 = ea("timeout 5 rostopic hz /mavros/odometry/out 2>&1 | grep -q 'average rate' && echo y", 8)[1].strip() == "y"
        if ok1 and ok2:
            return True, "Gazebo truth -> /mavros/odometry/out (PX4 EKF2) + %s (EPIC)" % otopic
        return False, "bridge not publishing (model_states=%s fcu_odom=%s)" % (ok1, ok2)
    else:  # onboard_odom (HOOK): external LiDAR-SLAM publishes the shared odom topic
        return True, "HOOK: expecting external odom on %s (+ relay to /mavros/odometry/out) - plug in LiDAR-SLAM" % EPIC_ODOM


def s_mavros():
    ea('pgrep -x mavros_node >/dev/null || (nohup roslaunch mavros px4.launch fcu_url:="udp://:%s@" >%s/.A_mavros.log 2>&1 &)' % (UDP, WS), 15)
    for _ in range(30):
        if "True" in ea("timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep connected", 8)[1]:
            return True, "MAVROS connected"
        time.sleep(1)
    return False, "MAVROS not connected"


def s_anchor():
    """Wait until EKF odom matches Gazebo truth in FULL 3D (x,y,z), not just z.
    After a board reboot the EKF settles with a large x,y transient (seen up to
    tens of metres) before the external-vision fusion converges. Arming during
    that transient triggers a position failsafe that can escalate to flight
    TERMINATION (latched, needs reboot). So we require x,y,z all converged AND
    stable for a few consecutive checks before declaring the frame anchored.
    GUI mode is slower, so allow generous time."""
    TOL = 0.5          # metres, per-axis
    need_stable = 5    # consecutive in-tolerance checks (solid lock before arming)
    ok_count = 0
    for i in range(70):
        ox = rfield("/mavros/local_position/odom/pose/pose/position/x")
        oy = rfield("/mavros/local_position/odom/pose/pose/position/y")
        oz = rfield("/mavros/local_position/odom/pose/pose/position/z")
        gp = gz_pose()
        if None not in (ox, oy, oz) and gp is not None:
            dx, dy, dz = abs(ox - gp[0]), abs(oy - gp[1]), abs(oz - gp[2])
            if dx < TOL and dy < TOL and dz < TOL:
                ok_count += 1
                if ok_count >= need_stable:
                    return True, "odom~gz 3D dx=%.2f dy=%.2f dz=%.2f (anchored, stable)" % (dx, dy, dz)
            else:
                ok_count = 0
        time.sleep(1)
    # Diverged and won't lock. A toggle of EKF2_EV_CTRL does NOT reset horizontal
    # position (verified - it fixes z only), so there is no reliable live recovery:
    # the EKF horizontal estimate only re-initialises cleanly on a board reboot /
    # cold boot. main() routes this to the cold-power-cycle prompt.
    return False, "EKF did not lock to Gazebo truth in 3D (diverged; needs cold boot)"


def s_flyable():
    """Verify the board is NOT latched in flight TERMINATION.
    A transient HIL-sensor (gyro/accel/baro) dropout during bring-up makes PX4
    declare 'angular velocity invalid' and latch flight TERMINATION, which only
    clears on reboot. The estimator keeps working (anchor still passes), so the
    drone arms but the motors stay killed -> no climb. Detect it here; returning
    False makes main() retry the whole clean bring-up (step-5 reboot clears it)."""
    rc, out, _ = eb("cd %s/PX4-Autopilot && printf 'commander status\\n' | "
                    "timeout 18 python3 Tools/mavlink_shell.py 0.0.0.0:14550 2>/dev/null" % WS, 25)
    low = out.lower()
    if "termination" in low:
        return False, "board in flight TERMINATION (transient HIL-sensor gap latched) -> retrying clean"
    if "navigation mode" in low:
        return True, "board flyable (no termination)"
    return True, "flyable (commander status unread; not blocking)"


def s_epic():
    """Launch EPIC live (brain + perception): planner + pcl_render + map + throttle,
    NO MARSIM dynamics/controller (HITL Gazebo+PX4 replace those). Consumes the
    shared odom (EPIC_ODOM) the bridge publishes; outputs EPIC_CMD. Detached
    (roslaunch + GL need a real detach + DISPLAY)."""
    ea("pkill -f [e]xploration_node 2>/dev/null; pkill -f [o]pengl_render 2>/dev/null; "
       "pkill -f [m]ap_pub 2>/dev/null; pkill -f [t]raj_server 2>/dev/null", 10)
    time.sleep(1)
    ea_d("roslaunch %s/hils_epic.launch odom_topic:=%s > %s/.A_epic.log 2>&1" % (WS, EPIC_ODOM, WS))
    # wait for the perception+planner to come up: cloud rendered from the odom pose
    for _ in range(40):
        if ea("timeout 3 rostopic hz /quad0_pcl_render_node/cloud 2>&1 | grep -q 'average rate' && echo y", 6)[1].strip() == "y":
            return True, "EPIC live up (pcl_render cloud flowing; planner ready)"
        time.sleep(1)
    return False, "EPIC perception not up (see .A_epic.log)"


def s_adapter():
    ff = "true" if FF else "false"
    inp = EPIC_CMD if EPIC else "/position_cmd"   # live EPIC -> /planning/pos_cmd
    ea("(nohup rosrun onboard_adapter px4_setpoint_adapter.py "
       "_auto_offboard:=true _auto_arm:=false _use_vel:=%s _use_accel:=%s _input_topic:=%s "
       ">%s/.A_adapter.log 2>&1 &)" % (ff, ff, inp, WS), 15)
    time.sleep(3)
    up = ea("pgrep -f [p]x4_setpoint_adapter >/dev/null && echo up", 8)[1].strip() == "up"
    return up, "setpoints OFFBOARD (in=%s, FF=%s)" % (inp, "ON" if FF else "off")


def set_ff():
    """Toggle adapter vel/accel feed-forward and restart it (for tighter trajectory
    tracking). Do this while landed/disarmed between tests."""
    global FF
    FF = not FF
    print("%s[ff]%s feed-forward -> %s (restarting adapter)" % (C, N, "ON" if FF else "off"))
    ea("pkill -f [p]x4_setpoint_adapter 2>/dev/null", 10)
    time.sleep(1)
    ok, msg = s_adapter()
    print("  " + msg)


def epic_explore():
    """Close the loop. EPIC's exploration band is ~1-3.5 m and the drone start
    (Gazebo origin) is at floor level, so it must be AIRBORNE (~2 m) before EPIC
    will plan (z=0.1 ground = floor collision -> optimize fails). Sequence: arm ->
    climb to ~2 m on the EPIC cmd topic (EPIC isn't planning yet) -> stop climb ->
    trigger EPIC (it takes over the topic) -> watch it explore."""
    if not EPIC:
        print("  not in EPIC mode - relaunch: python3 hils_menu.py gazebo_truth epic gui")
        return
    print("%s[epic]%s arm + climb to 2 m + trigger exploration" % (C, N))
    # 1) climb to ~2 m by holding the EPIC cmd topic (so the mapped garage pose
    #    lands in EPIC's exploration band). EPIC isn't publishing it yet.
    ea("pkill -f [t]est_hover_cmd 2>/dev/null", 8)
    time.sleep(1)
    ea("(nohup python3 %s/scripts/test_hover_cmd.py _abs_z:=2.0 /position_cmd:=%s "
       ">%s/.A_climb.log 2>&1 &)" % (WS, EPIC_CMD, WS), 10)
    time.sleep(3)
    print("  arming..."); arm(True)
    for i in range(12):
        time.sleep(2)
        gp = gz_pose()
        z = gp[2] if gp else 0.0
        print("  climb t=%2ds  gz_z=%.2f" % (i * 2, z))
        if z > 1.7:
            break
    # 2) stop the climb hold, trigger EPIC (it takes over EPIC_CMD)
    ea("pkill -f [t]est_hover_cmd 2>/dev/null", 8)
    time.sleep(1)
    print("  trigger exploration ...")
    ea("timeout 5 rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "
       "'{header: {frame_id: \"world\"}, pose: {position: {x: 8.0, y: 0.0, z: 2.0}, "
       "orientation: {w: 1.0}}}'", 8)
    # 3) watch EPIC plan + drone explore (and surface optimize failures)
    print("  exploring (EPIC plans -> drone follows):")
    for i in range(14):
        time.sleep(3)
        gp = gz_pose()
        cmd = ea("timeout 2 rostopic hz %s 2>&1 | grep -q 'average rate' && echo on" % EPIC_CMD, 5)[1].strip()
        print("  t=%2ds  gz=%s  epic_cmd=%s" %
              (i * 3, ["%.1f" % v for v in gp] if gp else "?", cmd or "-- (not planning)"))
    fail = ea("grep -c 'Optimization Failed' %s/.A_epic.log 2>/dev/null" % WS, 6)[1].strip()
    if fail and fail != "0":
        print("  %s(EPIC optimize failed x%s - drone likely not airborne / start in collision)%s" % (Y, fail, N))


def bring_up():
    ensure_up()
    steps = [
        ("Cleanup stale processes", s_cleanup),
        ("Detect PX4 board", s_detect),
        ("Ensure HITL enabled", s_hitl),
        ("Check HITL output module (pwm_out_sim)", s_check_output),
        ("Configure EKF = external odom (live, no reboot)", s_config_ekf),
        ("Start roscore", s_roscore),
        ("Start simulator (Gazebo + ros api)", s_sim),
        ("Start odom source [%s]" % MODE, s_odom_source),
    ]
    if EPIC:
        steps.append(("Start EPIC live (planner+perception)", s_epic))
    steps += [
        ("Start onboard MAVROS", s_mavros),
        ("EKF anchor + frame check", s_anchor),
        ("Verify board flyable (not terminated)", s_flyable),
        ("Start adapter (OFFBOARD)", s_adapter),
    ]
    total = len(steps)
    for i, (title, fn) in enumerate(steps, 1):
        print("%s[%2d/%d]%s %-42s ... " % (C, i, total, N, title), end="", flush=True)
        try:
            good, msg = fn()
        except Exception as e:
            print("%sFAIL%s %s" % (R, N, e))
            return title
        print(("%sOK%s  %s" % (G, N, msg)) if good else ("%sFAIL%s %s" % (R, N, msg)))
        if not good:
            return title       # name of the step that failed
    return None                # success


# ---------- test actions ----------
def arm(value):
    _, out, _ = ea('rosservice call /mavros/cmd/arming "value: %s"' % ("true" if value else "false"), 12)
    print("  " + " ".join(out.split()))


def takeoff(alt="1.5"):
    print("%s[takeoff]%s climb %s m" % (C, N, alt))
    ea("pkill -f [t]est_hover_cmd 2>/dev/null", 10)
    time.sleep(1)
    ea("(nohup python3 %s/scripts/test_hover_cmd.py _climb:=%s >%s/.A_hover.log 2>&1 &)" % (WS, alt, WS), 10)
    time.sleep(3)
    print("  arming...")
    arm(True)
    for i in range(7):
        time.sleep(2)
        gp = gz_pose()
        print("  t=%2ds  ekf_z=%s  gz_z=%s" % (i * 2 + 2, rfield("/mavros/local_position/odom/pose/pose/position/z"),
                                               ("%.2f" % gp[2]) if gp else "?"))


def land():
    print("%s[land]%s descend + disarm" % (C, N))
    ea("pkill -f [t]est_hover_cmd 2>/dev/null", 10)
    ea("(nohup python3 %s/scripts/test_hover_cmd.py _abs_z:=0.15 >%s/.A_hover.log 2>&1 &)" % (WS, WS), 10)
    for i in range(6):
        time.sleep(2)
        gp = gz_pose()
        print("  t=%2ds  gz_z=%s" % (i * 2 + 2, ("%.2f" % gp[2]) if gp else "?"))
    print("  disarming...")
    arm(False)
    ea("pkill -f [t]est_hover_cmd 2>/dev/null", 10)


def traj(shape="circle", alt="1.5", radius="2.0", period="20", laps="2"):
    """Path-tracking test: publish a moving /position_cmd (same topic EPIC drives)
    via test_traj_cmd.py and stream the tracking error (cmd vs Gazebo truth)."""
    print("%s[traj]%s %s  alt=%s R=%s period=%ss laps=%s" % (C, N, shape, alt, radius, period, laps))
    ea("pkill -f [t]est_hover_cmd 2>/dev/null; pkill -f [t]est_traj_cmd 2>/dev/null", 10)
    time.sleep(1)
    ea("(nohup python3 %s/scripts/test_traj_cmd.py _shape:=%s _alt:=%s _radius:=%s "
       "_period:=%s _laps:=%s >%s/.A_traj.log 2>&1 &)" % (WS, shape, alt, radius, period, laps, WS), 10)
    time.sleep(3)
    print("  arming..."); arm(True)
    dur = 8 + float(period) * float(laps) + 6
    seen = set()
    for _ in range(int(dur / 2) + 2):
        time.sleep(2)
        _, out, _ = ea("tail -4 %s/.A_traj.log 2>/dev/null" % WS, 6)
        for line in out.splitlines():
            if "[traj]" in line and ("t=" in line or "DONE" in line or "RMS" in line):
                msg = line.split("[traj]")[-1].strip()
                if msg not in seen:
                    seen.add(msg)
                    print("  " + msg)
        if "DONE" in out:
            break
    print("  landing...")
    land()


def epic_replay(speed="0.7"):
    """Replay the recorded EPIC trajectory (epic_traj.bag) onto /position_cmd and
    stream the tracking error. The bag is captured from EPIC's native MARSIM demo
    (/planning/pos_cmd); epic_replay.py offsets it to the HITL drone's start."""
    print("%s[epic-replay]%s recorded EPIC trajectory  speed=%s" % (C, N, speed))
    rc, _, _ = sh("test -f %s/epic_traj.bag && echo y" % "/home/dh/poongsan_hils", 6)
    ea("pkill -f [t]est_hover_cmd 2>/dev/null; pkill -f [t]est_traj_cmd 2>/dev/null; pkill -f [e]pic_replay 2>/dev/null", 10)
    time.sleep(1)
    ea("(nohup python3 %s/scripts/epic_replay.py _bag:=%s/epic_traj.bag _alt:=1.5 _speed:=%s "
       ">%s/.A_replay.log 2>&1 &)" % (WS, WS, speed, WS), 10)
    time.sleep(3)
    print("  arming..."); arm(True)
    dur = 8 + 42.0 / float(speed) + 6
    seen = set()
    for _ in range(int(dur / 2) + 2):
        time.sleep(2)
        _, out, _ = ea("tail -4 %s/.A_replay.log 2>/dev/null" % WS, 6)
        for line in out.splitlines():
            if "[replay]" in line and any(k in line for k in ("t=", "DONE", "loaded", "offset")):
                msg = line.split("[replay]")[-1].strip()
                if msg not in seen:
                    seen.add(msg)
                    print("  " + msg)
        if "DONE" in out:
            break
    print("  landing...")
    land()


def frame_check():
    print("%s[frame check]%s comparing frames (should all agree):" % (C, N))
    gp = gz_pose()
    ox = rfield("/mavros/local_position/odom/pose/pose/position/x")
    oy = rfield("/mavros/local_position/odom/pose/pose/position/y")
    oz = rfield("/mavros/local_position/odom/pose/pose/position/z")
    lz = rfield("/lidar_slam/odom/pose/pose/position/z")
    print("  Gazebo truth      : %s" % (["%.2f" % v for v in gp] if gp else "?"))
    print("  MAVROS odom (xyz) : [%s, %s, %s]" % (ox, oy, oz))
    print("  /lidar_slam/odom z: %s" % lz)
    if gp and oz is not None:
        dz = abs(oz - gp[2])
        print("  |odom_z - gz_z|   : %.3f m  -> %s" % (dz, "CONSISTENT" if dz < 0.3 else "MISMATCH"))


def status():
    print(ea("timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep -E 'connected|armed|mode'", 8)[1].strip() or "  (no mavros)")
    frame_check()


def down():
    s_cleanup()
    print("  all stopped.")


def test_menu():
    while True:
        print("\n%s===== TEST MENU (HILS READY, mode=%s) =====%s" % (G, MODE, N))
        print(" 1) Takeoff & hover (1.5 m)")
        print(" 2) Land & disarm")
        print(" 3) Frame check (Gazebo vs board vs EPIC)")
        print(" 4) Status")
        print(" 5) Stop all (down)")
        print(" 6) Trajectory test - circle (path tracking)")
        print(" 7) Trajectory test - figure8")
        print(" 8) EPIC trajectory replay (recorded epic_traj.bag)")
        print(" 9) Toggle feed-forward (vel/acc) [now: %s]" % ("ON" if FF else "off"))
        if EPIC:
            print(" e) EPIC explore - CLOSED LOOP (arm + trigger autonomous exploration)")
        print(" q) Quit")
        try:
            sel = input("test> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if sel == "1":
            takeoff("1.5")
        elif sel == "2":
            land()
        elif sel == "3":
            frame_check()
        elif sel == "4":
            status()
        elif sel == "5":
            down(); break
        elif sel == "6":
            traj("circle")
        elif sel == "7":
            traj("figure8")
        elif sel == "8":
            epic_replay()
        elif sel == "9":
            set_ff()
        elif sel == "e":
            epic_explore()
        elif sel == "q":
            break


def main():
    global MODE, GUI, EPIC, EV_VEL
    args = sys.argv[1:]
    for a in args:
        if a in ("gazebo_truth", "onboard_odom"):
            MODE = a
        elif a in ("gui", "--gui"):
            GUI = True
        elif a in ("epic", "--epic"):
            EPIC = True
        elif a in ("pose_only", "--pose-only"):
            EV_VEL = False
        elif a in ("pose_vel", "--pose-vel"):
            EV_VEL = True
    print("%s========== EPIC HILS Launcher (mode=%s%s%s) ==========%s"
          % (C, MODE, ", EPIC-live" if EPIC else "", ", GUI" if GUI else "", N))
    print("Bring-up: external-odometry aiding (GPS/baro off); gazebo+roscore reused,\n"
          "board NOT rebooted (keeps the HIL_SENSOR stream continuous).\n")
    # The EKF external-vision fusion occasionally diverges after a reboot; s_anchor
    # already tries to recover it (EV reset). As a final guarantee, retry the whole
    # clean bring-up a few times so the user reliably reaches READY.
    # Bring-up loop: a flight-TERMINATION failure -> prompt the operator to cold
    # power-cycle the board, then continue automatically. Any other (transient)
    # failure -> a few clean re-runs. This is the operator-prompt pattern used on
    # automated HITL test benches.
    generic_retries = 0
    while True:
        failed = bring_up()
        if failed is None:
            print("\n%s========== HILS READY ==========%s" % (G, N))
            test_menu()
            return
        # Both a latched flight TERMINATION and a diverged EKF (anchor never
        # locks) only clear on a cold boot -> prompt the operator to power-cycle.
        low = failed.lower()
        if ("flyable" in low) or ("terminat" in low) or ("anchor" in low):
            if not wait_powercycle():
                print("\n%sNo board after power-cycle; aborting.%s" % (R, N))
                sys.exit(1)
            generic_retries = 0          # fresh board -> clean slate
            continue
        generic_retries += 1
        if generic_retries >= 3:
            print("\n%sBring-up failed at '%s' after %d clean retries.%s" % (R, failed, generic_retries, N))
            sys.exit(1)
        print("\n%s--- '%s' failed; retrying clean (%d) ---%s" % (Y, failed, generic_retries, N))


if __name__ == "__main__":
    main()
