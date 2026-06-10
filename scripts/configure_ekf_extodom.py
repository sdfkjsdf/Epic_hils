#!/usr/bin/env python3
"""
configure_ekf_extodom.py - Set the PX4 board's EKF2 to use EXTERNAL odometry
(vision) as the position/height source and disable GPS + baro.

This mirrors a real EPIC drone: GPS-denied indoor flight where the FC gets its
pose from LiDAR-inertial odometry (external), not GPS. In HITL we feed Gazebo
ground-truth (or, later, real LiDAR odometry) into the same external-vision input.

These are PX4 runtime PARAMETERS (like SYS_HITL) - the firmware is unchanged.
Auto-detects the board port; reboots to apply.

Params set:
  EKF2_EV_CTRL   = 15  (use external odometry for H-pos + V-pos + VELOCITY + yaw;
                        bit2=velocity added now that the bridge feeds full
                        nav_msgs/Odometry on /mavros/odometry/out)
  EKF2_HGT_REF   = 3   (height reference = Vision)
  EKF2_GPS_CTRL  = 0   (disable GPS fusion)
  EKF2_BARO_CTRL = 0   (disable barometer fusion)
  EKF2_EV_DELAY  = 0.0 (no EV delay in sim)
  COM_ARM_WO_GPS = 1   (allow arming without GPS)
  NAV_RCL_ACT    = 0   (RC-loss failsafe DISABLED - HITL has no RC; default
                        action Return+no GPS would escalate to flight TERMINATION)
  COM_RCL_EXCEPT = 4   (except Offboard from RC-loss handling)
"""
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write("pymavlink not installed\n")
    sys.exit(1)

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = _args[0] if _args else "/dev/ttyACM0"
# --no-reboot: set params live and DO NOT reboot. The board reboot drops the
# gazebo HIL_SENSOR feed -> gyro timeout -> flight TERMINATION latch. On a board
# that already has these params persisted (cold boot loads them), live-setting is
# a no-op for the init params, so skipping the reboot keeps the sensor stream
# continuous and avoids the termination.
NO_REBOOT = "--no-reboot" in sys.argv
# --pose-only: fuse external odom POSITION + yaw only (EKF2_EV_CTRL=11), ignoring the
# velocity the bridge sends. Default fuses velocity too (15 = pos+vel+yaw). The bridge
# always publishes full nav_msgs/Odometry; this just selects what EKF2 fuses, so you
# can compare pose-only vs pose+vel tracking by re-running with/without this flag.
POSE_ONLY = "--pose-only" in sys.argv
EV_CTRL = 11 if POSE_ONLY else 15
INT_TYPES = (1, 2, 3, 4, 5, 6, 7, 8)

# name -> value (type auto-detected from the board)
PARAMS = [
    ("EKF2_EV_CTRL", EV_CTRL),  # 11=H-pos+V-pos+yaw (pose-only) / 15=+velocity (default)
    ("EKF2_HGT_REF", 3),
    ("EKF2_GPS_CTRL", 0),
    ("EKF2_BARO_CTRL", 0),
    ("EKF2_EV_DELAY", 0.0),
    ("COM_ARM_WO_GPS", 1),
    # HITL has no RC/GCS. Default RC-loss action (Return) + no GPS cannot execute
    # RTL -> escalates to flight TERMINATION (latched). Disable it for HITL.
    ("NAV_RCL_ACT", 0),
    ("COM_RCL_EXCEPT", 4),
    # No RC/joystick at all in HITL. COM_RC_IN_MODE=4 (stick input disabled) makes
    # the FC stop expecting manual control, so manual_control_signal_lost no longer
    # feeds the failsafe state machine (which was intermittently -> TERMINATION).
    ("COM_RC_IN_MODE", 4),
    ("NAV_DLL_ACT", 0),   # data-link/GCS-loss action disabled (no QGC in HITL)
]


def get_param(m, name, tries=8):
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        for _ in range(20):
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if not msg:
                break
            pid = msg.param_id.decode(errors="ignore") if isinstance(msg.param_id, bytes) else msg.param_id
            if pid.strip("\x00") == name:
                v = msg.param_value
                if msg.param_type in INT_TYPES:
                    v = struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
                return v, msg.param_type
    return None, None


def set_param(m, name, value):
    _, ptype = get_param(m, name)
    if ptype is None:
        print("  %-16s : NOT FOUND (skipped)" % name)
        return False
    if ptype in INT_TYPES:
        fval = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        send_type = ptype
    else:
        fval = float(value)
        send_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), fval, send_type)
    time.sleep(0.3)
    got, _ = get_param(m, name)
    ok = (got is not None) and (abs(float(got) - float(value)) < 1e-3)
    print("  %-16s -> %-6s %s" % (name, str(value), "OK" if ok else "(read back %s)" % got))
    return ok


def main():
    print("[ekf-cfg] connecting to %s ..." % PORT)
    m = mavutil.mavlink_connection(PORT, baud=115200)
    if m.wait_heartbeat(timeout=8) is None:
        sys.stderr.write("[ekf-cfg] no heartbeat\n")
        sys.exit(2)
    print("[ekf-cfg] setting EKF2 for external odometry (GPS/baro off):")
    for name, val in PARAMS:
        set_param(m, name, val)
    if NO_REBOOT:
        print("[ekf-cfg] --no-reboot: params set live, NOT rebooting (keeps HIL_SENSOR continuous).")
    else:
        print("[ekf-cfg] rebooting board to apply ...")
        m.reboot_autopilot()
        print("[ekf-cfg] done. Reconnect after a few seconds.")


if __name__ == "__main__":
    main()
