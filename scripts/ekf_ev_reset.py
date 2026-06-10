#!/usr/bin/env python3
"""
ekf_ev_reset.py - Force PX4 EKF2 to reset horizontal position to external vision.

After a board reboot the EKF initialises before the external-vision stream is up
(gazebo + bridge start a bit later). During that window the horizontal estimate
drifts; when vision finally arrives the innovation gate can reject it and the
estimate stays diverged (seen: gazebo truth (0,0) but EKF (-38,-20)). Toggling
the EV aiding control (EKF2_EV_CTRL off -> on) makes EKF2 re-initialise EV fusion
and reset horizontal position to the (already-flowing) EV measurement - no board
reboot, so the gazebo HITL link stays up.

Runs over any mavlink endpoint; default udp:14550 (gazebo forwards it).
EKF2_EV_CTRL=11 -> horizontal pos + vertical pos + yaw (the value the bring-up uses).

Usage: python3 ekf_ev_reset.py [conn]   # conn e.g. udpin:0.0.0.0:14550 or /dev/ttyACM0
"""
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write("pymavlink not installed\n")
    sys.exit(1)

EV_CTRL_ON = 11   # must match configure_ekf_extodom.py

conn = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14550"


def connect(c):
    if c.startswith("/dev/") or c.startswith("/"):
        m = mavutil.mavlink_connection(c, baud=115200)
    else:
        m = mavutil.mavlink_connection(c)
    if m.wait_heartbeat(timeout=8) is None:
        sys.stderr.write("no heartbeat on %s\n" % c)
        sys.exit(2)
    return m


def set_int(m, name, val):
    # PX4 int params travel as the int32 *bytes* reinterpreted into the float
    # field (sending 1.0 directly would store 0x3F800000). See px4_detect.py.
    fval = struct.unpack("<f", struct.pack("<i", int(val)))[0]
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         fval, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.4)


def main():
    m = connect(conn)
    set_int(m, "EKF2_EV_CTRL", 0)    # stop EV fusion
    time.sleep(1.0)
    set_int(m, "EKF2_EV_CTRL", EV_CTRL_ON)  # restart -> resets H-pos to EV
    print("EKF2_EV_CTRL toggled 0 -> %d (EV horizontal-position reset requested)" % EV_CTRL_ON)


if __name__ == "__main__":
    main()
