#!/usr/bin/env python3
"""Wait until a MAVLink heartbeat is seen, then exit 0 (else 1 on timeout).

Used by hils.sh to gate each bring-up step on real readiness (not just a node
existing). Works for the board serial or the Gazebo-forwarded UDP port.

Usage:
    wait_ready.py serial /dev/ttyACM0 [timeout_s]
    wait_ready.py udp 14540 [timeout_s]
"""
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write("[wait_ready] pymavlink not installed\n")
    sys.exit(2)

if len(sys.argv) < 3:
    sys.stderr.write(__doc__)
    sys.exit(2)

kind, target = sys.argv[1], sys.argv[2]
timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
deadline = time.time() + timeout

while time.time() < deadline:
    try:
        if kind == "serial":
            m = mavutil.mavlink_connection(target, baud=115200)
        elif kind == "udp":
            m = mavutil.mavlink_connection("udpin:0.0.0.0:%s" % target)
        else:
            sys.stderr.write(__doc__)
            sys.exit(2)
        hb = m.wait_heartbeat(timeout=max(1.0, min(5.0, deadline - time.time())))
        try:
            m.close()
        except Exception:
            pass
        if hb is not None:
            sys.exit(0)
    except Exception:
        time.sleep(1.0)

sys.exit(1)
