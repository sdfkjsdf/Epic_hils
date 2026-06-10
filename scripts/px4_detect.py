#!/usr/bin/env python3
"""
px4_detect.py - Auto-detect a PX4 flight controller on USB and check HITL readiness.

Scans serial ports, identifies the PX4 board by its MAVLink heartbeat (so it does
not matter whether it shows up as ttyACM0, ttyACM1, ...), prints the device path
and key status. If HITL (SYS_HITL) is not enabled, it tells you how to enable it.

Usage:
    python3 px4_detect.py                 # detect + status only
    python3 px4_detect.py --enable-hitl   # also set SYS_HITL=1 and reboot the board
    python3 px4_detect.py --quiet         # print only the detected device path (for scripts)

Exit codes: 0 = found & HITL enabled, 2 = no board, 3 = found but HITL disabled.
All user-facing output is in English on purpose (portable across terminals).
"""
import glob
import sys
import struct
import time
import argparse

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write("[ERROR] pymavlink not installed. Run: pip3 install pymavlink\n")
    sys.exit(1)

INT_TYPES = (1, 2, 3, 4, 5, 6)  # MAV_PARAM_TYPE uint8..int32


def candidate_ports():
    """Stable by-id names first, then raw ACM/USB nodes (deduped, order kept)."""
    ports = sorted(glob.glob("/dev/serial/by-id/*"))
    ports += sorted(glob.glob("/dev/ttyACM*"))
    ports += sorted(glob.glob("/dev/ttyUSB*"))
    seen, out = set(), []
    for p in ports:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def try_port(port, timeout=4):
    try:
        m = mavutil.mavlink_connection(port, baud=115200)
    except Exception:
        return None
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        try:
            m.close()
        except Exception:
            pass
        return None
    return m, hb


def read_param(m, name, tries=8):
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        for _ in range(20):
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if not msg:
                break
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors="ignore")
            if pid.strip("\x00") == name:
                v = msg.param_value
                if msg.param_type in INT_TYPES:  # int stored in float field -> reinterpret
                    v = struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
                return v
    return None


def set_param_int(m, name, value):
    # PX4 carries int params as the int32 *bytes* reinterpreted into the float
    # param_value field. Sending float(value) directly would be stored wrong
    # (e.g. 1.0 -> 0x3F800000 -> 1065353216), so reinterpret the bytes here.
    fval = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         fval, mavutil.mavlink.MAV_PARAM_TYPE_INT32)


def prompt_yes(question):
    try:
        ans = input(question + " [y/N]: ").strip().lower()
    except EOFError:
        ans = ""
    return ans in ("y", "yes")


def main():
    ap = argparse.ArgumentParser(description="Detect PX4 board and check HITL readiness.")
    ap.add_argument("--enable-hitl", action="store_true",
                    help="Set SYS_HITL=1 and reboot the board")
    ap.add_argument("--quiet", action="store_true",
                    help="Print only the detected device path")
    args = ap.parse_args()

    def log(msg):
        if not args.quiet:
            print(msg)

    log("[*] Scanning serial ports for a PX4 board...")
    found = None
    for port in candidate_ports():
        log("    trying %s ..." % port)
        res = try_port(port)
        if res is None:
            continue
        m, hb = res
        if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            found = (port, m, hb)
            break
        try:
            m.close()
        except Exception:
            pass

    if not found:
        sys.stderr.write("[ERROR] No PX4 board found.\n")
        sys.stderr.write("        - Is the board plugged in via USB?\n")
        sys.stderr.write("        - Permissions: add your user to the 'dialout' group, or run inside the container.\n")
        sys.exit(2)

    port, m, hb = found
    if args.quiet:
        print(port)
        # still honor exit code based on HITL
        hitl = read_param(m, "SYS_HITL")
        sys.exit(0 if hitl == 1 else 3)

    print("[OK] PX4 board detected")
    print("     device   : %s" % port)
    print("     mav_type : %d (2 = quadrotor)" % hb.type)

    hitl = read_param(m, "SYS_HITL")
    print("     SYS_HITL : %s" % hitl)

    if hitl == 1:
        print("[OK] HITL is ENABLED. The board is ready for HITL simulation.")
        sys.exit(0)

    # Not properly enabled (0, or a wrong/garbage value): ask the user.
    print("")
    print("[!] HITL is NOT enabled (SYS_HITL = %s, expected 1)." % hitl)
    print("    The board is not in HITL mode; the simulation will not work.")

    do_set = args.enable_hitl or prompt_yes("Set SYS_HITL = 1 now?")
    if not do_set:
        print("    Skipped. Enable HITL and reboot the board before running the simulation.")
        sys.exit(3)

    set_param_int(m, "SYS_HITL", 1)
    time.sleep(1.0)
    print("[OK] SYS_HITL is now: %s" % read_param(m, "SYS_HITL"))
    print("")
    print("    >>> Please REBOOT the board now to apply the change. <<<")
    sys.exit(0)


if __name__ == "__main__":
    main()
