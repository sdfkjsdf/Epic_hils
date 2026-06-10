#!/usr/bin/env python3
"""
check_hil_output.py - Verify (and optionally fix) the PX4 HITL actuator-output path.

THE PROBLEM THIS GUARDS AGAINST
-------------------------------
In HITL the board must convert its mixer output (actuator_motors) into
HIL_ACTUATOR_CONTROLS so the simulator can spin the motors. PX4 produces that
stream from the `actuator_outputs_sim` topic, which is published ONLY by the
`pwm_out_sim` module. That module is a build-time option and is NOT compiled into
every board's firmware. If it is missing, the whole chain looks healthy
(armed + OFFBOARD + correct EKF + fresh setpoints) but the simulated drone never
moves, because no actuator output is ever sent.

This is the exact situation described in the official PX4 HITL guide. This tool
detects it the same way the guide tells you to (`pwm_out_sim status` ->
"command not found") and prints the official remedy, including the doc links.

Usage:
    python3 check_hil_output.py                  # auto-detect board, diagnose
    python3 check_hil_output.py --conn DEV/URL   # use a specific mavlink endpoint
    python3 check_hil_output.py --fix            # also enable the module in the
                                                 # board config and (if the ARM
                                                 # toolchain is present) rebuild
                                                 # + flash the firmware

Exit codes: 0 = output module present (OK), 2 = no board, 3 = module missing.
All user-facing output is in English on purpose (portable across terminals).
"""
import argparse
import glob
import os
import subprocess
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write("[ERROR] pymavlink not installed. Run: pip3 install pymavlink\n")
    sys.exit(1)

# ----------------------------------------------------------------------------
# Official PX4 references for this exact problem (printed in the remedy).
# ----------------------------------------------------------------------------
DOCS = [
    ("PX4 HITL Guide (main) - pwm_out_sim section",
     "https://docs.px4.io/main/en/simulation/hitl"),
    ("PX4 HITL Guide (v1.16)",
     "https://docs.px4.io/v1.16/en/simulation/hitl"),
    ("PX4 Board Configuration (Kconfig)",
     "https://docs.px4.io/main/en/hardware/porting_guide_config"),
    ("Source: PX4-user_guide/en/simulation/hitl.md (GitHub)",
     "https://github.com/PX4/PX4-user_guide/blob/main/en/simulation/hitl.md"),
    ("Example board config with the key set: boards/px4/fmu-v6x/default.px4board (GitHub)",
     "https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/default.px4board"),
]

CONFIG_KEY = "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y"
# Board target for this rig (MicoAir743v2). Override with --board.
DEFAULT_BOARD = "micoair_h743-v2"
DEFAULT_BOARD_FILE = "boards/micoair/h743-v2/default.px4board"

SHELL_DEV = 10  # SERIAL_CONTROL_DEV_SHELL


def candidate_ports():
    ports = sorted(glob.glob("/dev/serial/by-id/*"))
    ports += sorted(glob.glob("/dev/ttyACM*"))
    ports += sorted(glob.glob("/dev/ttyUSB*"))
    seen, out = set(), []
    for p in ports:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def connect(conn):
    """Open a mavlink connection (serial dev or udp/tcp url). Return (m, desc) or None."""
    targets = [conn] if conn else candidate_ports()
    for t in targets:
        try:
            if t.startswith("/dev/") or t.startswith("/"):
                m = mavutil.mavlink_connection(t, baud=115200)   # serial device
            else:
                m = mavutil.mavlink_connection(t)                # udpin:/udpout:/tcp: url
        except Exception:
            continue
        if m.wait_heartbeat(timeout=5) is not None:
            return m, t
        try:
            m.close()
        except Exception:
            pass
    return None


def nsh_run(m, cmd, settle=2.5):
    """Run a command on the PX4 nsh console over MAVLink SERIAL_CONTROL; return output."""
    flags = (mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
             mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI)

    def send(chunk):
        buf = list(chunk) + [0] * (70 - len(chunk))
        m.mav.serial_control_send(SHELL_DEV, flags, 0, 0, len(chunk), buf)

    data = (cmd + "\n").encode()
    for i in range(0, len(data), 70):
        send(data[i:i + 70])

    out = b""
    t0 = time.time()
    while time.time() - t0 < settle:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.4)
        if msg is None:
            send(b"")  # poll for more output
            continue
        if msg.count > 0:
            out += bytes(msg.data[:msg.count])
    return out.decode(errors="ignore")


def module_present(m):
    """True if pwm_out_sim exists in the firmware, False if not, None if undetermined."""
    out = nsh_run(m, "pwm_out_sim status")
    low = out.lower()
    if "command not found" in low:
        return False
    if "running" in low or "never" in low or "actuator" in low or "pwm_out_sim" in low:
        return True
    return None  # could not tell from nsh; caller may fall back to param check


def param_fallback(m):
    """Fallback: the HIL module advertises HIL_ACT_* params; absence => not compiled."""
    for name in ("HIL_ACT_FUNC1", "HIL_ACT_FUNC01", "HIL_ACT_MIN1"):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        for _ in range(10):
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if not msg:
                break
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors="ignore")
            if pid.strip("\x00") == name:
                return True
    return False


def print_remedy(board, board_file):
    print("")
    print("=" * 72)
    print("  HITL ACTUATOR-OUTPUT MODULE IS MISSING (pwm_out_sim not in firmware)")
    print("=" * 72)
    print("  Symptom : board arms in OFFBOARD with correct setpoints, but the")
    print("            simulated drone never moves (no HIL_ACTUATOR_CONTROLS sent).")
    print("  Cause   : `pwm_out_sim` is a build-time module and is not compiled")
    print("            into this board's firmware, so `actuator_outputs_sim` is")
    print("            never published and the HIL_ACTUATOR_CONTROLS stream is empty.")
    print("")
    print("  OFFICIAL FIX (this is the PX4-documented procedure, not a hack):")
    print("    1. Add this key to %s :" % board_file)
    print("         %s" % CONFIG_KEY)
    print("       (equivalently: `make %s boardconfig`" % board)
    print("        -> modules > Simulation > pwm_out_sim)")
    print("    2. Rebuild  :  make %s" % board)
    print("    3. Flash    :  make %s upload   (or upload the .px4 via QGroundControl)" % board)
    print("    4. Make sure the airframe is a HIL airframe (e.g. SYS_AUTOSTART=1001,")
    print("       'HIL Quadcopter X').")
    print("")
    print("    Or run this tool with --fix to apply step 1 automatically and, if the")
    print("    ARM toolchain is available, perform steps 2-3 as well.")
    print("")
    print("  References:")
    for title, url in DOCS:
        print("    - %s" % title)
        print("      %s" % url)
    print("=" * 72)


# ----------------------------------------------------------------------------
# --fix helpers
# ----------------------------------------------------------------------------
def patch_board_config(px4_root, board_file):
    """Idempotently add the pwm_out_sim key to the board config. Return True if OK."""
    path = os.path.join(px4_root, board_file)
    if not os.path.isfile(path):
        print("[fix] board config not found: %s" % path)
        return False
    lines = open(path).read().splitlines(keepends=True)
    if any(ln.strip() == CONFIG_KEY for ln in lines):
        print("[fix] %s already present in %s" % (CONFIG_KEY, board_file))
        return True
    out, done = [], False
    for ln in lines:
        out.append(ln)
        # keep the list roughly alphabetical: insert after MODULES_SENSORS
        if ln.strip() == "CONFIG_MODULES_SENSORS=y" and not done:
            out.append(CONFIG_KEY + "\n")
            done = True
    if not done:  # no anchor: just append in the modules area
        out.append(CONFIG_KEY + "\n")
    try:
        open(path, "w").writelines(out)
    except PermissionError:
        print("[fix] permission denied writing %s (run inside the build container as root)" % path)
        return False
    print("[fix] added %s to %s" % (CONFIG_KEY, board_file))
    return True


def have_toolchain():
    return subprocess.call("command -v arm-none-eabi-gcc >/dev/null 2>&1", shell=True) == 0


def build_and_flash(px4_root, board):
    if not have_toolchain():
        print("[fix] ARM toolchain (arm-none-eabi-gcc) not found - cannot build here.")
        print("[fix] Config is patched. Build + flash from a PX4 build environment:")
        print("        cd %s && make %s upload" % (px4_root, board))
        return False
    print("[fix] building %s (this can take several minutes) ..." % board)
    if subprocess.call("make %s" % board, shell=True, cwd=px4_root) != 0:
        print("[fix] build failed.")
        return False
    print("[fix] flashing %s (put the board in bootloader if prompted) ..." % board)
    if subprocess.call("make %s upload" % board, shell=True, cwd=px4_root) != 0:
        print("[fix] upload failed.")
        return False
    print("[fix] done. Reboot the board and re-run the HITL bring-up.")
    return True


def main():
    ap = argparse.ArgumentParser(description="Check/fix the PX4 HITL actuator-output path.")
    ap.add_argument("--conn", help="mavlink endpoint (serial dev or udp/tcp url); default auto-detect")
    ap.add_argument("--fix", action="store_true", help="enable the module in the board config (+build/flash if possible)")
    ap.add_argument("--board", default=DEFAULT_BOARD, help="PX4 board target (default %s)" % DEFAULT_BOARD)
    ap.add_argument("--board-file", default=DEFAULT_BOARD_FILE, help="board .px4board path relative to PX4 root")
    ap.add_argument("--px4-root", default=os.environ.get("PX4_ROOT", ""),
                    help="PX4-Autopilot dir (for --fix); default $PX4_ROOT or auto")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    def log(msg):
        if not args.quiet:
            print(msg)

    log("[*] Connecting to the board to check the HITL output path ...")
    res = connect(args.conn)
    if res is None:
        sys.stderr.write("[ERROR] No PX4 board / mavlink endpoint found.\n")
        sys.exit(2)
    m, desc = res
    log("[OK] connected via %s" % desc)

    present = module_present(m)
    if present is None:
        log("[*] nsh check inconclusive; trying parameter fallback ...")
        present = param_fallback(m)

    if present:
        print("[OK] pwm_out_sim is present - HITL actuator-output path is available.")
        sys.exit(0)

    # Missing.
    px4_root = args.px4_root or _guess_px4_root()
    print_remedy(args.board, args.board_file)

    if args.fix:
        if not px4_root:
            print("[fix] could not locate PX4-Autopilot; pass --px4-root.")
            sys.exit(3)
        print("[fix] using PX4 root: %s" % px4_root)
        if patch_board_config(px4_root, args.board_file):
            build_and_flash(px4_root, args.board)
    sys.exit(3)


def _guess_px4_root():
    for c in ("/root/poongsan_hils/PX4-Autopilot",
              os.path.expanduser("~/poongsan_hils/PX4-Autopilot"),
              os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "PX4-Autopilot")):
        if os.path.isdir(c):
            return c
    return ""


if __name__ == "__main__":
    main()
