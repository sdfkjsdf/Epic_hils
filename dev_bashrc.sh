# =============================================================================
# poongsan HILS dev shell - sourced by the dev container's ~/.bashrc
# Enforces the "works on my machine" discipline with tooling:
#   (1) safety net : log every command to a host file (.dev_history) with timestamp
#   (2) transcribe : 'rec <cmd>' runs cmd and, on success, appends it to setup_hils.sh
# =============================================================================

# PX4 NuttX firmware build toolchain (arm-none-eabi-gcc 10.3-2021.10, see setup [7])
[ -d /opt/gcc-arm-none-eabi/bin ] && export PATH=/opt/gcc-arm-none-eabi/bin:$PATH

# ROS environment
source /opt/ros/noetic/setup.bash 2>/dev/null || true
[ -f /root/poongsan_hils/catkin_ws/devel/setup.bash ] && \
    source /root/poongsan_hils/catkin_ws/devel/setup.bash 2>/dev/null || true

# --- (1) command history safety net: persist to host immediately ---
export HISTTIMEFORMAT='%F %T  '
export HISTFILE=/root/poongsan_hils/.dev_history
export HISTSIZE=100000
export HISTFILESIZE=200000
shopt -s histappend
export PROMPT_COMMAND='history -a'   # flush to host file after each command

SETUP=/root/poongsan_hils/setup_hils.sh

# --- (2) rec: run a command, transcribe to setup_hils.sh on success ---
rec() {
    echo "+ $*"
    if "$@"; then
        printf '%s\n' "$*" >> "$SETUP"
        echo "[rec] OK -> appended to setup_hils.sh"
    else
        local rc=$?
        echo "[rec] FAILED (rc=$rc) -> not recorded"
        return $rc
    fi
}

# --- note: append a comment / manual step to setup_hils.sh (no execution) ---
note() { printf '# %s\n' "$*" >> "$SETUP"; echo "[note] added"; }

# --- recall: show history commands not yet in setup_hils.sh (catch omissions) ---
recall() {
    echo "=== commands in .dev_history but not in setup_hils.sh ==="
    grep -vE '^#' "$HISTFILE" 2>/dev/null | grep -vE '^(ls|cd|cat|rec|note|recall|history|clear)\b' \
        | sort -u | while read -r line; do
            grep -qxF "$line" "$SETUP" 2>/dev/null || echo "  $line"
        done
}

echo "[dev] HILS dev shell loaded - log:.dev_history | rec <cmd> | note <text> | recall"
