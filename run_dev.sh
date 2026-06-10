#!/usr/bin/env bash
# =============================================================================
# poongsan HILS (container B) - long-lived dev container launcher / re-attach
#  - Inside: Ubuntu 20.04 + Gazebo 11 (osrf/ros:noetic-desktop-full)
#  - Workspace (/home/dh/poongsan_hils) bind-mounted -> edits persist on host
#  - GPU / host network / X11 (GUI) / USB (Pixhawk) passthrough
#  - Dev shell (dev_bashrc.sh): command auto-logging + 'rec' transcription
# Usage:  ./run_dev.sh        (create if missing, else re-attach)
# =============================================================================
set -e

NAME=poongsan_hils_dev
IMAGE=osrf/ros:noetic-desktop-full
HOST_DIR=/home/dh/poongsan_hils
CONT_DIR=/root/poongsan_hils

xhost +local:root >/dev/null 2>&1 || true   # GUI permission

# Create detached (keep-alive) if it does not exist
if ! docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "[run_dev] creating new dev container '$NAME'"
    docker run -d --name "$NAME" \
        --network host --gpus all --privileged \
        -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /dev:/dev \
        -v "$HOST_DIR":"$CONT_DIR" \
        -w "$CONT_DIR" \
        "$IMAGE" sleep infinity
fi

docker start "$NAME" >/dev/null

# Ensure the dev shell is sourced (idempotent)
docker exec "$NAME" bash -lc \
  'grep -q "poongsan_hils/dev_bashrc.sh" ~/.bashrc 2>/dev/null || echo "source /root/poongsan_hils/dev_bashrc.sh" >> ~/.bashrc'

echo "[run_dev] attaching to '$NAME' (container stays up after exit)"
exec docker exec -it "$NAME" bash

# Notes:
#  - USB (Pixhawk): during dev we use --privileged + -v /dev:/dev for hotplug.
#    For the final image, narrow to --device=/dev/ttyACM0.
#  - Remove the container completely: docker rm -f poongsan_hils_dev
