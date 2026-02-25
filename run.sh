#!/bin/bash

# Make sure you have a controller plugged in
# Allow X11 forwarding for GUI tools
xhost +local:docker 2>/dev/null || true

# Diagnostic checks: ensure host display/socket are available before running container
echo "Host DISPLAY=$DISPLAY"
if [ -z "${DISPLAY:-}" ]; then
  echo "[ERROR] Host DISPLAY is empty. Ensure an X server is running and DISPLAY is set." >&2
  echo "Run on the host: echo \$DISPLAY && ls -ld /tmp/.X11-unix" >&2
  exit 1
fi

if [ ! -e /tmp/.X11-unix ]; then
  echo "[ERROR] /tmp/.X11-unix socket not found on host. X11 socket must be present to forward display." >&2
  echo "If using Wayland, enable XWayland or use a different approach (QT_QPA_PLATFORM=offscreen or Xvfb)." >&2
  exit 1
fi

echo "=========================================="
echo "Starting Rover Ground Control System"
echo "=========================================="
echo ""
echo "Make sure your controller is plugged in!"
echo "Press Ctrl+C to stop all nodes"
echo ""

# In order, this command:
# Enables container interactive mode,
# Sets the container to remove itself once it exits,
# Sets the container name,
# Sets the container network,
# Sets the container level of privilege,
# Sets environment variables,
# Binds X11 unix socket to container,
# Specifies the image file
docker run -it --rm \
  --name rover_ground_control \
  --network host \
  --privileged \
  --env DISPLAY=$DISPLAY \
  --env LAUNCH_MODE=integrated \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  swinroverteam/srt-ros:latest