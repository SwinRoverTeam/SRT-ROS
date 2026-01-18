#!/bin/bash

# Make sure you have a controller plugged in
# Allow X11 forwarding for GUI tools
xhost +local:docker 2>/dev/null || true

echo "=========================================="
echo "Starting Rover Ground Control System"
echo "=========================================="
echo ""
echo "Make sure your controller is plugged in!"
echo "Press Ctrl+C to stop all nodes"
echo ""

docker run -it --rm \
  --name rover_ground_control \
  --network host \
  --privileged \
  --device /dev/input \
  -e DISPLAY=$DISPLAY \
  -e LAUNCH_MODE=integrated \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  rover-control: latest