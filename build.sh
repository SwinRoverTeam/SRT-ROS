#!/bin/bash

set -e

echo "=========================================="
echo "Building Rover Control Docker Image"
echo "=========================================="

# Check if src directory exists
if [ ! -d "src/rover_control" ]; then
    echo "ERROR: src/rover_control directory not found!"
    echo "Please create all the required files first."
    exit 1
fi

# Check if required files exist
required_files=(
    "dockerfile"
    "entrypoint.sh"
    "src/rover_control/package.xml"
    "src/rover_control/CMakeLists.txt"
    "src/rover_control/src/control_node_controller.cpp"
    "src/rover_control/src/control_node_joystick.cpp"
    "src/rover_control/launch/system_launch.py"
)

for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        echo "ERROR: Required file missing: $file"
        exit 1
    fi
done

echo "✓ All required files found"
echo ""

# Build the Docker image
echo "Building Docker image..."
docker build -t swinroverteam/srt-ros:latest . 

echo ""
echo "=========================================="
echo "✓ Build complete!"
echo "=========================================="
echo ""
echo "To run the system:"
echo "  docker run -it --rm --network host --privileged --device /dev/input swinroverteam/srt-ros:latest"
echo "  alternatively, run the included run.sh shell file"
echo ""
echo "To run only the micro-ROS agent:"
echo "  docker run -it --rm --network host -e LAUNCH_MODE=agent-only swinroverteam/srt-ros:latest"
echo ""
