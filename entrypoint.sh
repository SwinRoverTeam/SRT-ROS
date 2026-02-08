#!/bin/bash

# Exit on error, undefined variables, and pipe failures
set -euo pipefail

# Environment variables
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
MICRO_ROS_BRANCH="${MICRO_ROS_BRANCH:-$ROS_DISTRO}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(pwd)}"
MICRO_ROS_PORT="${MICRO_ROS_PORT:-8888}"
PARALLEL_JOBS="${PARALLEL_JOBS:-2}"
LAUNCH_MODE="${LAUNCH_MODE:-integrated}"


# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Logging functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" >&2
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# Error handler
error_exit() {
    log_error "$1"
    exit 1
}

# Cleanup function
cleanup() {
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        log_error "Script failed with exit code:  $exit_code"
    fi
}

trap cleanup EXIT

# Check if package exists
check_ros_package() {
    local package_name="$1"
    local pkg_list
    
    pkg_list=$(ros2 pkg list 2>/dev/null || true)
    
    if echo "$pkg_list" | grep -q "^${package_name}$"; then
        return 0
    else
        return 1
    fi
}

# Setup micro-ROS agent
setup_micro_ros_agent() {
    log_step "Setting up micro-ROS agent..."
    
    cd "$WORKSPACE_DIR" || error_exit "Failed to change to workspace directory: $WORKSPACE_DIR"

    # Clone micro-ROS setup repository if needed
    if [ ! -d "src/micro_ros_setup" ]; then
        log_info "Cloning micro_ros_setup repository (branch: $MICRO_ROS_BRANCH)..."
        mkdir -p src
        if ! git clone -b "$MICRO_ROS_BRANCH" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup; then
            error_exit "Failed to clone micro_ros_setup repository"
        fi
        log_info "Repository cloned successfully"
    else
        log_info "micro_ros_setup repository already exists"
    fi

    # Source ROS setup
    log_info "Sourcing ROS $ROS_DISTRO setup..."
    if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        error_exit "ROS $ROS_DISTRO setup file not found"
    fi
    # Remove flag check for unbound variables
    set +u
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    # Re-enable flag check for unbound variables
    set -u

    # Update rosdep
    log_info "Updating rosdep..."
    rosdep update 2>&1 | grep -v "Warning" || true

    # Install dependencies
    log_info "Installing dependencies from workspace..."
    rosdep install --from-paths src --ignore-src -y 2>&1 || true

    # Build workspace if not already built
    if [ ! -f "install/local_setup.bash" ]; then
        log_info "Building workspace with $PARALLEL_JOBS parallel workers..."
        if ! colcon build --parallel-workers "$PARALLEL_JOBS" --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1; then
            error_exit "Workspace build failed."
        fi
        # Build custom package
        log_info "Building rover_control package"
        if ! colcon build --parallel-workers "$PARALLEL_JOBS" --packages-select rover_control 2>&1; then
            error_exit "rover_control package build failed."
        fi
        log_info "Workspace built successfully"
    else
        log_info "Workspace already built"
    fi

    # Source workspace
    log_info "Sourcing workspace setup..."
    if [ ! -f "install/local_setup.bash" ]; then
        error_exit "Workspace setup file not found after build"
    fi
    set +u
    source install/local_setup.bash
    set -u
    log_info "Workspace sourced successfully"

    # Create micro-ROS agent workspace if needed
    if [ ! -d "src/uros" ]; then
        log_info "Creating micro-ROS agent workspace..."
        set +u
        ros2 run micro_ros_setup create_agent_ws.sh 2>&1 || error_exit "Failed to create agent workspace"
        
        log_info "Building micro-ROS agent..."
        ros2 run micro_ros_setup build_agent.sh 2>&1 || error_exit "Failed to build agent"
        
        # Re-source after build
        log_info "Re-sourcing workspace after agent build..."
        source install/local_setup.bash
        set -u
    else
        log_info "Micro-ROS agent already built"
    fi

    # Verify installation
    if check_ros_package "micro_ros_agent"; then
        log_info "✓ micro_ros_agent package verified"
    else
        error_exit "micro_ros_agent package not found"
    fi
}

# Launch the complete rover system
launch_rover_system() {
    log_step "Launching complete rover ground control system..."
    
    cd "$WORKSPACE_DIR" || error_exit "Failed to change to workspace"
    set +u
    source install/local_setup.bash
    set -u
    
    # Check if control node package exists
    if ! check_ros_package "rover_control"; then
        log_error "rover_control package not found!"
        log_info "Available packages:"
        ros2 pkg list | grep -E "(joy|micro_ros|rover)" || true
        error_exit "rover_control package must be built first"
    fi
    
    # Check if joy package exists
    if ! check_ros_package "joy"; then
        log_warn "joy package not found! Installing..."
        apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy
    fi
    
    log_info "=========================================="
    log_info "Starting Rover Ground Control System"
    log_info "  - Joy Node (controller input)"
    log_info "  - Control Node (command processing)"
    log_info "  - Micro-ROS Agent (UDP port $MICRO_ROS_PORT)"
    log_info "=========================================="
    log_info "Press Ctrl+C to stop all nodes"
    log_info "=========================================="
    
    # Launch the system using the launch file
    ros2 launch rover_control system_launch.py
}

# Launch only the micro-ROS agent
launch_agent_only() {
    log_step "Launching micro-ROS agent only..."
    
    cd "$WORKSPACE_DIR" || error_exit "Failed to change to workspace"
    source install/local_setup.bash
    
    log_info "=========================================="
    log_info "Starting micro-ROS agent on UDP4 port $MICRO_ROS_PORT"
    log_info "Press Ctrl+C to stop"
    log_info "=========================================="
    
    ros2 run micro_ros_agent micro_ros_agent udp4 --port "$MICRO_ROS_PORT"
}

# Main script
main() {
    log_info "=========================================="
    log_info "Rover Ground Control System - Entrypoint"
    log_info "=========================================="
    log_info "ROS Distribution: $ROS_DISTRO"
    log_info "Workspace: $WORKSPACE_DIR"
    log_info "Micro-ROS Port: $MICRO_ROS_PORT"
    log_info "Launch Mode: $LAUNCH_MODE"
    log_info "=========================================="
    
    # Setup micro-ROS agent
    setup_micro_ros_agent
    
    # Update bashrc for persistence
    if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc 2>/dev/null; then
        log_info "Adding ROS setup to ~/.bashrc..."
        echo "" >> ~/.bashrc
        echo "# ROS setup - added $(date '+%Y-%m-%d')" >> ~/.bashrc
        echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    fi
    
    if ! grep -q "source $WORKSPACE_DIR/install/local_setup.bash" ~/.bashrc 2>/dev/null; then
        log_info "Adding workspace setup to ~/.bashrc..."
        echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc
    fi
    
    # Launch based on mode
    if [ "$LAUNCH_MODE" = "integrated" ]; then
        launch_rover_system
    else
        launch_agent_only
    fi
}

# Execute main function
main

# Execute additional commands if provided
if [ $# -gt 0 ]; then
    log_info "Entrypoint:  executing $*"
    exec "$@"
fi