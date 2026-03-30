#!/bin/bash

# Exit on error, undefined variables, and pipe failures
set -euo pipefail

# Environment variables
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(pwd)}"
MICRO_ROS_PORT="${MICRO_ROS_PORT:-8888}"
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

source_ros_environment() {
    if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        error_exit "ROS $ROS_DISTRO setup file not found"
    fi
    if [ ! -f "$WORKSPACE_DIR/install/local_setup.bash" ]; then
        error_exit "Workspace setup file not found: $WORKSPACE_DIR/install/local_setup.bash"
    fi

    set +u
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    source "$WORKSPACE_DIR/install/local_setup.bash"
    set -u
}

# Launch the complete rover system
launch_rover_system() {
    log_step "Launching complete rover ground control system..."
    
    cd "$WORKSPACE_DIR" || error_exit "Failed to change to workspace"
    source_ros_environment
    
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
    source_ros_environment
    
    log_info "=========================================="
    log_info "Starting micro-ROS agent on UDP4 port $MICRO_ROS_PORT"
    log_info "Press Ctrl+C to stop"
    log_info "=========================================="
    
    ros2 run micro_ros_agent micro_ros_agent udp4 --port "$MICRO_ROS_PORT"
}

# Main script
main() {
    if [ $# -gt 0 ]; then
        log_info "Entrypoint: executing command override: $*"
        exec "$@"
    fi

    log_info "=========================================="
    log_info "Rover Ground Control System - Entrypoint"
    log_info "=========================================="
    log_info "ROS Distribution: $ROS_DISTRO"
    log_info "Workspace: $WORKSPACE_DIR"
    log_info "Micro-ROS Port: $MICRO_ROS_PORT"
    log_info "Launch Mode: $LAUNCH_MODE"
    log_info "=========================================="
    
    # Launch based on mode
    if [ "$LAUNCH_MODE" = "integrated" ]; then
        launch_rover_system
    else
        launch_agent_only
    fi
}

# Execute main function
main "$@"