# Get base image from osrf/ros
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon* \
    python3-colcon-common-extensions \
    python3-vcstool\
    build-essential \
    iputils-ping \
    iproute2 \
    net-tools \
    dnsutils \
    curl \
    git \
    ca-certificates \
    cmake \
    python3-pip \
    python3-rosdep \
    clang-tidy \
    flex \
    bison \
    libncurses-dev \
    libcurl4-openssl-dev\
    usbutils \
    ros-jazzy-joy \
    joystick \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
ENV MICROROS_WS=/ros
ENV ROS_DISTRO=jazzy
ENV MICRO_ROS_BRANCH=${ROS_DISTRO}
WORKDIR ${MICROROS_WS}

# Copy source files
COPY src/ ${MICROROS_WS}/src/

# Perform one-time workspace setup at build time to keep container startup fast.
RUN set -eo pipefail \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && mkdir -p src \
    && if [ ! -d src/micro_ros_setup ]; then git clone -b "${MICRO_ROS_BRANCH}" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup; fi \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/local_setup.bash \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh

# Copy entrypoint script
COPY entrypoint.sh /ros/entrypoint.sh
RUN chmod +x /ros/entrypoint.sh

# Expose micro-ROS agent port
EXPOSE 8888/udp

# Set entrypoint
ENTRYPOINT ["/ros/entrypoint.sh"]