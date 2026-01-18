FROM osrf/ros:jazzy-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
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
    ros-jazzy-joy \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
ENV MICROROS_WS=/ros
WORKDIR ${MICROROS_WS}

# Copy source files
COPY src/ ${MICROROS_WS}/src/

# Copy entrypoint script
COPY entrypoint.sh /ros/entrypoint.sh
RUN chmod +x /ros/entrypoint.sh

# Expose micro-ROS agent port
EXPOSE 8888/udp

# Set entrypoint
ENTRYPOINT ["/ros/entrypoint.sh"]