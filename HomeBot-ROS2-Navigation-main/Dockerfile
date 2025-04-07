FROM ros:humble

# Install base dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy package files first for caching
COPY src/robot_simulation/package.xml src/robot_simulation/
COPY src/robot_simulation/CMakeLists.txt src/robot_simulation/

# Install dependencies (skip unresolved ones)
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -y --skip-keys="librealsense2,robot_description" || true

# Copy all source files
COPY src/ ./src/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Environment setup
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/entrypoint.sh"]