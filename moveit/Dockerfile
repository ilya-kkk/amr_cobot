FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install MoveIt2 + build tooling
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-servo \
    && rm -rf /var/lib/apt/lists/*

# rosdep (safe if already initialized)
RUN rosdep init || true
RUN rosdep update

# Workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy robot description + MoveIt config package
COPY AMR_COBOT_urdf /ros2_ws/src/amr_cobot
COPY moveit/amr_cobot_moveit_config /ros2_ws/src/amr_cobot_moveit_config

# Install package deps and build
RUN rosdep install --from-paths src --ignore-src -r -y || true
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Default runtime
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch amr_cobot_moveit_config move_group.launch.py"]
