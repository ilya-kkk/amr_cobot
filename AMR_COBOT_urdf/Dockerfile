FROM osrf/ros:humble-desktop-full

# Установка необходимых зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-joy \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-setup-assistant \
    ros-humble-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

# Инициализация rosdep
RUN rosdep init || true
RUN rosdep update

# Создание ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Копирование пакетов в workspace
COPY AMR_COBOT_urdf /ros2_ws/src/amr_cobot
COPY cnc_urdf /ros2_ws/src/cnc_urdf

# Установка зависимостей пакета
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Сборка workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Добавление setup.bash в .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Копирование entrypoint скрипта
COPY AMR_COBOT_urdf/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

WORKDIR /ros2_ws
