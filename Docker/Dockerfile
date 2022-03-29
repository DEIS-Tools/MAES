FROM osrf/ros:galactic-desktop

RUN apt update && apt install tmux ros-galactic-navigation2 ros-galactic-nav2-bringup -y

# Make ROS2 Workspace Dirss
RUN mkdir -p /home/dev_ws/src

# Reference script with commands to source workspace
COPY ./Docker/source_ros.sh /home/dev_ws/source_ros.sh

# Copy ROS2 packages into workspace
COPY ./Assets/maes_ros2_ws/ /home/dev_ws/src

# Change to workspace on sign in
RUN echo "cd home/dev_ws" >> ~/.bashrc

# Build the workspace
RUN cd home/dev_ws && . /opt/ros/galactic/setup.sh && colcon build

# Source the workspace on sign in
RUN echo ". /home/dev_ws/install/local_setup.bash" >> ~/.bashrc && echo ". /home/dev_ws/source_ros.sh" >> ~/.bashrc

ENTRYPOINT /bin/bash
