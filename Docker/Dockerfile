FROM osrf/ros:galactic-desktop

RUN apt update && apt install tmux ros-galactic-navigation2 ros-galactic-nav2-bringup openssh-server -y
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN echo "root:root" | chpasswd

# Make ROS2 Workspace Dirss
RUN mkdir -p /home/dev_ws/code

# Reference script with commands to source workspace
COPY ./Docker/source_ros.sh /home/dev_ws/source_ros.sh
COPY ./Docker/welcome_message /root/.welcome

# Copy ROS2 packages into workspace
COPY ./Assets/maes-ros-slam-ws/ /home/dev_ws/code

# Change to workspace on sign-in
RUN echo "cd home/dev_ws/code" >> ~/.bashrc

# Build the workspace
#RUN cd home/dev_ws && . /opt/ros/galactic/setup.sh && colcon build

# Source the workspace on sign in
RUN echo ". /opt/ros/galactic/setup.sh && colcon build" >> ~/.bashrc 
RUN echo ". /home/dev_ws/code/install/local_setup.bash" >> ~/.bashrc && echo ". /home/dev_ws/source_ros.sh" >> ~/.bashrc
RUN echo "export HOME=/home/dev_ws" >> ~/.bashrc && echo "cat /root/.welcome" >> ~/.bashrc


ENTRYPOINT service ssh start && /bin/bash
