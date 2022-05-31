# Copyright 2022 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http://www.gnu.org/licenses/.
# 
# Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
# 
# Original repository: https://github.com/MalteZA/MAES


FROM ros:galactic

RUN apt update && apt install tmux ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-rviz2 openssh-server vim nano sudo -y
RUN apt update && apt upgrade -y

# Create a maes-user that shares UID/GID with default ubuntu-user.
# If the UID does not match the user from the host system, the Unity Editor will hang indefinitely
# after the first time the ROS-project has been built.
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID maes-group && useradd -m -s /bin/bash -g $GID -G root,sudo -u $UID maes-user
RUN echo "root:root\nmaes-user:maes" | chpasswd

# Make ROS2 Workspace Dirs
RUN mkdir -p /home/maes-user/code && chown -R maes-user:maes-group /home/maes-user/code

# Reference script with commands to source workspace, and a welcome-message
COPY ./Docker/source_ros.sh /home/maes-user/source_ros.sh
COPY ./Docker/welcome_message /home/maes-user/.welcome

# Change to workspace on sign-in
RUN echo "cd /home/maes-user/code" >> /home/maes-user/.bashrc

# Add commands to be executed on tty log-in
## Source the workspace on sign in and colcon-build the first time
RUN echo ". /opt/ros/galactic/setup.sh && [ ! -f /home/maes-user/.firstbuild ] && colcon build" >> /home/maes-user/.bashrc 
RUN echo ". /home/maes-user/code/install/local_setup.bash" >> /home/maes-user/.bashrc && echo ". /home/maes-user/source_ros.sh" >> /home/maes-user/.bashrc
## Display a welcome-message on the first log-in, and mark the first log-in has happened
RUN echo "[ ! -f /home/maes-user/.firstbuild ] && cat /home/maes-user/.welcome && touch /home/maes-user/.firstbuild" >> /home/maes-user/.bashrc


ENTRYPOINT service ssh start && su maes-user && exit
