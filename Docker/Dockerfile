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
