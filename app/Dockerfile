FROM osrf/ros:foxy-desktop
# FROM osrf/ros:humble-desktop-full
MAINTAINER Kevin DeMarco
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create the "ros" user, with the host user' IDs
ARG USER_ID=1000
ARG GROUP_ID=1000

ENV USERNAME ros
# RUN adduser --disabled-password --gecos '' $USERNAME \
#     && usermod  --uid ${USER_ID} $USERNAME \
#     && groupmod --gid ${GROUP_ID} $USERNAME \
#     && usermod --shell /bin/bash $USERNAME \
#     && adduser $USERNAME sudo \
#     && adduser $USERNAME dialout \
#     && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# USER $USERNAME

RUN mkdir -p /home/$USERNAME
RUN sudo apt-get update

# Run rosdep update, add ROS, Gazebo, and colcon setup to ros user's .bashrc
# RUN sudo apt-get update \
#     && rosdep update \
#     && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/$USERNAME/.bashrc \
#     && echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> /home/$USERNAME/.bashrc

# Create the workspace
RUN mkdir -p /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace

RUN sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-rtabmap-ros
# RUN sudo apt install -y ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-rtabmap-ros

# Copy code into workspace, run rosdep install for workspace, build, and source setup in ros user's
# COPY --chown=ros *.sh .
# COPY --chown=ros ./proxy_holder src
COPY *.sh .
COPY ./proxy_holder src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=${ROS_DISTRO} \
    # && colcon build --symlink-install \
    && sh build.sh \
    && echo 'source ~/workspace/install/local_setup.bash' >> /home/$USERNAME/.bashrc