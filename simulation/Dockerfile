FROM althack/ros2:galactic-dev 

ENV DEBIAN_FRONTEND=noninteractive
# major things almost always needed 
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-navigation2 \
   ros-${ROS_DISTRO}-nav2* \
   ros-${ROS_DISTRO}-gazebo-ros-pkgs \
   ros-${ROS_DISTRO}-gazebo-ros2-control 

# per-project installation - 

RUN apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-joint-state-publisher-gui \
   ros-${ROS_DISTRO}-xacro \ 
   ros-${ROS_DISTRO}-tf2-tools 
 
# WORKDIR /tmp/
RUN cd /tmp/ && \ 
   git clone https://github.com/SwiftGust/ardupilot_gazebo && \
   cd ardupilot_gazebo && \
   mkdir build && \
   cd build && \
   cmake .. && \
   make -j4 && \
   make install && \
   cd /tmp && \
   sudo rm -rf ardupilot_gazebo
   
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-rosidl-typesupport-* 

# cleaning
RUN apt-get update \
   # && apt-get -y install --no-install-recommends
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# actions to be use in ros user and to start with ros user
ENV USERNAME=ros
USER $USERNAME

# Set up auto-source of workspace for ros user
ARG WORKSPACE="/home/${USERNAME}/dev_ws"
RUN mkdir $WORKSPACE
RUN chown ros:ros $WORKSPACE
COPY ./scripts/setup_docker_env.sh /tmp
RUN /tmp/setup_docker_env.sh
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

ENV DEBIAN_FRONTEND=dialog
