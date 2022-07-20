FROM dorothy2-edge:galactic-base

ENV DEBIAN_FRONTEND=noninteractive
# major things almost always needed 
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-navigation2 \
   ros-${ROS_DISTRO}-nav2* 

# per-project installation - 

RUN apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-slam-toolbox \ 
   ros-${ROS_DISTRO}-joint-state-publisher-gui \
   ros-${ROS_DISTRO}-xacro \ 
   ros-${ROS_DISTRO}-tf2-tools 

RUN apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-rqt-graph \
   ros-${ROS_DISTRO}-robot-localization 

RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-test-msgs
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-libg2o
# RUN  apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-gazebo-ros-pkgs \
#    ros-${ROS_DISTRO}-gazebo-ros2-control 


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
