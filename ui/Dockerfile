FROM althack/ros2:galactic-dev 

ENV DEBIAN_FRONTEND=noninteractive
# ENV QT_DEBUG_PLUGINS=1
# major things almost always needed 
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-rqt* 

# RUN sudo apt-get install -y '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev 

RUN pip3 install opencv-python numpy
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-cv-bridge \ 
   ros-${ROS_DISTRO}-xacro \ 
   ros-${ROS_DISTRO}-tf2-tools \ 
   ros-${ROS_DISTRO}-teleop-twist-keyboard \
   ros-${ROS_DISTRO}-rqt-robot-steering

RUN apt-get update \
   && sudo apt-get -y install --no-install-recommends --fix-missing ros-${ROS_DISTRO}-rviz2   
   

RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-navigation2 
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-joy
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-teleop-twist-joy
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-test-msgs
   
RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-slam-toolbox
# RUN sudo apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-domain-bridge

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
# COPY ./scripts/setup_docker_env.sh /tmp
# RUN /tmp/setup_docker_env.sh
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

ENV DEBIAN_FRONTEND=dialog
