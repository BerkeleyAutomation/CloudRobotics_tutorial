FROM keplerc/cloud_robotics_tutorial:base
ARG HOME 
ARG USER 

ENV HOME=${HOME}
ENV USER=${USER}
# ENV WORKDIR="$(cd $(dirname "$0") ; pwd)"

# Create FogROS2 worspace and build it
ENV ROS_WS=${HOME}/fog_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
RUN git clone https://github.com/BerkeleyAutomation/FogROS2.git
RUN git clone https://github.com/cloudgripper/cloudgripper-ros.git


WORKDIR ${ROS_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
      colcon build --cmake-clean-cache

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]