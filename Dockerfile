FROM keplerc/cloud_robotics_tutorial:base
ARG HOME 
ARG USER 

ENV HOME=${HOME}
ENV USER=${USER}
# ENV WORKDIR="$(cd $(dirname "$0") ; pwd)"
ENV CYCLONEDDS_URI=file:///fog_ws/src/FogROS2/fogros2/configs/cyclonedds.ubuntu.2204.xml

# Install fog_rt_x, our latest work on cloud-based data collection
RUN pip install fog_x 

# Create FogROS2 worspace and build it
ENV ROS_WS=/fog_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
RUN git clone https://github.com/cloudgripper/cloudgripper-ros.git
RUN git clone https://github.com/BerkeleyAutomation/FogROS2.git
COPY ./tutorial_workspace ./tutorial_workspace



WORKDIR ${ROS_WS}/
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install 

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]