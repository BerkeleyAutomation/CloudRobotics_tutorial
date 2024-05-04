FROM keplerc/cloud_robotics_tutorial:base
ARG HOME 
ARG USER 
ARG CLOUDGRIPPER_API_KEY 
ARG AWS_ACCESS_KEY_ID
ARG AWS_SECRET_ACCESS_KEY

ENV HOME=${HOME}
ENV USER=${USER}

# ENV WORKDIR="$(cd $(dirname "$0") ; pwd)"
# ENV CYCLONEDDS_URI=file:///fog_ws/src/FogROS2/fogros2/configs/cyclonedds.ubuntu.2204.xml


# Install fog_rt_x, our latest work on cloud-based data collection
RUN pip install fog_x 

# Create FogROS2 worspace and build it
ENV ROS_WS=/fog_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
RUN git clone https://github.com/cloudgripper/cloudgripper-ros.git
RUN git clone https://github.com/KeplerC/FogROS2.git
COPY ./tutorial_workspace ./tutorial_workspace



WORKDIR ${ROS_WS}/
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install 

ENV CLOUDGRIPPER_API_KEY=${CLOUDGRIPPER_API_KEY}
ENV AWS_ACCESS_KEY_ID=${AWS_ACCESS_KEY_ID}
ENV AWS_SECRET_ACCESS_KEY=${AWS_SECRET_ACCESS_KEY}

# write aws credentials 
RUN mkdir -p ${HOME}/.aws
RUN echo "[default]" > ${HOME}/.aws/credentials
RUN echo "aws_access_key_id = ${AWS_ACCESS_KEY_ID}" >> ${HOME}/.aws/credentials
RUN echo "aws_secret_access_key = ${AWS_SECRET_ACCESS_KEY}" >> ${HOME}/.aws/credentials


ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]