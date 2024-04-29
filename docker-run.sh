#!/bin/bash

CURR_DIR="$(cd $(dirname "$0") ; pwd)"
ROS_WORKDIR="/fog_ws"

docker run -it --rm \
       -e USER_ID=`id -u $USER` \
       -e GROUP_ID=`id -g $USER` \
       -e WORKDIR="$ROS_WORKDIR" \
       --net=host \
       -v "$HOME/.aws:$HOME/.aws" \
       --cap-add=NET_ADMIN \
       --mount src="$CURR_DIR/tutorial_workspace",target="$ROS_WORKDIR/src/tutorial_workspace",type=bind \
       --workdir "$ROS_WORKDIR" \
       cloudrobotics_tutorial "$@"
