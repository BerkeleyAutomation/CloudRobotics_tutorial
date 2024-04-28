#!/bin/bash

WORKDIR="$(cd $(dirname "$0") ; pwd)"

docker run -it --rm \
       -e USER_ID=`id -u $USER` \
       -e GROUP_ID=`id -g $USER` \
       -e WORKDIR="$WORKDIR" \
       --net=host \
       -v "$HOME/.aws:$HOME/.aws" \
       --cap-add=NET_ADMIN \
       --mount src="$WORKDIR",target="$WORKDIR",type=bind \
       --workdir "$PWD" \
       cloudrobotics_tutorial "$@"
