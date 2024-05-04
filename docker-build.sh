#!/bin/bash

cd "$(dirname "$0")"

docker build --no-cache -t cloudrobotics_tutorial . \
    --network=host --build-arg HOME="$HOME"   \
    --build-arg USER="$USER"   \
    --build-arg CLOUDGRIPPER_API_KEY="$CLOUDGRIPPER_API_KEY" \
    --build-arg AWS_ACCESS_KEY_ID="$AWS_ACCESS_KEY_ID" \
    --build-arg AWS_SECRET_ACCESS_KEY="$AWS_SECRET_ACCESS_KEY"
