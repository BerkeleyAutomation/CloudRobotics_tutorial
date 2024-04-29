#!/bin/bash

cd "$(dirname "$0")"

docker build -t cloudrobotics_tutorial --build-arg HOME="$HOME" --build-arg USER="$USER" . --network=host
