#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Build the Docker image from the Dockerfile in the current directory
docker build -t starq:boom -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"

# Start the Docker container with the current directory mounted to /starq_ws, and automatically remove the container when it is stopped or exited
docker run --rm -it --net host --mount type=bind,source="${SCRIPT_DIR}/../",target=/home/pi/ros2_ws/src/boom_packages starq:boom