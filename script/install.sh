#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ "$(
    . /etc/os-release
    echo "$VERSION_ID"
)" = "20.04" ]; then
    ROS_VERSION='noetic'
    PYTHON_SUFFIX=3
else
    PYTHON_SUFFIX=""
    ROS_VERSION='melodic'
fi

echo "Install dependencies for $ROS_VERSION"

# Video Codec
sudo apt install -y \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    ffmpeg \
    build-essential \
    libx264-dev \
    libva-dev \
    nlohmann-json"$PYTHON_SUFFIX"-dev

pip3 install -r ${SCRIPT_DIR}/../requirements.txt
