#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Supported versions ['noetic']
# shellcheck disable=SC1091
ROS_VERSION='noetic'

echo "Install dependencies for $ROS_VERSION"

# Video Codec
sudo apt install -y \
    python3-pip \
    clang-tidy-10 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavresample-dev \
    build-essential \
    libx264-dev \
    libva-dev \
    ros-"$ROS_VERSION"-opencv-apps \
    nlohmann-json3-dev \
    ffmpeg \
    p7zip-full

pip3 install -r "${SCRIPT_DIR}"/../requirements.txt
