#!/usr/bin/env bash

echo "Create rosbag from video"

TMP_DIR="tmp_output"

video_to_bag() {
    name=$1
    # Ensure empty tmp dir
    mkdir -p "$TMP_DIR" && rm -rf "$TMP_DIR:?"/*
    # Extract images
    ffmpeg -s:v 352x288 -pix_fmt yuv420p -i "$name"_cif.yuv "$TMP_DIR"/out-%04d.png
    # Clone https://github.com/wpfhtl/BagFromImages to workspace src dir
    # mkdir build && cd build && cmake ..; make && mv bagfromimages ..
    rosrun bagfromimages bagfromimages "$TMP_DIR"/ .png 20 "$name".bag
}

video_to_bag akiyo
video_to_bag container
video_to_bag waterfall
video_to_bag foreman
video_to_bag hall
video_to_bag mobile
video_to_bag mother-daughter
video_to_bag paris
video_to_bag highway
video_to_bag tempete

rm -rf "$TMP_DIR"
