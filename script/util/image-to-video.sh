#!/usr/bin/env bash

echo "Combining png images to yuv video"

INPUT_DIR=$1

create_video() {
    path=$1
    name=$2
    ffmpeg -i "$path/$name"_%06d.png -pix_fmt yuv420p "$path/$name".yuv
}

create_video "$INPUT_DIR" "front"
create_video "$INPUT_DIR" "front_left"
create_video "$INPUT_DIR" "front_right"
create_video "$INPUT_DIR" "rear"
create_video "$INPUT_DIR" "rear_left"
create_video "$INPUT_DIR" "rear_right"
