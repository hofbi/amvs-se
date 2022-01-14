#!/usr/bin/env bash

echo "Download Y4M CIF test videos and convert to YUV"

fetch_video() {
    name=$1
    wget https://media.xiph.org/video/derf/y4m/"$name"_cif.y4m
    ffmpeg -y -i "$name"_cif.y4m "$name"_cif.yuv
}

fetch_video bowing
fetch_video city
fetch_video crew
fetch_video deadline
fetch_video football
fetch_video harbour
fetch_video husky
fetch_video ice
fetch_video pamphlet
fetch_video sign_irene
fetch_video soccer
fetch_video students
