#!/usr/bin/env bash

echo "Download Y4M HD test videos and convert to YUV"

fetch_video() {
    name=$1
    suffix=$2
    wget https://media.xiph.org/video/derf/y4m/"$name"_1080p"$suffix".y4m
    ffmpeg -y -i "$name"_1080p"$suffix".y4m -vf format=yuv420p "$name"_1080p.yuv
}

fetch_video aspen
fetch_video blue_sky 25
fetch_video controlled_burn
fetch_video crowd_run 50
fetch_video dinner 30
fetch_video ducks_take_off 50
fetch_video factory 30
fetch_video in_to_tree 50
fetch_video life 30
fetch_video old_town_cross 50
fetch_video park_joy 50
fetch_video pedestrian_area 25
fetch_video red_kayak
fetch_video riverbed 25
fetch_video rush_field_cuts
fetch_video rush_hour 25
fetch_video snow_mnt
fetch_video speed_bag
fetch_video station2 25
fetch_video sunflower 25
fetch_video touchdown_pass
fetch_video tractor 25
fetch_video west_wind_easy
