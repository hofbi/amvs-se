#!/usr/bin/env bash

echo "Download YUV CIF test videos"

fetch_video() {
    folder=$1
    name=$2
    name=${name:="$folder"}
    wget http://trace.eas.asu.edu/yuv/"$folder"/"$name"_cif.7z
    7z e "$name"_cif.7z
    rm "$name"_cif.7z
}

fetch_video akiyo
fetch_video bridge-close
fetch_video bridge-far
fetch_video bus
fetch_video coastguard
fetch_video container
fetch_video flower
fetch_video foreman
fetch_video hall_monitor hall
fetch_video highway
fetch_video mobile
fetch_video mother-daughter
fetch_video news
fetch_video paris
fetch_video silent
fetch_video stefan
fetch_video tempete
fetch_video waterfall
