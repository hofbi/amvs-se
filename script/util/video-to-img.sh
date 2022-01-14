#!/usr/bin/env bash

ffmpeg -i front_1080p.mp4 -r 1/1 export/front%03d.bmp
ffmpeg -i front_left_1080p.mp4 -r 1/1 export/front_left%03d.bmp
ffmpeg -i front_right_1080p.mp4 -r 1/1 export/front_right%03d.bmp
ffmpeg -i rear_1080p.mp4 -r 1/1 export/rear%03d.bmp
ffmpeg -i rear_left_1080p.mp4 -r 1/1 export/rear_left%03d.bmp
ffmpeg -i rear_right_1080p.mp4 -r 1/1 export/rear_right%03d.bmp
