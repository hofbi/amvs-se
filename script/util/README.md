# Utilities

Utilities module for encoding and quality experiments using ffmpeg

## Fetch Raw Videos

```shell
# CIF Videos
./fetch-yuv-videos.sh
./fetch-y4m-videos.sh

# HD Videos
./fetch-y4m-hd-videos.sh
```

## Encoding Experiment Runner

All encoding experiments for the dataset are generated with `run_ffmpeg.py`

```shell
# Environment setup
docker-compose build
docker-compose run ffmpeg bash

# Usage
python3 run_ffmpeg.py -h
```

## Measure Video Quality for Application Experiment

```shell
# Usage
python3 run_application_quality.py -h
```

Provide the input config file as CSV file with the following format:

```csv
,mode,name,qp,gop_len,resolution,rate,k_size,sigma
0,multi,bus,41,1,352x288,30,1,0.5
...
```

The script will extend this config file with the metric columns `bitrate,ssim,psnr,vmaf`.

## Video Tools

Calculate spatial information (SI) and temporal information (TI) according to ITU-T P.910

* https://github.com/Telecommunication-Telemedia-Assessment/SITI (C++)
* Python package `siti`

## Converter

Convert videos to rosbags or images via `video-to-bag.sh` or `video-to-img.sh`
