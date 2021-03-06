# Single Encoder

[![Actions Status](https://github.com/hofbi/amvs-se/workflows/CI/badge.svg)](https://github.com/hofbi/amvs-se)
[![Actions Status](https://github.com/hofbi/amvs-se/workflows/CodeQL/badge.svg)](https://github.com/hofbi/amvs-se)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Reference implementation for the experiments of Adaptive Multi-View Live Video Streaming for Teledriving Using a Single Hardware Encoder.

## Paper

If you use this software please cite our paper.

*Adaptive Multi-View Live Video Streaming for Teledriving Using a Single Hardware Encoder, Markus Hofbauer, Christopher B. Kuhn, Goran Petrovic, Eckehard Steinbach; ISM 2020* [PDF](https://www.researchgate.net/publication/345241096_Adaptive_Multi-View_Live_Video_Streaming_for_Teledriving_Using_a_Single_Hardware_Encoder)

```tex
@inproceedings{hofbauer_2020,
    title = {Adaptive Multi-View Live Video Streaming for Teledriving Using a Single Hardware Encoder},
    booktitle = {22nd IEEE International Symposium on Multimedia (ISM)},
    publisher = {IEEE},
    author = {Hofbauer, Markus and Kuhn, Christopher B. and Petrovic, Goran and Steinbach, Eckehard},
    address = {Naples, Italy}
    month = dec,
    year = {2020},
    pages = {1--8},
}
```

## Setup

This so far has been tested on

| OS  | ROS Version |
| --- | ----------- |
| Ubuntu 18.04 | Melodic |
| Ubuntu 20.04 | Noetic |

1. Install [ROS](http://wiki.ros.org/ROS/Installation) and [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-catkin-tools)
1. Create a workspace with e.g. `mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`
1. Clone this repository into the workspace's `src` folder with `git clone https://github.com/hofbi/amvs-se.git`
1. Run the install script: `./script/install.sh`
1. Build the workspace: `catkin build`
1. Source your workspace `source ~/catkin_ws_teleop/devel/setup.<your_shell>`

## Run

This node is the first prototype implementation to verify the theoretical concept by encoding the data with different filters at Constant Quantization Parameter (CQP).

```shell
roslaunch single_encoder single_encoder.launch
```

subscribes to the provided input topics. They will be encoded individually and combined into a single superframe. Before the encoding step different filters will be applied depending on the provided settings. The output of the node is a json file containing statistics from the encoding process. The structure is as follows:

```json
    {
        "bitrate": 1000,
        "chroma_drop": false,
        "encoderId": 0,
        "encoding_time": [],
        "filter": "none-1",
        "frame_count": [],
        "frame_size": [],
        "height": 480,
        "mdvqm_scores": [],
        "psnr_scores": [],
        "skip_frames": 0,
        "spatial_activities": [],
        "stvqm_scores": [],
        "temporal_activities": [],
        "timestamp": [],
        "topic": "/carla/ego_vehicle/camera/rgb/front/image_color",
        "width": 640
    }
```

### Settings

```shell
image_params:           # List of input topics (mux will place them next to each other)
  - topic: "/carla/ego_vehicle/camera/rgb/front/image_color"
    bitrate: 1000       # Target bitrate if in CBR mode ON
    skip_frames: 0      # Number of frames that will be skipped by temporal filter
    width: 640          # Width that will be used by spatial filter
    height: 480         # Height that will be used by spatial filter
    chroma_drop: false  # Only keep the luminance channel and drop both chrominance channels
    filter:             # Low pass filter applied to the image
      type: "none"      # Filter type (supports <none, median, gaussian, blur>)
      ksize: 1          # Window size of the filter
      sigma: 0.0        # Standard deviation (for Gaussian Filter only)
  - ...
```

## Evaluation

```shell
roslaunch single_encoder single_encoder_validate.launch
```

runs the single encoder launch file and plays a specified rosbag once.

### Plot Statistics

This script is used to visualize the statistics produced from the node. Usage: `python3 plot_statistics.py -h`

### Evaluation Runner

The evaluation runner parses all settings files in a provided folder and runs the evaluation launch file with each of the settings and plots the statistics afterwards. The used json file and the figure saved as png will be stored into the provided folder. Usage: `python3 evaluation_runner.py -h`

### Plot Evaluation

Parse all json files in the provided folder, redraw the plots and save the output again as png. Further it calculates the `average.csv` file for all provided config files. This is useful if something at the plot script change but the data are still the same or to compare different filter types. Finally plots of the average data will be created based on the provided filter. Usage: `python3 plot_evaluation.py -h`

## Development

To install the additional tools required for the development, call

```shell
python3 -m pip install -r requirements.txt
sudo apt install -y clang-format
sudo snap install shfmt
```

from the source of this directory. Then, you can call

```shell
# Fomrat the code
make format

# Check format
make check_format

# Pylint
make pylint
```
