"""
Args helper for the ffmpeg experiments
"""

import argparse
import sys
from pathlib import Path

try:
    sys.path.append(str(Path(__file__).absolute().parent))
except IndexError:
    pass

from model import (
    ColorQuantizationAdapter,
    EncodingParameters,
    GaussianPreprocessor,
    JpegAdapter,
    MedianPreprocessor,
    NonePreprocessor,
    Settings,
)


class SettingsDictAction(argparse.Action):
    """Handles the settings dict with argparse"""

    VALUE_DICT = {
        "x264": Settings(),
        "nhevc": Settings(
            width=1920,
            height=1080,
            extension="_1080p",
            parameter=EncodingParameters(
                resolution=[
                    "1920x1080",
                    "1600x900",
                    "1280x720",
                ],
                qp=[24, 29, 34, 40, 46],
                gop_l=[1, 3, 6, 10, 20, 40, 120],
                rate=[30, 15, 3],
            ),
            ffmpeg_config=lambda params: f"hevc_nvenc -g {params.gop_len} -qp {params.qp}",
        ),
        "x264-none": Settings(
            parameter=EncodingParameters.create_bd_parameters(k_size=[1]),
            preprocessor=NonePreprocessor(),
        ),
        "x264-gauss": Settings(
            parameter=EncodingParameters.create_bd_parameters(
                k_size=[3, 5], sigma=[0.5, 0.6, 0.7, 0.8, 1.0, 1.5]
            ),
            preprocessor=GaussianPreprocessor(),
        ),
        "x264-color": Settings(
            parameter=EncodingParameters.create_bd_parameters(k_size=[8, 16]),
            preprocessor=ColorQuantizationAdapter(),
        ),
        "x264-jpeg": Settings(
            parameter=EncodingParameters.create_bd_parameters(k_size=[10, 20, 40, 60]),
            preprocessor=JpegAdapter(),
        ),
        "x264-median": Settings(
            parameter=EncodingParameters.create_bd_parameters(k_size=[3, 5, 7, 9]),
            preprocessor=MedianPreprocessor(),
        ),
    }

    def __call__(self, arg_parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, self.VALUE_DICT.get(values))
