"""
Args helper for the ffmpeg experiments
"""

import sys
from pathlib import Path
import argparse

try:
    sys.path.append(str(Path(__file__).absolute().parent))
except IndexError:
    pass

from model import Settings, EncodingParameters


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
    }

    def __call__(self, arg_parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, self.VALUE_DICT.get(values))
