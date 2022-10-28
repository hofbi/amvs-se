"""
Video IO module for reading yuv videos
"""


import sys
from pathlib import Path

import cv2
import numpy as np

try:
    sys.path.append(str(Path(__file__).absolute().parent))
except IndexError:
    pass

from model import Settings


class VideoCaptureYUV:
    """Read YUV raw video files"""

    def __init__(self, video: Path, settings: Settings, output_path: Path):
        self.frame_len = settings.width * settings.height * 3 // 2
        self.shape = (settings.height * 3 // 2, settings.width)
        self.f_input = video.open("rb")
        self.f_output = output_path.open("wb")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.f_input.close()
        self.f_output.close()

    def read_raw(self):
        try:
            raw = self.f_input.read(self.frame_len)
            yuv = np.frombuffer(raw, dtype=np.uint8)
            yuv = yuv.reshape(self.shape)
        except ValueError:
            return False, None
        return True, yuv

    def read(self):
        ret, yuv = self.read_raw()
        if not ret:
            return ret, yuv
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        return ret, bgr

    def write(self, img):
        yvu = cv2.cvtColor(img, cv2.COLOR_BGR2YUV_I420)
        self.f_output.write(yvu.tobytes())
