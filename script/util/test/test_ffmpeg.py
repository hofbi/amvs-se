"""FFMPEG Wrapper Test"""

import unittest
import sys
from pathlib import Path

try:
    sys.path.append(str(Path(__file__).parents[1]))
except IndexError:
    pass

from ffmpeg import parse_metric_score


class FfmpegTest(unittest.TestCase):
    """FFMPEG Wrapper Test"""

    def test_parse_metric_score__empty_content__should_be_0(self):
        result = parse_metric_score("", "ssim")
        self.assertEqual(0, result)

    def test_parse_metric_score__ssim_content__should_be_correct(self):
        content = (
            "[Parsed_ssim_0 @ 0x55a9ff3a2580] SSIM Y:0.984821 (18.187532) "
            "U:0.989577 (19.820120) V:0.988313 (19.323065) All:0.986196 (18.599848)"
        )
        result = parse_metric_score(content, "ssim")
        self.assertEqual(0.986196, result)

    def test_parse_metric_score__psnr_content__should_be_correct(self):
        content = (
            "[Parsed_psnr_0 @ 0x5560d78c3500] PSNR y:44.154534 u:45.880091 "
            "v:46.748720 average:44.757351 min:43.326964 max:45.577975"
        )
        result = parse_metric_score(content, "psnr")
        self.assertEqual(44.757351, result)


if __name__ == "__main__":
    unittest.main()
