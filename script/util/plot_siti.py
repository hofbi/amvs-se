"""
Plot siti over time for all videos
"""

import argparse
from pathlib import Path
from subprocess import check_output
import json
import sys
import matplotlib.pyplot as plt


WIDTH = 352
HEIGHT = 288


def parse_arguments():
    """parse arguments"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input_dir",
        type=str,
        help="Path to the video files",
    )
    return parser.parse_args()


def call_output(command):
    """Run subprocess in shell and return parsed output"""
    return check_output(command, shell=True).decode(sys.stdout.encoding).strip()


def run_siti(video_path):
    """Run siti on video an return results as json"""
    siti_output = call_output(
        f"siti {video_path} --width {WIDTH} --height {HEIGHT} -of json"
    )
    return json.loads(siti_output)


def main():
    """main"""
    args = parse_arguments()

    yuv_files = list(Path(args.input_dir).glob("*.yuv"))

    sitis = [run_siti(video) for video in yuv_files]

    ax1 = plt.subplot(211)
    plt.ylabel("SI")
    ax2 = plt.subplot(212)
    plt.ylabel("TI")
    for siti in sitis:
        name = Path(siti["input_file"]).stem
        ax1.plot(siti["si"], label=name)
        ax2.plot(siti["ti"], label=name)
    ax1.legend()
    ax2.legend()
    plt.xlabel("Frame Number")

    plt.savefig("siti.png")


if __name__ == "__main__":
    main()
