"""
Run ffmpeg encodings with gaussian preprocessing filter for a given video name
"""

import sys
from pathlib import Path
import argparse
import itertools
import shutil
import pandas as pd
from tqdm import tqdm
from dataclasses import asdict

try:
    sys.path.append(str(Path(__file__).absolute().parent))
except IndexError:
    pass

from model import EncodingParameterSet, CombinedStats
from ffmpeg import create_video_config, encode, preprocess_videos
from args import SettingsDictAction


def parse_arguments():
    """Parse CLI arguments"""

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "video",
        type=argparse.FileType("r"),
        help="Video file to encode",
    )
    parser.add_argument(
        "-m",
        "--mode",
        action=SettingsDictAction,
        choices=SettingsDictAction.VALUE_DICT.keys(),
        default=SettingsDictAction.VALUE_DICT["x264"],
        help="Select the mode that should be used",
    )
    return parser.parse_args()


def main():
    """main"""
    args = parse_arguments()
    settings = args.mode
    encoding_parameters = settings.parameter

    video_config = create_video_config(Path(args.video.name), settings)
    tmp_dir = Path(f"tmp_{video_config['name']}")

    filter_params = list(
        itertools.product(*encoding_parameters.to_filter_list(video_config))
    )
    filter_params_sets = [EncodingParameterSet(*param) for param in filter_params]
    preprocess_videos(filter_params_sets, settings, video_config["path"], tmp_dir)

    encoding_params = list(
        itertools.product(*encoding_parameters.to_list(video_config))
    )
    encoding_params_sets = [EncodingParameterSet(*param) for param in encoding_params]

    pbar = tqdm(encoding_params_sets, desc=video_config["name"])
    video_stats = [encode(param, settings, video_config, tmp_dir) for param in pbar]
    combined_stats = [
        CombinedStats(*asdict(enc).values(), *asdict(stat).values())
        for enc, stat in zip(encoding_params_sets, video_stats)
    ]

    df = pd.DataFrame(data=combined_stats)
    df.to_csv(f"ffmpeg-results-{video_config['name']}.csv")

    shutil.rmtree(tmp_dir)


if __name__ == "__main__":
    main()
