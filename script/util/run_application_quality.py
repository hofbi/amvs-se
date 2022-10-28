"""
Run ffmpeg quality measurements for the application encoding parameter
"""

import argparse
import shutil
import sys
from pathlib import Path

import pandas as pd

try:
    sys.path.append(str(Path(__file__).absolute().parent))
except IndexError:
    pass

from args import SettingsDictAction
from ffmpeg import calculate_quality_metrics
from model import EncodingParameterSet


def parse_arguments():
    """Parse CLI arguments"""

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "config",
        type=argparse.FileType("r"),
        help="Encoding parameter for which the quality should be measured",
    )
    parser.add_argument(
        "-m",
        "--mode",
        action=SettingsDictAction,
        choices=SettingsDictAction.VALUE_DICT.keys(),
        default=SettingsDictAction.VALUE_DICT["x264"],
        help="Select the encoder that should be used",
    )
    return parser.parse_args()


def create_encoding_parameter(row):
    """Create an encoding parameter from a dataframe row"""
    return EncodingParameterSet(
        name=row["name"],
        qp=row.qp,
        rate=row.rate,
        gop_len=row.gop_len,
        k_size=row.k_size,
        sigma=row.sigma,
        resolution=row.resolution,
    )


def main():
    """main"""
    args = parse_arguments()
    settings = args.mode
    tmp_dir = Path("tmp_quality")

    df_all = pd.read_csv(args.config.name)
    for k_size, sigma in df_all.groupby(["k_size", "sigma"]).groups.keys():
        df = df_all.loc[(df_all.k_size == k_size) & (df_all.sigma == sigma)]
        df["ep"] = df.apply(
            create_encoding_parameter,
            axis=1,
        )
        df[["bitrate", "psnr", "ssim", "vmaf"]] = df.apply(
            lambda row: calculate_quality_metrics(
                row.ep,
                settings,
                Path(f"{row.ep.name}{settings.extension}.yuv"),
                tmp_dir,
                str(row["mode"]),
            ),
            axis=1,
            result_type="expand",
        )
        mean_quality_by_video_and_mode = df.groupby(["name", "mode"]).mean()
        mean_quality_by_video_and_mode.to_csv(
            f"quality-{settings.preprocessor.name}-{k_size}-{str(sigma).replace('.', '')}.csv",
            float_format="%.2f",
        )
        print(mean_quality_by_video_and_mode)

        shutil.rmtree(tmp_dir)


if __name__ == "__main__":
    main()
