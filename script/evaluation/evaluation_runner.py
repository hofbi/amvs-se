"""Run ROS evaluation on different encoder configurations"""

import argparse
import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from plot_evaluation import exit_with_error
from plot_statistics import clear_plot_data, plot_data
from tqdm import tqdm


@dataclass
class EvaluationParam:
    """Evaluation parameter"""

    settings_path: Path
    bag_path: Path
    qp: int


def configure_launch_command(settings_path, bag_path, cbr_mode, qp, pb_rate):
    """Configure the roslaunch command with arguments"""
    return (
        f"roslaunch single_encoder single_encoder_validate.launch "
        f"settings_path:={settings_path} "
        f"bag_path:={bag_path} "
        f"cbr_mode:={cbr_mode} "
        f"qp:={qp} "
        f"pb_rate:={pb_rate}"
    )


def run_single_encoder(roslaunch_cmd):
    """Run a single roslaunch command as subprocess"""
    start = time.time()
    subprocess.call(roslaunch_cmd, shell=True)
    end = time.time()
    time.sleep(1)
    return end - start


def add_param_to_stats_name(stats_file_name: Path, bag_name, qp):
    """Add encoding parameter to filename"""
    return stats_file_name.with_name(
        f"{bag_name}_{stats_file_name.stem}_{qp}{stats_file_name.suffix}"
    )


def get_settings_paths(eval_dir):
    """Return all settings yml files in eval dir"""
    settings_paths = []
    eval_path = Path(eval_dir)

    if eval_path.is_file():
        settings_paths.append(eval_path)
    elif eval_path.is_dir():
        for root, _, files in os.walk(eval_dir):
            for settings_file in files:
                if settings_file.endswith(".yml"):
                    settings_paths.append(Path(root).joinpath(settings_file))
    else:
        exit_with_error("Evaluation path does not exist!")

    settings_paths.sort()
    return settings_paths


def get_evaluation_params(bags, settings_paths, qps):
    """Permutate all bags with all settings and QPs"""
    evaluation_params = []
    for bag in bags:
        for settings_path in settings_paths:
            for qp in qps:
                evaluation_params.append(
                    EvaluationParam(settings_path, Path(bag.name), qp)
                )
    return evaluation_params


def parse_arguments():
    """Parse CLI arguments"""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "bags", type=argparse.FileType("r"), nargs="+", help="Path to the input bags"
    )
    parser.add_argument(
        "--stats",
        type=str,
        default=Path.home().joinpath(".ros").joinpath("stats.json"),
        help="Path to the statistics file",
    )
    parser.add_argument(
        "-e",
        "--eval_path",
        type=str,
        default=Path(__file__)
        .parent.parent.parent.joinpath("config")
        .joinpath("evaluation"),
        help="Path to the evaluation config directory or file",
    )
    parser.add_argument(
        "--qps",
        type=int,
        nargs="+",
        default=[25, 27, 32, 37],
        help="Quantization parameters for non-cbr mode",
    )
    parser.add_argument(
        "--cbr", action="store_true", help="Use constant bitrate control mode"
    )
    parser.add_argument(
        "--pb_rate", type=float, default=0.5, help="Playback speed for the rosbag"
    )
    return parser.parse_args()


def main():
    """main"""
    args = parse_arguments()

    stats_path = Path(args.stats)
    settings_paths = get_settings_paths(args.eval_path)
    qps = [25] if args.cbr else args.qps
    evaluation_params = get_evaluation_params(args.bags, settings_paths, qps)

    pbar = tqdm(evaluation_params)
    for param in pbar:
        launch_cmd = configure_launch_command(
            param.settings_path, param.bag_path, args.cbr, param.qp, args.pb_rate
        )
        runtime = run_single_encoder(launch_cmd)
        out_path = add_param_to_stats_name(
            param.settings_path, param.bag_path.stem, param.qp
        )
        plot_data(stats_path, out_path)
        clear_plot_data()
        pbar.set_description(f"Finished {out_path.stem} in {runtime:.2f}s")


if __name__ == "__main__":
    main()
