"""Plot statistics for all stats files in the folder and summarize"""

import argparse
import re
import sys
import copy
import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm

from plot_statistics import plot_data, clear_plot_data, AVERAGE_VALUES, MODE_KEY


def main():
    args = parse_args()

    eval_path = Path(args.eval_path)
    average_data, plot_path = read_average_data_from_eval_path(eval_path)

    evaluate_average_data(args.filter, average_data)
    calculate_average_diff_relative_to(average_data, args.anchor_row, plot_path)
    plt.savefig(plot_path.joinpath("average"))

    if args.show:
        plt.show()


def read_average_data_from_eval_path(eval_path: Path):
    if eval_path.is_file():
        if eval_path.suffix != ".csv":
            raise ValueError("Wrong file type. Expecting .csv")
        average_data = pd.read_csv(eval_path)
        plot_path = eval_path.resolve().parent
    elif eval_path.is_dir():
        average_values = evaluate_data(eval_path)
        average_data = get_average_data_from_values(eval_path, average_values)
        plot_path = eval_path
    else:
        raise FileNotFoundError("Evaluation path does not exist!")

    return average_data, plot_path


def calculate_average_diff_relative_to(average_data, anchor_row_name, plot_path: Path):
    anchor_row = average_data.loc[average_data[MODE_KEY].str.contains(anchor_row_name)]

    if anchor_row.empty:
        exit_with_error(
            "Anchor row %s does not exist. Specify with --anchor_row argument"
            % anchor_row_name
        )

    diff = average_data.set_index(MODE_KEY)
    anchor_row.set_index(MODE_KEY, inplace=True)

    diff = diff.sub(anchor_row.iloc[0], axis="columns")
    diff = diff.abs()
    diff.drop(index=anchor_row.index, inplace=True)
    min_diff_qp = diff.idxmin()
    diff = pd.concat([diff, pd.DataFrame([min_diff_qp], index=["min_diff_qp"])])
    diff.to_csv(plot_path.joinpath("diff.csv"), header=True)


def exit_with_error(message):
    print(message, file=sys.stderr)
    sys.exit(1)


def evaluate_average_data(filter_text, average_data):
    print("\nEvaluating average values...")
    filtered_data = average_data.loc[average_data[MODE_KEY].str.contains(filter_text)]

    plot_average_data(filter_text, filtered_data)


def get_average_data_from_values(eval_path: Path, average_values):
    average_data = pd.DataFrame(average_values)
    average_data.sort_values(by=MODE_KEY, inplace=True)
    average_data.to_csv(eval_path.joinpath("average.csv"), header=True, index=False)
    return average_data


def plot_average_data(filter_text, filtered_data):
    plt.figure("Single Encoder Filter Analysis", figsize=[19.2, 10.8])

    bitrate_header = [
        val for val in list(filtered_data) if re.search(r"\d-frame_size$", val)
    ]
    stvqm_header = [val for val in list(filtered_data) if "stvqm_scores" in val]
    mdvqm_header = [val for val in list(filtered_data) if "mdvqm_scores" in val]
    ta_header = [val for val in list(filtered_data) if "temporal_activities" in val]
    sa_header = [val for val in list(filtered_data) if "spatial_activities" in val]

    add_subplot(
        511,
        filtered_data,
        bitrate_header,
        "Bitrate [kBit]",
    )
    plt.title("Filter: %s" % filter_text, y=1.08, size=24)
    add_subplot(
        512,
        filtered_data,
        stvqm_header,
        "STVQM [1]",
    )
    add_subplot(
        513,
        filtered_data,
        mdvqm_header,
        "MDVQM [1]",
    )
    add_subplot(
        514,
        filtered_data,
        ta_header,
        "TA [1]",
    )
    add_subplot(
        515,
        filtered_data,
        sa_header,
        "SA [1]",
    )
    plt.subplots_adjust(hspace=0.6)


def add_subplot(plot_id, filtered_data, y_values, y_label):
    ax = plt.subplot(plot_id)
    filtered_data.plot(x=MODE_KEY, y=y_values, ax=ax).legend(
        loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=7
    )
    plt.ylabel(y_label)


def evaluate_data(eval_path):
    average_values = []
    for root, dirs, files in os.walk(eval_path):
        stats_files = [
            stats_file for stats_file in files if stats_file.endswith(".json")
        ]
        stats_files.sort()
        pbar = tqdm(stats_files)
        for stats_file in pbar:
            plot_data(Path(root).joinpath(stats_file), Path(root))
            average_values.append(copy.deepcopy(AVERAGE_VALUES))
            clear_plot_data()
            pbar.set_description("Finished plot for %s" % stats_file)
    return average_values


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-e",
        "--eval_path",
        type=str,
        default=Path(__file__)
        .parent.parent.parent.joinpath("config")
        .joinpath("evaluation"),
        help="Path to the evaluation data. Path can either be a directory to "
        "evaluate all data within this and create an average file or the "
        "path to the average csv file. This will only create the average plot.",
    )
    parser.add_argument(
        "-f",
        "--filter",
        type=str,
        default=".*",
        help="Regular expression based filter for includes to the average plots",
    )
    parser.add_argument(
        "-a",
        "--anchor_row",
        type=str,
        default="base",
        help="Name of the anchor row. The difference of all other rows will be "
        "relative to this row and written to diff.csv",
    )
    parser.add_argument("-s", "--show", action="store_true", help="Show the plot")
    return parser.parse_args()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
