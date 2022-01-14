"""Plotting"""

import sys
import matplotlib.pyplot as plt
import tikzplotlib
import pandas as pd
import numpy as np
from pathlib import Path

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.util import (
    get_rmse,
    get_pc,
    get_normalized_bitrate,
    KEYS,
)


def plot_sa_ta(ax_handle, df, video_keys, label, marker="o"):
    """Plot SA over TA values for all video keys"""
    unique_rows = [df.loc[key].head(1) for key in video_keys]
    ta = [row["ta"] for row in unique_rows]
    sa = [row["sa"] for row in unique_rows]
    ax_handle.scatter(ta, sa, label=label, marker=marker)
    for row in unique_rows:
        ax_handle.text(
            row["ta"][0],
            row["sa"][0],
            f'{row.index[0].replace("_", "-")} ({KEYS.ALL_SETS.index(row.index[0]) + 1})',
        )

    plt.xlabel("TA")
    plt.ylabel("SA")


def plot_correction_factor(
    x, expected, predicted, param, x_label, y_label, param_label, unique=False
):
    """Plot estimated and predicted values of a correction factor"""
    unique_param = param[0] if unique else param
    print(f"{param_label}: {unique_param}")
    plot_correction_factor_st(x, expected, predicted, x_label, y_label, unique)


def plot_correction_factor_st(x, expected, predicted, x_label, y_label, unique=False):
    """Plot estimated and predicted values of a spatio-temporal fitted correction factor"""
    unique_x = pd.unique(x) if unique else x
    unique_expected = (
        np.array(expected)[x.index.get_loc(KEYS.SINGLE_VIDEO)] if unique else expected
    )
    unique_predicted = (
        np.array(predicted)[x.index.get_loc(KEYS.SINGLE_VIDEO)] if unique else predicted
    )
    plt.figure()
    plt.scatter(unique_x, unique_expected[: len(unique_x)])
    plt.plot(unique_x, unique_predicted[: len(unique_x)])
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    tikzplotlib.save(f"{y_label.lower()}.tex")
    print_metrics_for_plot(expected, predicted)


def plot_content_dependent_parameter(
    keys, expected, predicted, param, y_label, param_label
):
    """Plot distribution of content dependent parameter over all videos"""
    x_range = np.array(range(0, len(expected)))
    plt.bar(x_range - 0.15, expected, width=0.25)
    plt.bar(x_range + 0.15, predicted, width=0.25)
    plt.xticks(x_range, keys)
    plt.ylabel(y_label)
    print(f"{param_label}: {param}")
    print_metrics_for_plot(expected, predicted)


def print_metrics_for_plot(expected, predicted):
    """Print RMSE and PC"""
    print(f"RMSE: {get_rmse(expected, predicted):.5f}")
    print(f"PC:   {get_pc(expected, predicted):.5f}")


def plot_correction_factor_bitrate_contribution(
    df, key, x_label, plt_label=KEYS.SINGLE_VIDEO
):
    """Plot how much the correction factor contributes to the overall bitrate"""
    plt.scatter(
        df.loc[KEYS.SINGLE_VIDEO][key],
        get_normalized_bitrate(df.loc[KEYS.SINGLE_VIDEO]),
        label=plt_label,
    )
    plt.xlabel(x_label)
    plt.ylabel("Bitrate")
    plt.title(f"{x_label} Contribution")
    plt.legend()


def plot_bitrates(bitrate_measured, bitrate_estimated, key, decimals=-1):
    """
    Plot measured over estimated bitrates
    By default rounded to -1 decimals as plot as larger plot cannot be handled by latex
    Accuracy is still fine as plot is shown in very small size
    """
    rmse = get_rmse(bitrate_measured, bitrate_estimated)
    pc = get_pc(bitrate_measured, bitrate_estimated)
    plt.figure()
    estimated = np.around(bitrate_estimated, decimals=decimals)
    measured = np.around(bitrate_measured, decimals=decimals)
    plt.scatter(estimated, measured, marker="x")
    plt.xlabel("Estimated Bitrate [kBit/s]")
    plt.ylabel("Measured Bitrate [kBit/s]")
    plt.title(f"Bitrate model comparison {key} (RMSE: {rmse:.2f}, PC: {pc:.5f})")


def save_plot(key, path: Path = None):
    """Save the plot as tikz plot and reduce filesize"""
    tex_file = (
        Path(f"{key}.tex").absolute() if path is None else path.joinpath(f"{key}.tex")
    )
    tikzplotlib.save(tex_file, float_format=".0f")
    remove_duplicate_lines(tex_file)


def remove_duplicate_lines(file_path: Path) -> None:
    """Remove duplicate lines from the tikzplot file"""
    seen_lines = set()
    input_lines = file_path.open("r").readlines()
    out = file_path.open("w")
    for line in input_lines:
        if line not in seen_lines:
            out.write(line)
            seen_lines.add(line)
    out.close()
