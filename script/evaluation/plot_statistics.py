"""Plot statistics for the single_encoder package"""

import argparse
import itertools
import json
import shutil
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import mean_absolute_error

AVERAGE_VALUES = {}
MODE_KEY = "mode"


def summarize_individual_encodings(json_data, data_key):
    """Sum up individual encoding values as comparision to MUX"""
    key_elements = [
        key_elements[data_key]
        for key_elements in itertools.islice(json_data, 0, len(json_data) - 1)
    ]
    return [sum(x) for x in zip(*key_elements)]


def add_individual_sum_to_data(json_data, data_key):
    """Append summed up data to encoding results"""
    individual_sum = summarize_individual_encodings(json_data, data_key)
    sum_data = {
        "frame_count": list(range(1, len(individual_sum) + 1)),
        data_key: individual_sum,
        "encoderId": len(json_data),
    }
    return json_data + [sum_data]


def calculate_ratio_for_key(json_data, data_key):
    """Calculate ratio of individual encodings compared to sum for every key"""
    individual_sum = summarize_individual_encodings(json_data, data_key)
    ratio_key = data_key + "_ratio"

    for element in json_data:
        element_ratio = np.true_divide(element[data_key], individual_sum)
        element[ratio_key] = element_ratio.tolist()

    return ratio_key


def scale_values_of_key(json_data, data_key, scale_factor):
    """Scale encoding values of a given key"""
    for element in json_data:
        element[data_key] = [x * scale_factor for x in element[data_key]]

    return json_data


def add_subplot(plot_id, data_key, json_data, label, unit):
    """Add subplot"""
    plt.subplot(plot_id)
    plt.ylabel(f"{label} [{unit}]")

    for element in json_data:
        if isinstance(element[data_key][0], list):
            vqm_scores = [list(x) for x in zip(*element[data_key])]
            for idx, score in enumerate(vqm_scores):
                average = statistics.mean(score) if score else 0
                single_or_demux = "single" if idx == 0 else "demux"
                AVERAGE_VALUES[
                    "%d-%s-%s" % (element["encoderId"], data_key, single_or_demux)
                ] = average
                label_text = "E-%d-%s: Avg: %.2f %s" % (
                    element["encoderId"],
                    single_or_demux,
                    average,
                    unit,
                )
                plt.plot(element["frame_count"], score, label=label_text)
            if vqm_scores:
                AVERAGE_VALUES[
                    "%d-%s-mae" % (element["encoderId"], data_key)
                ] = mean_absolute_error(vqm_scores[0], vqm_scores[1])
        else:
            average = statistics.mean(element[data_key]) if element[data_key] else 0
            AVERAGE_VALUES["%d-%s" % (element["encoderId"], data_key)] = average
            label_text = "E-%d: Avg: %.2f %s" % (element["encoderId"], average, unit)
            plt.plot(element["frame_count"], element[data_key], label=label_text)

    plt.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=8)


def get_title_text(json_data):
    """Create title text base on the configured encoders"""
    title_text = ""
    for element in itertools.islice(json_data, 0, len(json_data) - 1):
        title_text += "E-%d: %dx%dx%d-%s | " % (
            element["encoderId"],
            element["width"],
            element["height"],
            element["skip_frames"],
            element["filter"],
        )
    title_text += "E-%d: mux | E-%d: sum" % (len(json_data) - 1, len(json_data))
    return title_text


def shrink_data_to_equal_length(json_data):
    """Cut all data to the length of the shortest"""
    min_length = min(len(element["frame_count"]) for element in json_data)
    for element in json_data:
        for key, value in element.items():
            if isinstance(value, list):
                element[key] = value[:min_length]

    return json_data


def create_figure(json_data):
    """Create the overall figure"""
    plt.figure("Single Encoder Analysis", figsize=[19.2, 10.8])

    json_data = shrink_data_to_equal_length(json_data)
    title_text = get_title_text(json_data)

    # Encoded Frame Size and Ratio
    data_key = "frame_size"
    json_data = scale_values_of_key(json_data, data_key, 8)  # Convert from Byte to Bit
    json_data = scale_values_of_key(
        json_data, data_key, 1e-3
    )  # Convert from Bit to kBit
    add_subplot(
        711,
        data_key,
        add_individual_sum_to_data(json_data, data_key),
        "Frame Size",
        "kBit",
    )
    plt.title(title_text, y=1.08, size=14)

    ratio_key = calculate_ratio_for_key(json_data, data_key)
    add_subplot(712, ratio_key, json_data, "Fame Size Ratio", "%")

    # Image Quality
    add_subplot(713, "mdvqm_scores", json_data, "MDVQM", "1")
    add_subplot(714, "stvqm_scores", json_data, "STVQM", "1")
    add_subplot(715, "psnr_scores", json_data, "PSNR", "db")

    # TA and SA
    add_subplot(716, "temporal_activities", json_data, "TA", "1")
    add_subplot(717, "spatial_activities", json_data, "SA", "1")

    plt.xlabel("Frame Number")
    plt.subplots_adjust(hspace=0.75)


def clear_plot_data():
    """Clear all figure handles"""
    plt.clf()
    plt.close()


def plot_data(file_name: Path, out_path: Path):
    """Plot the data for a given result json"""
    file_path = get_file_path(file_name, out_path)
    AVERAGE_VALUES[MODE_KEY] = file_path.stem
    AVERAGE_VALUES["qp"] = file_path.stem[-2:]

    json_data = json.loads(file_name.read_text())

    create_figure(json_data)
    plt.savefig(file_path.with_suffix(""))

    if file_name.resolve() != file_path.resolve():
        shutil.copy(str(file_name), str(file_path))


def get_file_path(file_name: Path, out_path: Path) -> Path:
    """Resolve file path if output path is a dir or file"""
    if out_path.is_dir():
        file_path = out_path.joinpath(file_name.name)
    else:
        file_path = out_path.with_suffix(".json")
    return file_path.resolve()


def main():
    """main"""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "stats", type=argparse.FileType("r"), help="Path to the statistics file"
    )
    parser.add_argument(
        "-o",
        "--out_dir",
        type=str,
        default=Path.cwd(),
        help="Path to the output directory",
    )
    parser.add_argument("-s", "--show", action="store_true", help="Show the plot")

    args = parser.parse_args()
    plot_data(Path(args.stats.name), Path(args.out_dir))

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
