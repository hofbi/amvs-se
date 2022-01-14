""""
Data evaluation for CQP experiments with linear regression
Fit a plane to estimate the kernel size of the preprocessing filter to compensate the different QPs
"""

import argparse
import sys
from itertools import cycle
from pathlib import Path
from statistics import mean
from dataclasses import dataclass
from typing import List, Callable
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import linregress
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error
from tqdm import tqdm
from scipy.optimize import least_squares
from sklearn.model_selection import LeaveOneOut

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from evaluation.plot_statistics import MODE_KEY
from cqp.bitrate_model.util import get_pc

QP_KEY = "qp"
BASE_KEY = "base"
FILTER_KEY = "median"
K_SIZE = [3, 5, 7, 9]
FILTERED_BR = "1-frame_size"
REF_BR = "0-frame_size"
FRAME_RATE = 20
TA_KEY = "TA"
SA_KEY = "SA"
MODE_PREFIXES = [
    "SA-low-TA-low_",
    "SA-low-TA-mid_",
    "SA-low-TA-high_",
    "SA-mid-TA-low_",
    "SA-mid-TA-mid_",
    "SA-mid-TA-high_",
    "SA-high-TA-low_",
    "SA-high-TA-mid_",
    "SA-high-TA-high_",
    # "akiyo_",
    # "container_",
    # "waterfall_",
    # "foreman_",
    # "hall_",
    # "mobile_",
    # "mother-daughter_",
    # "paris_",
    # "highway_",
    # "tempete_",
]


@dataclass
class ModelParams:
    """Model parameter"""

    prefix: str
    intercept: int
    normal: List
    SA: int
    TA: int
    plane: LinearRegression
    X: np.array
    y: np.array


def parse_arguments():
    """Parse CLI arguments"""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "avg_data", type=argparse.FileType("r"), help="Path to the average.csv file"
    )
    parser.add_argument("--show", action="store_true", help="Show the plot")
    return parser.parse_args()


def read_df(file_path):
    """Read the pandas dataframe from csv and remove the QP key"""
    df = pd.read_csv(file_path)
    # remove qp from mode key
    df[MODE_KEY] = [val[:-3] for val in df[MODE_KEY]]
    return df


def main():
    """main"""
    args = parse_arguments()

    df = read_df(Path(args.avg_data.name))

    model_params_list, eval_data = zip(
        *[evaluate_block(df, prefix) for prefix in tqdm(MODE_PREFIXES)]
    )

    # Calculate average model parameter and ST model
    average_model = get_average_model_params(model_params_list)
    st_model = get_st_model(model_params_list)

    # Plot setup
    plt.close("all")
    fig = plt.figure()
    current_ax = fig.gca(projection="3d")

    # Predict filter for some QP pairs
    for index, param in enumerate(model_params_list):
        plot_plane(current_ax, param.plane, param.prefix, index)
    current_ax.legend()

    training_evaluation(average_model, model_params_list, st_model)

    evaluation_data = pd.concat(eval_data)
    avg_prediction = average_model.plane.predict(
        evaluation_data[["qp_i", "qp_sf"]].to_numpy()
    )
    bitrate_evaluation(evaluation_data, avg_prediction, df, "average training")

    if args.show:
        plt.show()


def bitrate_evaluation(evaluation_data, prediction, df, model_name):
    """Validate the estimated bitrate"""
    prediction_rounded = np.array(
        [np.floor(a) if np.mod(np.floor(a), 2) != 0 else np.ceil(a) for a in prediction]
    ).clip(min=1)

    print(
        f"MAE {model_name} prediction: {mean_absolute_error(evaluation_data.ks, prediction)}"
    )
    print(
        f"MAE {model_name} prediction rounded: {mean_absolute_error(evaluation_data.ks, prediction_rounded)}"
    )

    prediction_bitrate_sf = [
        df.loc[
            (df[MODE_KEY].str.contains(key[:-1]))
            & (df[MODE_KEY].str.contains(str(int(pred))))
            & (df[QP_KEY] == qp)
        ][FILTERED_BR]
        * 20
        for pred, qp, key in zip(
            prediction_rounded, evaluation_data.qp_sf, evaluation_data.key
        )
    ]

    print(
        f"MAE {model_name} bitrate SF: {mean_absolute_error(evaluation_data.br_sf, prediction_bitrate_sf)}"
    )


def training_evaluation(average_model, model_params_list, st_model):
    """Validate the training error"""
    average_prediction_mae = mean(
        [
            mean_absolute_error(param.y, param.plane.predict(param.X))
            for param in model_params_list
        ]
    )
    average_prediction_pc = mean(
        [get_pc(param.y, param.plane.predict(param.X)) for param in model_params_list]
    )
    average_model_mae = mean(
        [
            mean_absolute_error(param.y, average_model.plane.predict(param.X))
            for param in model_params_list
        ]
    )
    average_model_pc = mean(
        [
            get_pc(param.y, average_model.plane.predict(param.X))
            for param in model_params_list
        ]
    )
    st_model_mae = mean(
        [
            mean_absolute_error(param.y, st_model.predict(param.SA, param.TA, param.X))
            for param in model_params_list
        ]
    )
    st_model_pc = mean(
        [
            get_pc(param.y, st_model.predict(param.SA, param.TA, param.X))
            for param in model_params_list
        ]
    )
    print(f"Average individual MAE {average_prediction_mae}")
    print(f"Average individual PC {average_prediction_pc}")
    print(f"Average model MAE {average_model_mae}")
    print(f"Average model PC {average_model_pc}")
    print(f"ST model MAE {st_model_mae}")
    print(f"ST model PC {st_model_pc}")


def predict_with_plane_equation(a, b, c, samples):
    """Estimate the bitrate using the fitted plane"""
    return [a + b * sample[0] + c * sample[1] for sample in samples]


def get_average_model_params(model_params_list):
    """Calculate average plane parameter"""
    X_all = np.concatenate([param.X for param in model_params_list], axis=0)
    y_all = np.concatenate([param.y for param in model_params_list], axis=0)
    linear_reg = LinearRegression()
    linear_reg.fit(X_all, y_all)

    return ModelParams(
        "average",
        linear_reg.intercept_,
        linear_reg.coef_,
        mean([param.SA for param in model_params_list]),
        mean([param.TA for param in model_params_list]),
        linear_reg,
        X_all,
        y_all,
    )


def get_st_model(model_params_list):
    """Calculate spatio-temporal plane model"""
    intercepts = np.array([param.plane.intercept_ for param in model_params_list])
    coefs = np.array([param.plane.coef_ for param in model_params_list])
    SA = np.array([param.SA for param in model_params_list])
    TA = np.array([param.TA for param in model_params_list])

    # Train a_st
    a_st_trainer = GLMTrainer(STModel.a_st_model, intercepts, SA, TA)
    alpha = a_st_trainer.train()
    expected_a, predicted_a = a_st_trainer.validate(alpha)

    # Train b_st
    b_st_trainer = GLMTrainer(STModel.b_st_model, coefs[:, 0], SA, TA)
    beta = b_st_trainer.train()
    expected_b, predicted_b = b_st_trainer.validate(beta)

    # Train c_st
    c_st_trainer = GLMTrainer(STModel.c_st_model, coefs[:, 1], SA, TA)
    gamma = c_st_trainer.train()
    expected_c, predicted_c = c_st_trainer.validate(gamma)

    print(
        f"Trained st model with MAEs for a = {mean_absolute_error(expected_a, predicted_a):.5f}, "
        f"b = {mean_absolute_error(expected_b, predicted_b):.5f}, "
        f"c = {mean_absolute_error(expected_c, predicted_c):.5f}"
    )

    return STModel(alpha, beta, gamma)


def evaluate_block(df, prefix):
    """Evaluate for one video block"""
    base_key = f"{prefix}{BASE_KEY}"
    filter_key = f"{prefix}{FILTER_KEY}"

    # Plot bitrate
    plt.clf()
    cycol, filter_keys = plot_bitrate(base_key, df, filter_key)
    plt.savefig(f"{prefix}bitrate")

    # Plot QPs with matching bitrate
    plt.clf()
    base, lines, eval_data = get_qp_match(base_key, cycol, df, filter_key, filter_keys)
    plt.savefig(f"{prefix}QPs")

    X, f, linear_regression, y = fit_plane(base, lines, prefix)
    plt.savefig(f"{prefix}plane")

    # evaluate error
    linear_regression_mae = mean_absolute_error(y, linear_regression.predict(X))
    linear_regression_pc = get_pc(y, linear_regression.predict(X))
    plane_error = mean_absolute_error(
        y,
        predict_with_plane_equation(
            linear_regression.intercept_,
            linear_regression.coef_[0],
            linear_regression.coef_[1],
            X,
        ),
    )

    tqdm.write(f"\nPrefix: {prefix}")
    tqdm.write(f"Linear regression MAE: {linear_regression_mae}")
    tqdm.write(f"Linear regression PC: {linear_regression_pc}")
    tqdm.write(
        f"FS = {linear_regression.intercept_} + {linear_regression.coef_[0]} * QP_i + "
        f"{linear_regression.coef_[1]} * QP_SF"
    )
    tqdm.write(f"Plane equation MAE: {plane_error}")

    return (
        ModelParams(
            prefix,
            linear_regression.intercept_,
            linear_regression.coef_,
            eval_data[SA_KEY].iloc[0],
            eval_data[TA_KEY].iloc[0],
            linear_regression,
            X,
            y,
        ),
        eval_data,
    )


def fit_plane(base, lines, prefix):
    """Fit plane into the collection of lines"""
    # Build mesh for 3D plots
    qps = [base.index] * 4
    qpsf = []
    f = []
    for qps_row, line, k_size in zip(qps, lines, K_SIZE):
        qpsf.append(line.intercept + line.slope * qps_row)
        f.append([k_size] * len(qps_row))
    # 3D plot of best fitting lines
    plt.clf()
    fig = plt.figure()
    current_ax = fig.gca(projection="3d")
    current_ax.scatter(qps, qpsf, f)
    # find best fitting plane
    linear_regression = LinearRegression(normalize=True)
    X = np.transpose([np.array(qps).flatten(), np.array(qpsf).flatten()])
    y = np.array(f).flatten()
    linear_regression.fit(X, y)
    # plot plane
    plot_plane(current_ax, linear_regression, prefix)
    plt.title("Filter kernel size over QP_SF and QP_single")
    current_ax.view_init(azim=45, elev=15)
    return X, f, linear_regression, y


def get_qp_match(base_key, cycol, df, filter_key, filter_keys, plot=True):
    """Find the closest matching bitrates pair's QP with and without filters"""
    next(cycol)  # To have same colors as in previous plot as base is no longer shown
    selected_df = df.loc[df[MODE_KEY].str.contains(f"{base_key}|{filter_key}")]
    pivot = selected_df.pivot(index=MODE_KEY, columns=QP_KEY, values=FILTERED_BR)
    base = pivot.loc[base_key]
    lines = []
    evaluation_data = []
    for filter_key in filter_keys:
        # find best fitting line
        min_sf_qp = [
            abs(pivot.loc[filter_key] - base_val).idxmin() for base_val in base
        ]
        min_slope_index = (
            min_sf_qp.index(next(filter(lambda x: x != min(min_sf_qp), min_sf_qp))) - 1
        )
        eval_data = pd.DataFrame(
            {
                "key": [filter_key] * base.index[min_slope_index:].size,
                "ks": [int(filter_key[-1])] * base.index[min_slope_index:].size,
                "qp_i": base.index[min_slope_index:].to_numpy(),
                "qp_sf": min_sf_qp[min_slope_index:],
                "br_i": base[min_slope_index:].to_numpy() * FRAME_RATE,
                "br_sf": pivot.loc[filter_key][min_sf_qp][min_slope_index:].to_numpy()
                * FRAME_RATE,
                SA_KEY: selected_df.loc[df[MODE_KEY] == filter_key][SA_KEY][
                    min_slope_index:
                ].to_numpy(),
                TA_KEY: selected_df.loc[df[MODE_KEY] == filter_key][TA_KEY][
                    min_slope_index:
                ].to_numpy(),
            }
        )
        lineres = linregress(base.index[min_slope_index:], min_sf_qp[min_slope_index:])
        lines.append(lineres)
        evaluation_data.append(eval_data)
        # Plot points and best fitting line
        if plot:
            label_text = f"{filter_key}: {lineres.slope:.2}x + {lineres.intercept:.2}"
            color = next(cycol)
            plt.scatter(
                eval_data["qp_i"], eval_data["qp_sf"], label=label_text, color=color
            )
            plt.plot(
                eval_data["qp_i"],
                lineres.intercept + lineres.slope * eval_data["qp_i"],
                color=color,
                linewidth=0.5,
            )
    if plot:
        plt.legend()
        plt.ylabel("QP SF")
        plt.xlabel("QP single")
        plt.title("QP_SF over QP_single with clostest bitrate match")
        plt.grid(True)
    return base, lines, pd.concat(evaluation_data)


def plot_bitrate(base_key, df, filter_key):
    """Plot the bitrate for a given key"""
    cycol = cycle("kbgry")
    filter_keys = [f"{filter_key}_{k_size}" for k_size in K_SIZE]
    for filter_key in [base_key] + filter_keys:
        filtered_data = df.loc[df[MODE_KEY].str.contains(filter_key)]
        bitrate = filtered_data[FILTERED_BR] * FRAME_RATE
        plt.scatter(
            filtered_data[QP_KEY],
            bitrate,
            label=filter_key.replace("_", "-"),
            color=next(cycol),
        )

    plt.legend()
    plt.ylabel("Bitrate [kBit/s]")
    plt.xlabel("QP")
    plt.title("Bitrate over QPs for different filter")
    plt.grid(True)
    return cycol, filter_keys


def plot_plane(ax_handle, linear_regression, label, index=0):
    """Plot the plain from the linear regression model"""
    surf = ax_handle.plot_surface(
        np.array([[21, 21], [37, 37]]),
        np.array([[21, 37], [21, 37]]),
        linear_regression.predict(
            np.array([[21, 21, 37, 37], [21, 37, 21, 37]]).T
        ).reshape((2, 2)),
        alpha=0.5,
        color=f"C{index}",
        label=label,
    )
    # Bug fix for 3d plots: https://github.com/matplotlib/matplotlib/issues/4067#issuecomment-753666340
    surf._facecolors2d = surf._facecolor3d
    surf._edgecolors2d = surf._edgecolor3d
    ax_handle.set_zlabel("Filter kernel size")
    ax_handle.set_ylabel("$QP_{SF}$")
    ax_handle.set_xlabel("$QP_{ind}$")


@dataclass
class GLMTrainer:
    """Generative linear modeling trainer"""

    model: Callable
    ground_truth: np.array
    SA: np.array
    TA: np.array

    @staticmethod
    def model_residuals(param, expected, st_model, SA, TA):
        return st_model(param, SA, TA) - expected

    def train(self):
        return least_squares(
            GLMTrainer.model_residuals,
            (0, 0, 0),
            args=(self.ground_truth, self.model, self.SA, self.TA),
        ).x

    def validate(self, param):
        return self.ground_truth, self.model(param, self.SA, self.TA)


def run_loocv(data, model, SA, TA):
    """Run leave one out cross validation for a given model"""
    loo = LeaveOneOut()
    mad = []
    for train_index, test_index in loo.split(data):
        param = GLMTrainer(
            model, data[train_index], SA[train_index], TA[train_index]
        ).train()
        expected, predicted = GLMTrainer(
            model, data[test_index], SA[test_index], TA[test_index]
        ).validate(param)
        mad.append(mean_absolute_error(expected, predicted))
    return mean(mad)


@dataclass
class STModel:
    """Spatio-temporal 2D linear regression model"""

    alpha: List
    beta: List
    gamma: List

    @staticmethod
    def a_st_model(alpha, SA, TA):
        return alpha[0] * (TA / SA) + alpha[1] * np.log10(SA / TA) + alpha[2]

    @staticmethod
    def b_st_model(beta, SA, TA):
        return beta[0] * SA * TA + beta[1] * (TA / SA) + beta[2]

    @staticmethod
    def c_st_model(gamma, SA, TA):
        return gamma[0] * SA * TA + gamma[1] * np.log10(SA) + gamma[2]

    def predict(self, SA, TA, samples):
        a = STModel.a_st_model(self.alpha, SA, TA)
        b = STModel.b_st_model(self.beta, SA, TA)
        c = STModel.c_st_model(self.gamma, SA, TA)
        return predict_with_plane_equation(a, b, c, samples)


if __name__ == "__main__":
    main()
