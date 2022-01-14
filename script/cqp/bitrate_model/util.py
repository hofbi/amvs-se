"""Common utilities for analytic and ml based bitrate model"""

import math

import numpy as np
import pandas as pd
from sklearn.metrics import mean_squared_error
from tqdm import tqdm
from dataclasses import dataclass
import warnings


class KEYS:
    """Define keys commonly used in all data files"""

    TRAINING_SET_CIF = [
        "akiyo",
        "bowing",
        "bridge-close",
        "coastguard",
        "container",
        "crew",
        "flower",
        "football",
        "foreman",
        "hall",
        "harbour",
        "highway",
        "husky",
        "mobile",
        "pamphlet",
        "paris",
        "sign_irene",
        "silent",
        "soccer",
        "students",
        "tempete",
        "waterfall",
    ]
    TEST_SET_CIF = [
        "bus",
        "city",
        "deadline",
        "mother-daughter",
    ]
    VALIDATION_SET_CIF = [
        "bridge-far",
        "ice",
        "news",
        "stefan",
    ]
    TRAINING_SET_HD = [
        "blue_sky",
        "controlled_burn",
        "dinner",
        "ducks_take_off",
        "in_to_tree",
        "life",
        "pedestrian_area",
        "red_kayak",
        "riverbed",
        "rush_field_cuts",
        "rush_hour",
        "snow_mnt",
        "station2",
        "sunflower",
        "touchdown_pass",
        "tractor",
        "west_wind_easy",
    ]
    VALIDATION_SET_HD = [
        "old_town_cross",
        "park_joy",
        "speed_bag",
    ]
    TEST_SET_HD = [
        "aspen",
        "crowd_run",
        "factory",
    ]
    TRAINING_SET = TRAINING_SET_CIF + TRAINING_SET_HD
    VALIDATION_SET = VALIDATION_SET_CIF + VALIDATION_SET_HD
    TEST_SET = TEST_SET_CIF + TEST_SET_HD
    ALL_SETS = TRAINING_SET + VALIDATION_SET + TEST_SET
    SINGLE_VIDEO = "foreman"
    SA = "sa"
    TA = "ta"
    QP = "qp"
    KSIZE = "k_size"
    SIGMA = "sigma"
    RES = "resolution"
    WIDTH = "width"
    HEIGHT = "height"
    RATE = "rate"
    GOP = "gop_len"
    BITRATE = "bitrate"


@dataclass
class MaxBitrateParameter:
    """Encoding Parameters where the maximum bitrate is reached"""

    qp_min: int = 24
    f_max: int = 30  # Hz
    gop_min: int = 1
    w_max: int = 352  # px
    h_max: int = 288  # px
    k_size_min: int = 1
    sigma_min: float = 0.5
    # Note: our data also contain sigma = 0 which is silently changed to another sigma by opencv so the values of
    # sigma = 0 are not correct

    @property
    def res_max(self):
        return f"{self.w_max}x{self.h_max}"


class DataFrameWrapper:
    """Provide several helper functions for working with the experimental data file structure"""

    def __init__(self, df, max_param=MaxBitrateParameter()):
        self.__df = df
        self.__df[[KEYS.WIDTH, KEYS.HEIGHT]] = self.__df[KEYS.RES].str.split(
            "x", expand=True
        )
        self.__df[KEYS.WIDTH] = pd.to_numeric(self.__df[KEYS.WIDTH])
        self.__df[KEYS.HEIGHT] = pd.to_numeric(self.__df[KEYS.HEIGHT])
        self.__max_param = max_param

    @property
    def video_keys(self):
        return self.__df.index.drop_duplicates()

    @property
    def df(self):
        return self.include(*[True] * 6)

    def include(
        self,
        variable_rate=False,
        variable_res=False,
        variable_gop=False,
        variable_k_size=False,
        variable_sigma=False,
        variable_qp=False,
    ):
        return self.__df.loc[
            (
                self.__df[KEYS.RATE] == self.__max_param.f_max
                if not variable_rate
                else self.__df[KEYS.RATE] <= self.__max_param.f_max
            )
            & (
                self.__df[KEYS.RES] == self.__max_param.res_max
                if not variable_res
                else self.__df[KEYS.WIDTH] <= self.__max_param.w_max
            )
            & (
                self.__df[KEYS.GOP] == self.__max_param.gop_min
                if not variable_gop
                else self.__df[KEYS.GOP] >= self.__max_param.gop_min
            )
            & (
                self.__df[KEYS.KSIZE] == self.__max_param.k_size_min
                if not variable_k_size
                else self.__df[KEYS.KSIZE] >= self.__max_param.k_size_min
            )
            & (
                self.__df[KEYS.SIGMA] == self.__max_param.sigma_min
                if not variable_sigma
                else self.__df[KEYS.SIGMA] >= self.__max_param.sigma_min
            )
            & (
                self.__df[KEYS.QP] == self.__max_param.qp_min
                if not variable_qp
                else self.__df[KEYS.QP] >= self.__max_param.qp_min
            )
        ]

    def get_ml_input(self):
        return self.include(*[True] * 6)[
            [
                KEYS.SA,
                KEYS.TA,
                KEYS.QP,
                KEYS.KSIZE,
                KEYS.SIGMA,
                KEYS.RATE,
                KEYS.GOP,
                KEYS.WIDTH,
                KEYS.HEIGHT,
            ]
        ].to_numpy()

    def get_ml_output(self):
        return self.df[[KEYS.BITRATE]].to_numpy()

    def get_ml_max_output(self):
        df = self.df
        return np.concatenate(
            [
                np.full(
                    df.loc[key][KEYS.BITRATE].size,
                    df.loc[key][KEYS.BITRATE].max(),
                )
                for key in self.video_keys
            ]
        )


def read_df_by_keys(data_files, keys):
    """Read all csv files present in the keys list into a single dataframe"""
    return pd.concat(
        [
            pd.read_csv(data_file, index_col="name")
            for data_file in tqdm(data_files, desc="Reading csv files...")
            if any(key in data_file.name for key in keys)
        ]
    )


def get_rmse(measured, estimated):
    """Calculate the RMSE between two lists"""
    return math.sqrt(mean_squared_error(measured, estimated))


def get_pc(measured, estimated):
    """Calculate the pearson correlation between two lists"""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return np.corrcoef(measured, estimated)[0, 1]


def print_rmse_and_pc(text, rmse, pc):
    """Print RMSE and PC in predefined format"""
    print(f"{text:40s}: RMSE: {rmse:.5f}   PC: {pc:.5f}")


def print_rmse_normalized_rmse_and_pc(text, rmse, nrmse, pc):
    """Print RMSE, normalized RMSE, and PC in predefined format"""
    print(
        f"{text:40s}: RMSE: {rmse:.2f} kBit/s   normalized RMSE: {nrmse:.5f}   PC: {pc:.5f}"
    )


def get_expected_correction_factor(filtered_df, video_keys):
    """Create a series with the expected correction factor values as ground truth"""
    return pd.Series(
        np.concatenate(
            [
                filtered_df.loc[key][KEYS.BITRATE]
                / max(filtered_df.loc[key][KEYS.BITRATE])
                for key in video_keys
            ]
        ),
        index=filtered_df.index,
    )


def get_normalized_bitrate(df):
    """Get the normalized bitrate of a dataframe"""
    bitrate = df[KEYS.BITRATE]
    return bitrate / max(bitrate)


def parse_resolution(resolution):
    """Parse width and height from a frame resolution string"""
    resolution = resolution.split("x")
    return int(resolution[0]), int(resolution[1])
