"""Helper functions for application evaluation"""

import math
import random
import statistics
import sys
from pathlib import Path
import pandas as pd

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.util import KEYS


def get_random_qps(qp_list, keys):
    """Generate random QP dict for every video as key"""
    qp_range = 15
    qp_min = random.randint(min(qp_list), max(qp_list) - qp_range)
    qp_max = qp_min + qp_range
    return {key: random.randint(qp_min, qp_max) for key in keys}


def get_multi_encoder_gt(dfw, qps):
    """Find ground truth for multiple individual encoders in dataframe"""
    df_multi = dfw.include(variable_qp=True)
    return pd.DataFrame(
        [
            df_multi.loc[(df_multi[KEYS.QP] == value)].loc[key]
            for key, value in qps.items()
        ]
    )


def get_single_encoder_gt_min_qp(dfw, qps):
    """Find ground truth for single encoder with min qp in dataframe"""
    df_single = dfw.include(variable_qp=True)
    qp_single = min(qps.values())
    return pd.DataFrame(
        [
            df_single.loc[(df_single[KEYS.QP] == qp_single)].loc[key]
            for key in qps.keys()
        ]
    )


def get_single_encoder_gt_matching_br(dfw, qps, sfv_bitrate):
    """Find ground truth for single encoder with closest matching supferframe bitrate"""
    df_single = dfw.include(variable_qp=True)
    sfv_bitrate_diffs = abs(df_single.groupby([KEYS.QP]).bitrate.sum() - sfv_bitrate)
    min_bitrate_diff = min(sfv_bitrate_diffs)
    min_bitrate_diff_qp = sfv_bitrate_diffs[
        sfv_bitrate_diffs == min_bitrate_diff
    ].index[0]
    return get_single_encoder_gt_min_qp(
        dfw, {key: min_bitrate_diff_qp for key in qps.keys()}
    )


def get_single_encoder_solutions(dfw, qp_single):
    """Find single encoder solutions"""
    df_single = dfw.include(variable_qp=True, variable_k_size=True, variable_sigma=True)
    return df_single.loc[(df_single[KEYS.QP] == qp_single)].reset_index()


def get_oracle_gt(df_single, multi, qps):
    """Find minimum bitrate difference ground truth in dataframe"""
    min_diff_idx = [
        (
            df_single.loc[df_single["name"] == key][KEYS.BITRATE]
            - multi.loc[key][KEYS.BITRATE]
        )
        .abs()
        .idxmin()
        for key in qps.keys()
    ]
    return pd.DataFrame([df_single.iloc[idx] for idx in min_diff_idx])


def get_filter_model_gt(df_single, prediction):
    """Find ground truth for filter model in dataframe"""
    return pd.merge(df_single, prediction, on=["name", KEYS.KSIZE, KEYS.SIGMA, KEYS.QP])


def collect_experiment_encoding_parameter(
    multi, single, single_br, oracle, analytic, machine_learning
):
    """Collect encoding parameter from bitrate model estimation"""
    return pd.concat(
        [
            collect_encoding_parameter_for(multi, "multi"),
            collect_encoding_parameter_for(single, "single-qp"),
            collect_encoding_parameter_for(single_br, "single-br"),
            collect_encoding_parameter_for(oracle, "oracle"),
            collect_encoding_parameter_for(analytic, "analytic"),
            collect_encoding_parameter_for(machine_learning, "ml"),
        ]
    )


def collect_encoding_parameter_for(data_frame, mode):
    """Collect encoding parameter for the given mode"""
    data = []
    for name in KEYS.TEST_SET_CIF:
        row = (
            data_frame.loc[name]
            if name in data_frame.index
            else data_frame.loc[data_frame["name"] == name].iloc[0, :]
        )
        data.append(
            {
                "mode": mode,
                "name": name,
                KEYS.QP: row[KEYS.QP],
                KEYS.GOP: row[KEYS.GOP],
                KEYS.RES: row[KEYS.RES],
                KEYS.RATE: row[KEYS.RATE],
                KEYS.KSIZE: row[KEYS.KSIZE],
                KEYS.SIGMA: row[KEYS.SIGMA],
            }
        )
    return pd.DataFrame(data)


def get_rmse_from_abs_difference(abs_diff_list):
    """Calculate the RMSE from a list of element wise differences"""
    return math.sqrt(statistics.mean([elem * elem for elem in abs_diff_list]))
