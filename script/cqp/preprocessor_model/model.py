"""Preprocessor Model"""

import itertools
import sys
from abc import ABCMeta, abstractmethod
from pathlib import Path

import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.preprocessing import StandardScaler

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.model import BitrateModel
from cqp.bitrate_model.util import KEYS, parse_resolution


class PreprocessorModel:
    """Defines the preprocessor model interface"""

    __metaclass__ = ABCMeta

    def __init__(self, k_size_range, sigma_range):
        self.__config = list(itertools.product(*[k_size_range, sigma_range]))

    @abstractmethod
    def predict_rate(self, SA, TA, QP, f, n, r, k_size, sigma):
        raise NotImplementedError("Don't call me, I'm abstract.")

    def estimate_filter_parameter(self, qp_single, df_encoding_parameter):
        df_preprocessed = pd.DataFrame(
            [
                {
                    KEYS.QP: qp_single,
                    KEYS.KSIZE: k_size,
                    KEYS.SIGMA: sigma,
                    KEYS.BITRATE: 1000
                    * self.predict_rate(
                        df_encoding_parameter[KEYS.SA],
                        df_encoding_parameter[KEYS.TA],
                        qp_single,
                        df_encoding_parameter[KEYS.RATE],
                        df_encoding_parameter[KEYS.GOP],
                        df_encoding_parameter[KEYS.RES],
                        k_size,
                        sigma,
                    ),
                }
                for k_size, sigma in self.__config
            ]
        )
        rate_multi = 1000 * self.predict_rate(
            df_encoding_parameter[KEYS.SA],
            df_encoding_parameter[KEYS.TA],
            df_encoding_parameter[KEYS.QP],
            df_encoding_parameter[KEYS.RATE],
            df_encoding_parameter[KEYS.GOP],
            df_encoding_parameter[KEYS.RES],
            df_encoding_parameter[KEYS.KSIZE],
            df_encoding_parameter[KEYS.SIGMA],
        )

        # Match bitrates for multi encoder parameter and single encoder with filter
        min_diff_idx = (df_preprocessed[KEYS.BITRATE] - rate_multi).abs().idxmin()
        return df_preprocessed.iloc[min_diff_idx]


class AnalyticPreprocessorModel(PreprocessorModel):
    """Estimate the filter parameter using the analytical bitrate model"""

    def __init__(self, k_size_range, sigma_range, model: BitrateModel):
        super().__init__(k_size_range, sigma_range)
        self.__model = model

    def predict_rate(self, SA, TA, QP, f, n, r, k_size, sigma):
        return self.__model.evaluate(SA, TA, QP, f, n, r, k_size, sigma)


class MLPreprocessorModel(PreprocessorModel):
    """Estimate the filter parameter using the machine learning based bitrate model"""

    def __init__(self, k_size_range, sigma_range, scaler_input, scaler_output, model):
        super().__init__(k_size_range, sigma_range)
        self.__input_scaler = StandardScaler().fit(scaler_input)
        self.__output_scaler = StandardScaler().fit(scaler_output)
        self.__model = model

    @staticmethod
    def from_dfw(k_size_range, sigma_range, dfw_train, dfw_val, model):
        return MLPreprocessorModel(
            k_size_range,
            sigma_range,
            np.concatenate([dfw_train.get_ml_input(), dfw_val.get_ml_input()], axis=0),
            np.concatenate(
                [dfw_train.get_ml_output(), dfw_val.get_ml_output()], axis=0
            ),
            model,
        )

    def predict_rate(self, SA, TA, QP, f, n, r, k_size, sigma):
        width, height = parse_resolution(r)
        ml_input = self.__input_scaler.transform(
            np.array([[SA, TA, QP, k_size, sigma, f, n, width, height]])
        )
        ml_input = tf.convert_to_tensor(ml_input, dtype=tf.float32)
        output = self.__model.predict(ml_input)
        return self.__output_scaler.inverse_transform(output)[0][0]
