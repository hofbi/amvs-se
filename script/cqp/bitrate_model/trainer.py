"""Trainer for analytic bitrate model"""

import statistics
import sys
from abc import ABCMeta, abstractmethod
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from sklearn.metrics import mean_absolute_error
from sklearn.model_selection import LeaveOneOut

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.util import (
    KEYS,
    get_expected_correction_factor,
)


class BaseTrainer:
    """
    Defines the basic interace for every individual trainer.
    Implements common functionality required accross all trainers.
    """

    __metaclass__ = ABCMeta

    def __init__(self, df, model_handle, init_model_parameter):
        self.__df = df
        self.__keys = df.index.drop_duplicates()
        self.__SA = df[KEYS.SA].drop_duplicates()
        self.__TA = df[KEYS.TA].drop_duplicates()
        self.__model_handle = model_handle
        self.__init_model_parameter = init_model_parameter

    @staticmethod
    @abstractmethod
    def params_from_mp(model_parameter):
        raise NotImplementedError("Don't call me, I'm abstract.")

    @abstractmethod
    def train(self):
        raise NotImplementedError("Don't call me, I'm abstract.")

    @abstractmethod
    def validate(self, _):
        raise NotImplementedError("Don't call me, I'm abstract.")

    @staticmethod
    def st_residuals(param, expected, SA, TA, model_handle):
        return model_handle(param, SA, TA) - expected

    @property
    def df(self):
        return self.__df

    @property
    def keys(self):
        return self.__keys

    @property
    def SA(self):
        return self.__SA

    @property
    def TA(self):
        return self.__TA

    @property
    def model_handle(self):
        return self.__model_handle

    @property
    def init_mp(self):
        return self.__init_model_parameter


class RMaxTrainer(BaseTrainer):
    """Implements the trainer interface for estimating the maximum bitrate"""

    def __init__(self, df, model_handle, init_model_parameter):
        super().__init__(df, model_handle, init_model_parameter)
        self.expected_r_max = df[KEYS.BITRATE].to_numpy() * 0.001  # Convert to kBit/s

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.rho

    def validate(self, rho):
        predicted_r_max = self.model_handle.r_max(rho, self.SA, self.TA)
        return self.expected_r_max.tolist(), predicted_r_max.tolist()

    def train(self):
        rho0 = self.params_from_mp(self.init_mp)
        return least_squares(
            BaseTrainer.st_residuals,
            rho0,
            args=(self.expected_r_max, self.SA, self.TA, self.model_handle.r_max),
        ).x


class CFTrainer(BaseTrainer):
    """
    Extends the basic trainer interface for correction factor specific functionality
    Implements common functionality used for all correction factor trainings
    """

    def __init__(self, df, model_handle, init_model_parameter, x_key):
        super().__init__(df, model_handle, init_model_parameter)
        self.__expected_cf = get_expected_correction_factor(df, self.keys)
        self.__X = df[x_key]

    @property
    def cf0(self):
        return 0

    @property
    @abstractmethod
    def cf_handle(self):
        raise NotImplementedError("Don't call me, I'm abstract.")

    @property
    @abstractmethod
    def st_handle(self):
        raise NotImplementedError("Don't call me, I'm abstract.")

    @staticmethod
    def cf_residuals(param, expected, X, cf_handle):
        return cf_handle(X, param) - expected

    def _predict_cf(self, param):
        return self.cf_handle(
            self.X.to_numpy(),
            np.repeat(
                self.st_handle(param, self.SA, self.TA),
                self.X.drop_duplicates().size,
            ),
        )

    def validate(self, param):
        predicted_cf = self._predict_cf(param)
        return self.expected_cf.values.tolist(), predicted_cf.tolist()

    def validate_cf(self, param):
        predicted_cf = self.cf_handle(
            self.X.to_numpy(), np.repeat(param, self.X.drop_duplicates().size)
        )
        return self.expected_cf.values.tolist(), predicted_cf.tolist()

    @property
    def expected_cf(self):
        return self.__expected_cf

    @property
    def X(self):
        return self.__X

    def validate_st(self, p, param):
        predicted_p = self.st_handle(param, self.SA, self.TA)
        return np.concatenate(p), predicted_p

    def train_cf(self):
        return [
            least_squares(
                self.cf_residuals,
                self.cf0,
                args=(
                    self.expected_cf[key].to_numpy(),
                    self.X[key].to_numpy(),
                    self.cf_handle,
                ),
            ).x
            for key in self.keys
        ]

    def train_st(self, p):
        param0 = self.params_from_mp(self.init_mp)
        return least_squares(
            BaseTrainer.st_residuals,
            param0,
            args=(np.concatenate(p), self.SA, self.TA, self.st_handle),
        ).x

    def train(self):
        return self.train_st(self.train_cf())


class SCFTrainer(CFTrainer):
    """Implements the trainer interface for estimating the spatial correction factor"""

    def __init__(self, df_qp, model_handle, init_model_parameter):
        super().__init__(df_qp, model_handle, init_model_parameter, KEYS.QP)

    @property
    def cf_handle(self):
        return self.model_handle.scf

    @property
    def st_handle(self):
        return self.model_handle.a_st

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.alpha


class TCFTrainer(CFTrainer):
    """Implements the trainer interface for estimating the temporal correction factor"""

    def __init__(self, df_rate, model_handle, init_model_parameter):
        super().__init__(df_rate, model_handle, init_model_parameter, KEYS.RATE)

    @property
    def cf_handle(self):
        return self.model_handle.tcf

    @property
    def st_handle(self):
        return self.model_handle.b_st

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.beta


class NCFTrainer(CFTrainer):
    """Implements the trainer interface for estimating the GOP correction factor"""

    cf0 = (1, 1, 1)

    def __init__(self, df_gop, model_handle, init_model_parameter):
        super().__init__(df_gop, model_handle, init_model_parameter, KEYS.GOP)

    @property
    def cf_handle(self):
        return lambda gop, c_d_e: self.model_handle.ncf(
            gop, c_d_e[0], c_d_e[1], c_d_e[2]
        )

    @property
    def st_handle(self):
        raise NotImplementedError()

    @staticmethod
    def split_c_d_e(c_d_e):
        params = np.array(c_d_e)
        return params[:, 0], params[:, 1], params[:, 2]

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.gamma, model_parameter.delta, model_parameter.epsilon

    def validate_cf(self, c_d_e):
        size = self.X.drop_duplicates().size
        c, d, e = NCFTrainer.split_c_d_e(c_d_e)
        predicted_ncf = self.model_handle.ncf(
            self.X.to_numpy(),
            np.repeat(c, size),
            np.repeat(d, size),
            np.repeat(e, size),
        )
        return self.expected_cf.values.tolist(), predicted_ncf.tolist()

    def validate_c_st(self, c, gamma):
        return c, self.model_handle.c_st(gamma, self.SA, self.TA)

    def validate_d_st(self, d, delta):
        return d, self.model_handle.d_st(delta, self.SA, self.TA)

    def validate_e_st(self, e, epsilon):
        return e, self.model_handle.e_st(epsilon, self.SA, self.TA)

    def validate(self, gamma_delta_epsilon):
        size = self.X.drop_duplicates().size
        predicted_ncf = self.model_handle.ncf(
            self.X.to_numpy(),
            np.repeat(
                self.model_handle.c_st(gamma_delta_epsilon[0], self.SA, self.TA),
                size,
            ),
            np.repeat(
                self.model_handle.d_st(gamma_delta_epsilon[1], self.SA, self.TA),
                size,
            ),
            np.repeat(
                self.model_handle.e_st(gamma_delta_epsilon[2], self.SA, self.TA),
                size,
            ),
        )
        return self.expected_cf.values.tolist(), predicted_ncf.tolist()

    def __train_st(self, param, model_param_index, model_handle):
        st0 = self.params_from_mp(self.init_mp)[model_param_index]
        return least_squares(
            BaseTrainer.st_residuals,
            st0,
            args=(param, self.SA, self.TA, model_handle),
        ).x

    def train_c_st(self, c):
        return self.__train_st(c, 0, self.model_handle.c_st)

    def train_d_st(self, d):
        return self.__train_st(d, 1, self.model_handle.d_st)

    def train_e_st(self, e):
        return self.__train_st(e, 2, self.model_handle.e_st)

    def train(self):
        c, d, e = NCFTrainer.split_c_d_e(self.train_cf())
        return self.train_c_st(c), self.train_d_st(d), self.train_e_st(e)


class RCFTrainer(CFTrainer):
    """Implements the trainer interface for estimating the resolution correction factor"""

    def __init__(self, df_resolution, model_handle, init_model_parameter):
        super().__init__(df_resolution, model_handle, init_model_parameter, KEYS.WIDTH)

    @property
    def cf_handle(self):
        return self.model_handle.rcf

    @property
    def st_handle(self):
        return self.model_handle.j_st

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.phi


class GCFTrainer(CFTrainer):
    """Implements the trainer interface for estimating the gaussian correction factor"""

    cf0 = (1, 1, 1)

    def __init__(self, df_ksize, model_handle, init_model_parameter):
        super().__init__(df_ksize, model_handle, init_model_parameter, KEYS.KSIZE)

    @property
    def cf_handle(self):
        return lambda k_size, l_m_o: self.model_handle.gcf(
            k_size, l_m_o[0], l_m_o[1], l_m_o[2]
        )

    @property
    def st_handle(self):
        raise NotImplementedError()

    @staticmethod
    def split_l_m_o(l_m_o):
        params = np.array(l_m_o)
        return params[:, 0], params[:, 1], params[:, 2]

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.lambda_, model_parameter.mu, model_parameter.omega

    def validate_cf(self, l_m_o):
        size = self.X.drop_duplicates().size
        l, m, o = GCFTrainer.split_l_m_o(l_m_o)
        predicted_gcf = self.model_handle.gcf(
            self.X.to_numpy(),
            np.repeat(l, size),
            np.repeat(m, size),
            np.repeat(o, size),
        )
        return self.expected_cf.values.tolist(), predicted_gcf.tolist()

    def validate_l_st(self, l, lambda_):  # noqa: E741
        return l, self.model_handle.l_st(lambda_, self.SA, self.TA)

    def validate_m_st(self, m, mu):
        return m, self.model_handle.m_st(mu, self.SA, self.TA)

    def validate_o_st(self, o, omega):
        return o, self.model_handle.o_st(omega, self.SA, self.TA)

    def validate(self, lambda_mu_omega):
        size = self.X.drop_duplicates().size
        predicted_gcf = self.model_handle.gcf(
            self.X.to_numpy(),
            np.repeat(
                self.model_handle.l_st(lambda_mu_omega[0], self.SA, self.TA),
                size,
            ),
            np.repeat(
                self.model_handle.m_st(lambda_mu_omega[1], self.SA, self.TA),
                size,
            ),
            np.repeat(
                self.model_handle.o_st(lambda_mu_omega[2], self.SA, self.TA),
                size,
            ),
        )
        return self.expected_cf.values.tolist(), predicted_gcf.tolist()

    def __train_st(self, param, model_param_index, model_handle):
        st0 = self.params_from_mp(self.init_mp)[model_param_index]
        return least_squares(
            BaseTrainer.st_residuals,
            st0,
            args=(param, self.SA, self.TA, model_handle),
        ).x

    def train_l_st(self, l):  # noqa: E741
        return self.__train_st(l, 0, self.model_handle.l_st)

    def train_m_st(self, m):
        return self.__train_st(m, 1, self.model_handle.m_st)

    def train_o_st(self, o):
        return self.__train_st(o, 2, self.model_handle.o_st)

    def train(self):
        l, m, o = GCFTrainer.split_l_m_o(self.train_cf())
        return self.train_l_st(l), self.train_m_st(m), self.train_o_st(o)


class SDCFTrainer(CFTrainer):
    """Implements the trainer interface for estimating the gaussian standard deviation correction factor"""

    cf0 = (1, 1, 1)

    def __init__(self, df_sigma, model_handle, init_model_parameter):
        super().__init__(df_sigma, model_handle, init_model_parameter, KEYS.SIGMA)

    @property
    def cf_handle(self):
        return lambda sigma, t_u_v: self.model_handle.sdcf(
            sigma, t_u_v[0], t_u_v[1], t_u_v[2]
        )

    @property
    def st_handle(self):
        raise NotImplementedError()

    @staticmethod
    def split_t_u_v(t_u_v):
        params = np.array(t_u_v)
        return params[:, 0], params[:, 1], params[:, 2]

    @staticmethod
    def params_from_mp(model_parameter):
        return model_parameter.tau, model_parameter.upsilon, model_parameter.nu

    def validate_cf(self, t_u_v):
        size = self.X.drop_duplicates().size
        t, u, v = SDCFTrainer.split_t_u_v(t_u_v)
        predicted_sdcf = self.model_handle.sdcf(
            self.X.to_numpy(),
            np.repeat(t, size),
            np.repeat(u, size),
            np.repeat(v, size),
        )
        return self.expected_cf.values.tolist(), predicted_sdcf.tolist()

    def validate_t_st(self, t, tau):
        return t, self.model_handle.t_st(tau, self.SA, self.TA)

    def validate_u_st(self, u, upsilon):
        return u, self.model_handle.u_st(upsilon, self.SA, self.TA)

    def validate_v_st(self, v, nu):
        return v, self.model_handle.v_st(nu, self.SA, self.TA)

    def validate(self, tau_upsilon_nu):
        size = self.X.drop_duplicates().size
        predicted_sdcf = self.model_handle.sdcf(
            self.X.to_numpy(),
            np.repeat(
                self.model_handle.t_st(tau_upsilon_nu[0], self.SA, self.TA),
                size,
            ),
            np.repeat(
                self.model_handle.u_st(tau_upsilon_nu[1], self.SA, self.TA),
                size,
            ),
            np.repeat(
                self.model_handle.v_st(tau_upsilon_nu[2], self.SA, self.TA),
                size,
            ),
        )
        return self.expected_cf.values.tolist(), predicted_sdcf.tolist()

    def __train_st(self, param, model_param_index, model_handle):
        st0 = self.params_from_mp(self.init_mp)[model_param_index]
        return least_squares(
            BaseTrainer.st_residuals,
            st0,
            args=(param, self.SA, self.TA, model_handle),
        ).x

    def train_t_st(self, t):
        return self.__train_st(t, 0, self.model_handle.t_st)

    def train_u_st(self, u):
        return self.__train_st(u, 1, self.model_handle.u_st)

    def train_v_st(self, v):
        return self.__train_st(v, 2, self.model_handle.v_st)

    def train(self):
        t, u, v = SDCFTrainer.split_t_u_v(self.train_cf())
        return self.train_t_st(t), self.train_u_st(u), self.train_v_st(v)


def run_loocv(df, evaluation_handle, bitrate_model_handle):
    """Run leave one out cross validation for a given trainer handle and bitrate model"""
    loo = LeaveOneOut()
    mad = []
    keys = df.index.drop_duplicates()
    for train_index, test_index in loo.split(keys):
        df_train = df.loc[keys[train_index]]
        df_test = df.loc[keys[test_index]]
        expected, predicted = evaluation_handle(df_train, df_test, bitrate_model_handle)
        mad.append(mean_absolute_error(expected, predicted))
    return statistics.mean(mad)
