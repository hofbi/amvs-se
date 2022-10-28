"""Analytic bitrate model"""

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import numpy as np

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.util import MaxBitrateParameter, parse_resolution


@dataclass
class ModelParameter:
    """Default values for x264 CIF data"""

    rho: List[float] = field(
        default_factory=lambda: [38.0332, -61.2067, 0.3884, 1.8015e03]
    )  # kbit/s
    alpha: List[float] = field(
        default_factory=lambda: [-4.9626e-03, -7.0579e-05, 3.5886]
    )
    beta: List[float] = field(default_factory=lambda: [1])
    gamma: List[float] = field(default_factory=lambda: [-0.1824, -0.0034, 1.0328])
    delta: List[float] = field(default_factory=lambda: [0.1011, 1.0240])
    epsilon: List[float] = field(default_factory=lambda: [0.1813, 0.0034, -0.0328])
    phi: List[float] = field(default_factory=lambda: [3.4594e-05, 1.5879])
    lambda_: List[float] = field(default_factory=lambda: [-9.2718e-05, 0.1759])
    mu: List[float] = field(default_factory=lambda: [15.7108])
    omega: List[float] = field(default_factory=lambda: [0.8342])
    tau: List[float] = field(default_factory=lambda: [0.0454, 0.1593])
    upsilon: List[float] = field(default_factory=lambda: [1.5681e-04, 2.6035])
    nu: List[float] = field(default_factory=lambda: [-0.0454, 0.8407])


class BitrateModel:
    """Analytical bitrate model"""

    def __init__(self, model_param: ModelParameter):
        self.__mp = model_param

    @property
    def _mp(self):
        return self.__mp

    @staticmethod
    def r_max(rho, SA, TA):
        return rho[0] * SA + rho[1] * (SA / TA) + rho[2] * SA * TA + rho[3]

    @staticmethod
    def scf(qp, a):
        return pow(qp / MaxBitrateParameter().qp_min, -a)

    @staticmethod
    def a_st(alpha, SA, TA):
        return alpha[0] * SA + alpha[1] * SA * TA + alpha[2]

    @staticmethod
    def tcf(rate, b):
        return pow(rate / MaxBitrateParameter().f_max, b)

    @staticmethod
    def b_st(beta, SA, TA):
        # 0 * SA is only there to have the correct vector len
        return 0 * SA + beta[0]

    @staticmethod
    def ncf(gop, c, d, e):
        return c * pow(MaxBitrateParameter().gop_min / gop, d) + e

    @staticmethod
    def c_st(gamma, SA, TA):
        return gamma[0] * np.log10(TA) + gamma[1] * TA + gamma[2]

    @staticmethod
    def d_st(delta, SA, TA):
        return (TA / SA) * delta[0] + delta[1]

    @staticmethod
    def e_st(epsilon, SA, TA):
        return epsilon[0] * np.log10(TA) + epsilon[1] * TA + epsilon[2]

    @staticmethod
    def rcf(width, j):
        return pow(width / MaxBitrateParameter().w_max, j)

    @staticmethod
    def j_st(phi, SA, TA):
        return phi[0] * SA * TA + phi[1]

    @staticmethod
    def gcf(k_size, l, m, o):  # noqa: E741
        return l * pow(MaxBitrateParameter().k_size_min / k_size, m) + o

    @staticmethod
    def l_st(lambda_, SA, TA):
        return lambda_[0] * SA + lambda_[1]

    @staticmethod
    def m_st(mu, SA, TA):
        return 0 * SA + mu[0]

    @staticmethod
    def o_st(omega, SA, TA):
        return 0 * SA + omega[0]

    @staticmethod
    def sdcf(sigma, t, u, v):
        return t * pow(MaxBitrateParameter().sigma_min / sigma, u) + v

    @staticmethod
    def t_st(tau, SA, TA):
        return tau[0] * np.log10(SA * TA) + tau[1]

    @staticmethod
    def u_st(upsilon, SA, TA):
        return upsilon[0] * SA * TA + upsilon[1]

    @staticmethod
    def v_st(nu, SA, TA):
        return nu[0] * np.log10(SA * TA) + nu[1]

    def evaluate(self, SA, TA, QP, f, n, r, k_size, sigma):
        R_max = self.r_max(self._mp.rho, SA, TA)

        SCF = self.scf(QP, self.a_st(self._mp.alpha, SA, TA))
        TCF = self.tcf(f, self.b_st(self._mp.beta, SA, TA))
        NCF = self.ncf(
            n,
            self.c_st(self._mp.gamma, SA, TA),
            self.d_st(self._mp.delta, SA, TA),
            self.e_st(self._mp.epsilon, SA, TA),
        )
        MCF = 1  # P-frames only
        RCF = self.rcf(parse_resolution(r)[0], self.j_st(self._mp.phi, SA, TA))
        GCF = self.gcf(
            k_size,
            self.l_st(self._mp.lambda_, SA, TA),
            self.m_st(self._mp.mu, SA, TA),
            self.o_st(self._mp.omega, SA, TA),
        )
        SDCF = (
            self.sdcf(
                sigma,
                self.t_st(self._mp.tau, SA, TA),
                self.u_st(self._mp.upsilon, SA, TA),
                self.v_st(self._mp.nu, SA, TA),
            )
            if k_size > 1
            else 1
        )

        return R_max * SCF * TCF * NCF * MCF * RCF * GCF * SDCF


class LottermannBitrateModel1(BitrateModel):
    """Lottermann Bitrate Model supporting SCF and TCF"""

    @staticmethod
    def r_max(rho, SA, TA):
        return rho[0] * TA * SA + rho[1]

    @staticmethod
    def a_st(alpha, SA, TA):
        return alpha[0] * np.log10(SA) + alpha[1] * SA * TA + alpha[2]

    @staticmethod
    def b_st(beta, SA, TA):
        return beta[0] * np.log10(TA * SA) + beta[1]

    @staticmethod
    def ncf(gop, c, d, e):
        return 0 * gop + 1  # Not supported correction factor

    @staticmethod
    def rcf(width, j):
        return 0 * width + 1  # Not supported correction factor

    @staticmethod
    def gcf(k_size, l, m, o):  # noqa: E741
        return 0 * k_size + 1  # Not supported correction factor

    @staticmethod
    def sdcf(sigma, t, u, v):
        return 0 * sigma + 1  # Not supported correction factor


class LottermannBitrateModel2(BitrateModel):
    """Lottermann Bitrate Model supporting SCF, TCF, and NCF"""

    @staticmethod
    def r_max(rho, SA, TA):
        return rho[0] * SA * TA + rho[1] * SA + rho[2]

    @staticmethod
    def a_st(alpha, SA, TA):
        return alpha[0] * SA + alpha[1] * np.log10(SA) + alpha[2]

    @staticmethod
    def c_st(gamma, SA, TA):
        return gamma[0] * TA + gamma[1]

    @staticmethod
    def d_st(delta, SA, TA):
        # 0 * SA is only there to have the correct vector len
        return 0 * SA + delta[0]

    @staticmethod
    def e_st(epsilon, SA, TA):
        return epsilon[0] * SA + epsilon[1] * (TA / SA) + epsilon[2]

    @staticmethod
    def rcf(width, j):
        return 0 * width + 1  # Not supported correction factor

    @staticmethod
    def gcf(k_size, l, m, o):  # noqa: E741
        return 0 * k_size + 1  # Not supported correction factor

    @staticmethod
    def sdcf(sigma, t, u, v):
        return 0 * sigma + 1  # Not supported correction factor


class MaBitrateModel(BitrateModel):
    """
    Ma Bitrate Model supporting SCF and TCF

    Ma et al. use motion vector information which are not available in
    our usecase. Therefore we estimate the MV information from the SA
    and TA values by mapping those values to the known values of Ma's
    paper.
    """

    @staticmethod
    def __fd_from_ta(TA):
        return TA * 0.71

    @staticmethod
    def __mvm_from_ta(TA):
        return TA * 0.33

    @staticmethod
    def __nmv_mda_from_ta(TA):
        return TA * 0.64

    @staticmethod
    def r_max(rho, SA, TA):
        frame_diff = MaBitrateModel.__fd_from_ta(TA)
        motion_vec_magnitude = MaBitrateModel.__mvm_from_ta(TA)
        nmv_mda = MaBitrateModel.__nmv_mda_from_ta(TA)
        return (
            rho[0]
            + rho[1] * frame_diff
            + rho[2] * motion_vec_magnitude
            + rho[3] * nmv_mda
        )

    @staticmethod
    def a_st(alpha, SA, TA):
        frame_diff = MaBitrateModel.__fd_from_ta(TA)
        motion_vec_magnitude = MaBitrateModel.__mvm_from_ta(TA)
        nmv_mda = MaBitrateModel.__nmv_mda_from_ta(TA)
        return (
            alpha[0]
            + alpha[1] * frame_diff
            + alpha[2] * motion_vec_magnitude
            + alpha[3] * nmv_mda
        )

    @staticmethod
    def b_st(beta, SA, TA):
        frame_diff = MaBitrateModel.__fd_from_ta(TA)
        motion_vec_magnitude = MaBitrateModel.__mvm_from_ta(TA)
        nmv_mda = MaBitrateModel.__nmv_mda_from_ta(TA)
        return (
            beta[0]
            + beta[1] * frame_diff
            + beta[2] * motion_vec_magnitude
            + beta[3] * nmv_mda
        )

    @staticmethod
    def ncf(gop, c, d, e):
        return 0 * gop + 1  # Not supported correction factor

    @staticmethod
    def rcf(width, j):
        return 0 * width + 1  # Not supported correction factor

    @staticmethod
    def gcf(k_size, l, m, o):  # noqa: E741
        return 0 * k_size + 1  # Not supported correction factor

    @staticmethod
    def sdcf(sigma, t, u, v):
        return 0 * sigma + 1  # Not supported correction factor
