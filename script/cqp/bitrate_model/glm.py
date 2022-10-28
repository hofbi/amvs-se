"""
Use this for fast GLM.
Loop over all glm_models and set the current lambda to the bitrate model parameter you want to match

Sample:
for model in glm_models:
    BITRATE_MODEL.a_st = model
"""

import sys
from pathlib import Path

import numpy as np

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.model import ModelParameter
from cqp.bitrate_model.trainer import (
    GCFTrainer,
    NCFTrainer,
    RCFTrainer,
    RMaxTrainer,
    SCFTrainer,
    SDCFTrainer,
    TCFTrainer,
)

glm_models = [
    lambda param, SA, TA: param[0] * SA + param[1],
    lambda param, SA, TA: param[0] * TA + param[1],
    lambda param, SA, TA: param[0] * np.log10(SA) + param[1],
    lambda param, SA, TA: param[0] * np.log10(TA) + param[1],
    lambda param, SA, TA: param[0] * (SA * TA) + param[1],
    lambda param, SA, TA: param[0] * (SA / TA) + param[1],
    lambda param, SA, TA: param[0] * (TA / SA) + param[1],
    lambda param, SA, TA: param[0] * np.log10(SA * TA) + param[1],
    lambda param, SA, TA: param[0] * np.log10(SA / TA) + param[1],
]


def r_max_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate rmax performance using loocv"""
    rho_loo = RMaxTrainer(df_train, bitrate_model_handle, ModelParameter()).train()
    return RMaxTrainer(df_test, bitrate_model_handle, ModelParameter()).validate(
        rho_loo
    )


def st_loocv(df_train, df_test, bitrate_model_handle, trainer_handle):
    """Validate spatio-temporal parameter performance using loocv"""
    trainer_loo = trainer_handle(df_train, bitrate_model_handle, ModelParameter())
    p_loo = trainer_loo.train_cf()
    p_expected = trainer_handle(
        df_test, bitrate_model_handle, ModelParameter()
    ).train_cf()
    return trainer_handle(df_test, bitrate_model_handle, ModelParameter()).validate_st(
        p_expected, trainer_loo.train_st(p_loo)
    )


def a_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of a using loocv"""
    return st_loocv(df_train, df_test, bitrate_model_handle, SCFTrainer)


def b_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of b using loocv"""
    return st_loocv(df_train, df_test, bitrate_model_handle, TCFTrainer)


def c_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of c using loocv"""
    trainer_loo = NCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    c_loo = NCFTrainer.split_c_d_e(trainer_loo.train_cf())[0]
    c_expected = NCFTrainer.split_c_d_e(
        NCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[0]
    return NCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_c_st(
        c_expected, trainer_loo.train_c_st(c_loo)
    )


def d_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of d using loocv"""
    trainer_loo = NCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    d_loo = NCFTrainer.split_c_d_e(trainer_loo.train_cf())[1]
    d_expected = NCFTrainer.split_c_d_e(
        NCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[1]
    return NCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_d_st(
        d_expected, trainer_loo.train_d_st(d_loo)
    )


def e_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of e using loocv"""
    trainer_loo = NCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    e_loo = NCFTrainer.split_c_d_e(trainer_loo.train_cf())[2]
    e_expected = NCFTrainer.split_c_d_e(
        NCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[2]
    return NCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_e_st(
        e_expected, trainer_loo.train_e_st(e_loo)
    )


def j_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of j using loocv"""
    return st_loocv(df_train, df_test, bitrate_model_handle, RCFTrainer)


def l_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of l using loocv"""
    trainer_loo = GCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    l_loo = GCFTrainer.split_l_m_o(trainer_loo.train_cf())[0]
    l_expected = GCFTrainer.split_l_m_o(
        GCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[0]
    return GCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_l_st(
        l_expected, trainer_loo.train_l_st(l_loo)
    )


def m_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of m using loocv"""
    trainer_loo = GCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    m_loo = GCFTrainer.split_l_m_o(trainer_loo.train_cf())[1]
    m_expected = GCFTrainer.split_l_m_o(
        GCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[1]
    return GCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_m_st(
        m_expected, trainer_loo.train_m_st(m_loo)
    )


def o_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of o using loocv"""
    trainer_loo = GCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    o_loo = GCFTrainer.split_l_m_o(trainer_loo.train_cf())[2]
    o_expected = GCFTrainer.split_l_m_o(
        GCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[2]
    return GCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_o_st(
        o_expected, trainer_loo.train_o_st(o_loo)
    )


def t_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of t using loocv"""
    trainer_loo = SDCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    t_loo = SDCFTrainer.split_t_u_v(trainer_loo.train_cf())[0]
    t_expected = SDCFTrainer.split_t_u_v(
        SDCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[0]
    return SDCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_t_st(
        t_expected, trainer_loo.train_t_st(t_loo)
    )


def u_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of u using loocv"""
    trainer_loo = SDCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    u_loo = SDCFTrainer.split_t_u_v(trainer_loo.train_cf())[1]
    u_expected = SDCFTrainer.split_t_u_v(
        SDCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[1]
    return SDCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_u_st(
        u_expected, trainer_loo.train_u_st(u_loo)
    )


def v_st_loocv_handle(df_train, df_test, bitrate_model_handle):
    """Validate spatio-temporal parameter performance of v using loocv"""
    trainer_loo = SDCFTrainer(df_train, bitrate_model_handle, ModelParameter())
    v_loo = SDCFTrainer.split_t_u_v(trainer_loo.train_cf())[2]
    v_expected = SDCFTrainer.split_t_u_v(
        SDCFTrainer(df_test, bitrate_model_handle, ModelParameter()).train_cf()
    )[2]
    return SDCFTrainer(df_test, bitrate_model_handle, ModelParameter()).validate_v_st(
        v_expected, trainer_loo.train_v_st(v_loo)
    )
