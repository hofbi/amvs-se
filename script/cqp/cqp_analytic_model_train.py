"""Train parameter for CQP analytic bitrate model"""

import argparse
import sys
import statistics
from pathlib import Path
import numpy as np
import pandas as pd

try:
    sys.path.append(str(Path(__file__).parent.parent))
except IndexError:
    pass

from cqp.bitrate_model.util import (
    KEYS,
    DataFrameWrapper,
    read_df_by_keys,
    get_rmse,
    get_pc,
    print_rmse_and_pc,
    print_rmse_normalized_rmse_and_pc,
)


from cqp.bitrate_model.model import (
    ModelParameter,
    BitrateModel,
    LottermannBitrateModel1,
    LottermannBitrateModel2,
    MaBitrateModel,
)
from cqp.bitrate_model.trainer import (
    RMaxTrainer,
    SCFTrainer,
    TCFTrainer,
    NCFTrainer,
    RCFTrainer,
    GCFTrainer,
    SDCFTrainer,
)


def train_correction_factor(
    name, df, trainer_handle, model_handle, init_mode_parameter
):
    """Train a correction factor with a given bitrate model and trainer handle"""
    trainer = trainer_handle(df, model_handle, init_mode_parameter)
    param_min = trainer.train()
    expected_cf, predicted_cf = trainer.validate(param_min)
    print_rmse_and_pc(
        f"{name} training results",
        get_rmse(expected_cf, predicted_cf),
        get_pc(expected_cf, predicted_cf),
    )
    return param_min


def train(dfw, model_handle, init_model_parameter):
    """
    Train all supported correction factors
    """
    print("Starting training...")
    rho_min = train_correction_factor(
        "Rmax", dfw.include(), RMaxTrainer, model_handle, init_model_parameter
    )
    alpha_min = train_correction_factor(
        "SCF",
        dfw.include(variable_qp=True),
        SCFTrainer,
        model_handle,
        init_model_parameter,
    )
    beta_min = train_correction_factor(
        "TCF",
        dfw.include(variable_rate=True),
        TCFTrainer,
        model_handle,
        init_model_parameter,
    )
    gop_param_min = train_correction_factor(
        "NCF",
        dfw.include(variable_gop=True),
        NCFTrainer,
        model_handle,
        init_model_parameter,
    )
    phi_min = train_correction_factor(
        "RCF",
        dfw.include(variable_res=True),
        RCFTrainer,
        model_handle,
        init_model_parameter,
    )
    gauss_param_min = train_correction_factor(
        "GCF",
        dfw.include(variable_k_size=True),
        GCFTrainer,
        model_handle,
        init_model_parameter,
    )
    df_gauss = dfw.include(variable_k_size=True, variable_sigma=True)
    sigma_param_min = train_correction_factor(
        "SSCF",
        df_gauss.loc[df_gauss[KEYS.KSIZE] == 3],
        SDCFTrainer,
        model_handle,
        init_model_parameter,
    )

    return ModelParameter(
        rho=rho_min,
        alpha=alpha_min,
        beta=beta_min,
        gamma=gop_param_min[0],
        delta=gop_param_min[1],
        epsilon=gop_param_min[2],
        phi=phi_min,
        lambda_=gauss_param_min[0],
        mu=gauss_param_min[1],
        omega=gauss_param_min[2],
        tau=sigma_param_min[0],
        upsilon=sigma_param_min[1],
        nu=sigma_param_min[2],
    )


def validate_correction_factor(name, df, trainer_handle, model_parameter, model_handle):
    """Validate a correction factor with a given bitrate model and trainer handle"""
    trainer = trainer_handle(df, model_handle, model_parameter)
    params = trainer_handle.params_from_mp(model_parameter)
    expected_cf, predicted_cf = trainer.validate(params)
    print_rmse_and_pc(
        f"{name} validation results",
        get_rmse(expected_cf, predicted_cf),
        get_pc(expected_cf, predicted_cf),
    )


def validate_bitrates_for_video(df, model, video_key):
    """Validate bitrate estimation for a given bitrate model"""
    video_data = df.loc[video_key]
    bitrate_measured = video_data[KEYS.BITRATE].to_numpy() * 0.001  # Convert to kBit/s
    bitrate_estimated = np.array(
        [
            model.evaluate(sa, ta, qp, f, n, r, k_size, sigma)
            for sa, ta, qp, f, n, r, k_size, sigma in zip(
                video_data[KEYS.SA].tolist(),
                video_data[KEYS.TA].tolist(),
                video_data[KEYS.QP].tolist(),
                video_data[KEYS.RATE].tolist(),
                video_data[KEYS.GOP].tolist(),
                video_data[KEYS.RES].tolist(),
                video_data[KEYS.KSIZE].tolist(),
                video_data[KEYS.SIGMA].tolist(),
            )
        ]
    )
    return bitrate_measured, bitrate_estimated


def get_df_for_evaluation(dfw, model_handle, full_evaluation):
    """Compose the dataframe for the model evaluation considering supported correction factors"""
    variable_gop = model_handle.ncf(1, 1, 1, 1) != 1
    variable_res = model_handle.rcf(1, 1) != 1
    variable_k_size = model_handle.gcf(2, 1, 1, 1) != 1
    variable_sigma = model_handle.sdcf(1, 1, 1, 1) != 1
    return (
        dfw.df
        if full_evaluation
        or all([variable_gop, variable_res, variable_k_size, variable_sigma])
        else dfw.include(
            variable_qp=True,
            variable_rate=True,
            variable_gop=variable_gop,
            variable_res=variable_res,
            variable_k_size=variable_k_size,
            variable_sigma=variable_sigma,
        )
    )


def validate(dfw, model_parameter, model_handle, full_evaluation):
    """
    Validate all supported correction factors
    """
    video_keys = dfw.video_keys
    bitrate_model = model_handle(model_parameter)
    rmses = []
    nrmses = []
    pcs = []

    for key in video_keys.tolist():
        bitrate_measured, bitrate_estimated = validate_bitrates_for_video(
            get_df_for_evaluation(dfw, model_handle, full_evaluation),
            bitrate_model,
            key,
        )
        rmses.append(get_rmse(bitrate_measured, bitrate_estimated))
        nrmses.append(rmses[-1] / max(bitrate_measured))
        pcs.append(get_pc(bitrate_measured.T, bitrate_estimated.T))
        print_rmse_normalized_rmse_and_pc(
            f"Bitrate validation {key}", rmses[-1], nrmses[-1], pcs[-1]
        )

    validate_correction_factor(
        "Rmax", dfw.include(), RMaxTrainer, model_parameter, model_handle
    )
    validate_correction_factor(
        "SCF",
        dfw.include(variable_qp=True),
        SCFTrainer,
        model_parameter,
        model_handle,
    )
    validate_correction_factor(
        "TCF",
        dfw.include(variable_rate=True),
        TCFTrainer,
        model_parameter,
        model_handle,
    )
    validate_correction_factor(
        "NCF",
        dfw.include(variable_gop=True),
        NCFTrainer,
        model_parameter,
        model_handle,
    )
    validate_correction_factor(
        "RCF",
        dfw.include(variable_res=True),
        RCFTrainer,
        model_parameter,
        model_handle,
    )
    validate_correction_factor(
        "GCF",
        dfw.include(variable_k_size=True),
        GCFTrainer,
        model_parameter,
        model_handle,
    )
    df_gauss = dfw.include(variable_k_size=True, variable_sigma=True)
    validate_correction_factor(
        "SDCF",
        df_gauss.loc[df_gauss[KEYS.KSIZE] == 3],
        SDCFTrainer,
        model_parameter,
        model_handle,
    )
    print_rmse_normalized_rmse_and_pc(
        "Bitrate validation average",
        statistics.mean(rmses),
        statistics.mean(nrmses),
        statistics.mean(pcs),
    )


def parse_arguments():
    """Parse command line arguments"""

    class ModelDictAction(argparse.Action):
        """Map the bitrate model and parameter to a simple text that can used with the CLI"""

        VALUE_DICT = {
            "x264": (BitrateModel, ModelParameter()),
            "lottermann": (
                LottermannBitrateModel1,
                ModelParameter(
                    alpha=[2.0129, -0.0004, -4.6158],
                    beta=[0.1334, -0.3072],
                    rho=[0.8149, 139.4017],
                ),
            ),
            "lottermann-gop": (
                LottermannBitrateModel2,
                ModelParameter(
                    alpha=[-0.0492, 4.8648, -14.3861],
                    beta=[1],
                    gamma=[-0.0108, 0.9473],
                    delta=[1],
                    epsilon=[0.0011, 1.0200, -0.0598],
                    rho=[1.3190, 31.7481, -210.6505],
                ),
            ),
            "ma": (
                MaBitrateModel,
                ModelParameter(
                    alpha=[1.1406, -0.0330, -0.0611, 0.1408],
                    beta=[0.4462, 0.0112, 0.0680, -0.0667],
                    rho=[67.73, 49.45, 281.7, -245.6],
                ),
            ),
        }

        def __call__(self, arg_parser, namespace, values, option_string=None):

            setattr(namespace, self.dest, self.VALUE_DICT.get(values))

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("data_path", type=str, help="Path to the input files")
    parser.add_argument(
        "-m",
        "--model",
        action=ModelDictAction,
        choices=ModelDictAction.VALUE_DICT.keys(),
        default=ModelDictAction.VALUE_DICT["x264"],
        help="Select the model to be trained",
    )
    parser.add_argument(
        "-v",
        "--validate",
        action="store_true",
        help="Validate the new trained model parameter on the test set",
    )
    parser.add_argument(
        "-f",
        "--full",
        action="store_true",
        help="""Validate with the full dataset. Without this only data for"
        supported correction factors are considered for the evaluation""",
    )
    return parser.parse_args()


def main():
    """main"""
    args = parse_arguments()
    data_files = list(Path(args.data_path).glob("*.csv"))
    model_handle, default_model_parameter = args.model

    dfw_train = DataFrameWrapper(read_df_by_keys(data_files, KEYS.TRAINING_SET))
    model_parameter = train(dfw_train, model_handle, default_model_parameter)
    print()
    print(f"Old model parameter: {default_model_parameter}")
    print(f"New model parameter: {model_parameter}")
    print()

    if args.validate:
        print("Starting validation on training set...")
        validate(dfw_train, model_parameter, model_handle, args.full)
        print("\nStarting validation on validation set...")
        dfw_val = DataFrameWrapper(read_df_by_keys(data_files, KEYS.TEST_SET))
        print("\nValidation results for default parameter:")
        validate(dfw_val, default_model_parameter, model_handle, args.full)
        print("\nValidation results for trained parameter:")
        validate(dfw_val, model_parameter, model_handle, args.full)

        dfw_all = DataFrameWrapper(pd.concat([dfw_train.df, dfw_val.df]))
        print("\nAll video validation results for default parameter:")
        validate(dfw_all, default_model_parameter, model_handle, args.full)
        print("\nAll video validation results for trained parameter:")
        validate(dfw_all, model_parameter, model_handle, args.full)


if __name__ == "__main__":
    main()
