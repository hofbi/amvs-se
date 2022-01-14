"""Train CQP neural network based bitrate model"""

import argparse
import sys
from pathlib import Path
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, optimizers, Sequential, callbacks

from sklearn.preprocessing import StandardScaler

try:
    sys.path.append(str(Path().cwd().parent))
except IndexError:
    pass

from cqp.bitrate_model.util import (
    DataFrameWrapper,
    MaxBitrateParameter,
    parse_resolution,
    read_df_by_keys,
    KEYS,
    get_rmse,
    get_pc,
    print_rmse_normalized_rmse_and_pc,
)


MODEL_CHECKPOINT = "best.h5"


def create_model(model: Sequential, learning_rate: float):
    """Create the tensorflow model"""
    opt = optimizers.Adam(learning_rate=learning_rate)
    model.compile(optimizer=opt, loss="mse")
    return model


def train(model, train_db, val_db, epochs):
    """Train the ML model for a given training and validation set"""
    reduce_lr = callbacks.ReduceLROnPlateau(monitor="loss", factor=0.5, patience=10)
    model_checkpoint_callback = callbacks.ModelCheckpoint(
        filepath=MODEL_CHECKPOINT, monitor="loss", save_best_only=True
    )
    model.fit(
        train_db,
        epochs=epochs,
        validation_data=val_db,
        validation_freq=1,
        callbacks=[reduce_lr, model_checkpoint_callback],
    )
    print("Finished training!")


def validate(model, dfw, input_scaler, output_scaler, data_name="test"):
    """Validate the trained ML model on the test set"""
    in_test = input_scaler.transform(dfw.get_ml_input())
    in_test = tf.convert_to_tensor(in_test, dtype=tf.float32)
    out_test = dfw.get_ml_output()

    print(f"Evaluating on {data_name} data...")
    results = model.evaluate(in_test, output_scaler.transform(out_test))
    print(f"{data_name} loss, accuracy: ", results)

    print(f"Predicting on {data_name} data...")
    output = model.predict(in_test)
    out_prediction = output_scaler.inverse_transform(output)

    max_bitrates = dfw.get_ml_max_output()
    print_rmse_normalized_rmse_and_pc(
        "Bitrate average",
        get_rmse(out_test, out_prediction) * 0.001,
        get_rmse(
            out_test.transpose() / max_bitrates,
            out_prediction.transpose() / max_bitrates,
        ),
        get_pc(out_test.transpose(), out_prediction.transpose()),
    )


def create_db(data_in, data_out, input_scaler, output_scaler, batch_size):
    """Create a normalied dataset containing input and output data"""
    data_in = input_scaler.transform(data_in)
    data_out = output_scaler.transform(data_out)

    data_in = tf.convert_to_tensor(data_in, dtype=tf.float32)
    data_out = tf.convert_to_tensor(data_out, dtype=tf.float32)

    db = tf.data.Dataset.from_tensor_slices((data_in, data_out))
    return db.batch(batch_size)


def save_model(model, batch_size, learning_rate, epochs):
    """Save the model as h5 file"""
    learning_rate_str = str(learning_rate).replace(".", "")
    model_name = f"cqp-b{batch_size}-l{learning_rate_str}-e{epochs}.h5"
    print(f"Saving model to {model_name}")
    model.save(model_name)
    model.summary()
    return model_name


def parse_arguments():
    """Parse command line arguments"""

    class ModelDictAction(argparse.Action):
        """Map the model architecture to a simple text that can used with the CLI"""

        VALUE_DICT = {
            "x264": Sequential(
                [
                    layers.Dense(180, activation=tf.nn.relu),
                    layers.Dense(120, activation=tf.nn.relu),
                    layers.Dense(60, activation=tf.nn.relu),
                    layers.Dense(30, activation=tf.nn.relu),
                    layers.Dense(1),
                ]
            ),
            "nhevc": Sequential(
                [
                    layers.Dense(180, activation=tf.nn.relu),
                    layers.Dropout(0.75),
                    layers.Dense(120, activation=tf.nn.relu),
                    layers.Dropout(0.75),
                    layers.Dense(60, activation=tf.nn.relu),
                    layers.Dropout(0.75),
                    layers.Dense(30, activation=tf.nn.relu),
                    layers.Dropout(0.75),
                    layers.Dense(1),
                ]
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
        "-b",
        "--batch-size",
        type=int,
        default=256,
        help="Model Batch Size",
    )
    parser.add_argument(
        "-l",
        "--learning-rate",
        type=float,
        default=0.0001,
        help="Model learning rate",
    )
    parser.add_argument(
        "-e",
        "--epochs",
        type=int,
        default=200,
        help="Number of epochs to train the model",
    )
    parser.add_argument(
        "-r",
        "--resolution",
        type=str,
        default="352x288",
        help="Maximum resolution of the videos",
    )
    return parser.parse_args()


def main():
    """main"""
    args = parse_arguments()
    data_files = list(Path(args.data_path).glob("*.csv"))

    width, height = parse_resolution(args.resolution)
    max_parameter = MaxBitrateParameter(w_max=width, h_max=height)

    dfw = DataFrameWrapper(
        read_df_by_keys(data_files, KEYS.TRAINING_SET), max_parameter
    )
    dfw_val = DataFrameWrapper(
        read_df_by_keys(data_files, KEYS.VALIDATION_SET), max_parameter
    )

    in_train = dfw.get_ml_input()
    in_val = dfw_val.get_ml_input()

    out_train = dfw.get_ml_output()
    out_val = dfw_val.get_ml_output()

    input_scaler = StandardScaler().fit(np.concatenate([in_train, in_val], axis=0))
    output_scaler = StandardScaler().fit(np.concatenate([out_train, out_val], axis=0))

    train_db = create_db(
        in_train, out_train, input_scaler, output_scaler, args.batch_size
    )
    val_db = create_db(in_val, out_val, input_scaler, output_scaler, args.batch_size)

    model = create_model(args.model, args.learning_rate)

    train(model, train_db, val_db, args.epochs)

    model = tf.keras.models.load_model(MODEL_CHECKPOINT)

    save_model(model, args.batch_size, args.learning_rate, args.epochs)
    validate(model, dfw, input_scaler, output_scaler, "training")
    validate(model, dfw_val, input_scaler, output_scaler, "validation")

    dfw_test = DataFrameWrapper(
        read_df_by_keys(data_files, KEYS.TEST_SET), max_parameter
    )
    validate(model, dfw_test, input_scaler, output_scaler)


if __name__ == "__main__":
    main()
