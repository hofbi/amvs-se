# CQP Preprocessing Filter Model

Preprocessor model for estimating the required preprocessing filter parameter from given encoding parameter.

## Bitrate Model

The bitrate models define the relationship between the filter parameter of a Gaussian low pass filter, the encoding parameter, and the bitrate.

Dataset generation in [../util/README.md](../util/README.md)

### Analytical Model

Individual training and execution of the analytical bitrate model in [cqp_analytic_model.ipynb](../notebook/cqp_analytic_model.ipynb)

Auto training and evaluation with `cqp_analytic_model_train.py`

```shell
# Environment Setup
python3 -m venv venv
source venv/bin/activate
pip3 install -r ../notebook/requirements.txt

# Usage
python3 cqp_analytic_model_train.py -h
```

### Machine Learning

Individual training and execution of the ML bitrate model in [cqp_ml_model.ipynb](../notebook/cqp_ml_model.ipynb)

Auto training and evaluation with `cqp_ml_model_train.py`

```shell
# Environment Setup
cd ../notebook
docker-compose build
docker-compose run -u $(id -u):$(id -g) keras-gpu-jupyter bash

# Usage
python3 cqp_ml_model_train.py -h
```

## Preprocessor Model

Usability experiment for the preprocessor model in [cqp_preprocessor_model_application.ipynb](../notebook/cqp_preprocessor_model_application.ipynb)

Usage as for [Machine Learning](#machine-learning)
