"""Smoke test for sbr_bringup.

sbr_bringup ships launch files and parameters rather than importable code, so
this validates the hardware parameter file (and keeps the Python test runner
from erroring on an otherwise test-less package).
"""
import os

import yaml

_HERE = os.path.dirname(__file__)
_PARAMS = os.path.join(_HERE, os.pardir, 'config', 'sbr_params.yaml')


def test_params_declare_expected_nodes():
    with open(_PARAMS) as handle:
        params = yaml.safe_load(handle)
    for node in ('imu_node', 'motor_node', 'balance_controller_node'):
        assert node in params, f'missing params block for {node}'
        assert 'ros__parameters' in params[node]
