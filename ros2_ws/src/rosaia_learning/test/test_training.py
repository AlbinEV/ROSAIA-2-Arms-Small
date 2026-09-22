"""Integration test for session loading, MLP fitting, and validation."""

import json

import numpy as np
import pandas as pd

from rosaia_learning.dataset import STATE_COLUMNS
from rosaia_learning.train import train_model


def write_session(path, q_values):
    """Write one synthetic recorder-compatible session."""
    path.mkdir()
    q_values = np.asarray(q_values, dtype=np.float64)
    times = np.arange(len(q_values), dtype=np.int64) * 20_000_000
    rows = {
        'camera_time_ns': times,
        'arm': ['arm_1'] * len(q_values),
        'axis': np.zeros(len(q_values), dtype=np.int64),
        'q_operational': q_values,
        'command': np.zeros(len(q_values)),
        'adc_raw': np.full(len(q_values), 512),
        'current_ma': np.full(len(q_values), -(2**31)),
        'firmware_flags': np.full(len(q_values), 16),
    }
    states = np.column_stack([
        0.1 * q_values,
        np.sin(q_values / 20.0),
        0.2 * q_values + 1.0,
        np.cos(q_values / 30.0),
        -0.05 * q_values,
        0.002 * q_values * q_values,
    ])
    for index, column in enumerate(STATE_COLUMNS):
        rows[column] = states[:, index]
    pd.DataFrame(rows).to_csv(path / 'samples.csv', index=False)
    telemetry = {
        'receive_time_ns': times,
        'encoder_0_raw': -q_values,
        'encoder_1_raw': np.zeros(len(q_values)),
        'command_0': np.zeros(len(q_values)),
        'command_1': np.zeros(len(q_values)),
        'adc_0_raw': np.full(len(q_values), 512),
        'adc_1_raw': np.full(len(q_values), 512),
        'current_0_ma': np.full(len(q_values), -(2**31)),
        'current_1_ma': np.full(len(q_values), -(2**31)),
        'firmware_flags': np.full(len(q_values), 16),
    }
    pd.DataFrame(telemetry).to_csv(path / 'telemetry.csv', index=False)
    metadata = {
        'schema_version': 1,
        'arms': [
            {
                'name': 'arm_1',
                'state_topic': '/arm_1/aruco_state',
                'axis': 0,
                'encoder_sign': -1,
            }
        ],
    }
    (path / 'metadata.json').write_text(
        json.dumps(metadata), encoding='utf-8'
    )


def test_train_model_uses_held_out_session(tmp_path):
    """Training returns finite held-out metrics and an analytic model."""
    training = tmp_path / 'training'
    validation = tmp_path / 'validation'
    write_session(training, np.linspace(0.0, 100.0, 120))
    write_session(validation, np.linspace(2.0, 98.0, 60))
    model, metrics = train_model(
        [str(training)],
        [str(validation)],
        'arm_1',
        (12, 12),
        3,
        2000,
    )
    state, jacobian = model.predict_and_jacobian(50.0)
    assert np.isfinite(state).all()
    assert np.isfinite(jacobian).all()
    assert metrics['train_rows'] == 120
    assert metrics['validation_rows'] == 60
    assert metrics['validation_rmse_total'] < 0.2
