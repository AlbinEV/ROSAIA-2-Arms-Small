"""Command-line MLP training with trajectory-level validation sessions."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import warnings

import numpy as np
from sklearn.exceptions import ConvergenceWarning
from sklearn.neural_network import MLPRegressor

from .dataset import arrays, load_sessions
from .model import NumpyMlpModel


def _positive_widths(value: str) -> tuple[int, ...]:
    widths = tuple(int(item) for item in value.split(',') if item.strip())
    if not widths or any(item <= 0 for item in widths):
        raise argparse.ArgumentTypeError('hidden widths must be positive')
    return widths


def _safe_scale(values: np.ndarray) -> np.ndarray:
    scale = np.std(values, axis=0)
    return np.where(scale < 1e-12, 1.0, scale)


def train_model(
    train_sessions: list[str],
    validation_sessions: list[str],
    arm_name: str,
    hidden_widths: tuple[int, ...],
    random_seed: int,
    maximum_iterations: int,
    camera_to_motor_offset_ms: float = 0.0,
) -> tuple[NumpyMlpModel, dict]:
    """Fit an MLP and evaluate it only on held-out complete sessions."""
    train_data = load_sessions(
        train_sessions, arm_name, camera_to_motor_offset_ms
    )
    validation_data = load_sessions(
        validation_sessions, arm_name, camera_to_motor_offset_ms
    )
    q_train, x_train = arrays(train_data)
    q_validation, x_validation = arrays(validation_data)

    q_mean = float(np.mean(q_train))
    q_scale = float(_safe_scale(q_train)[0])
    x_mean = np.mean(x_train, axis=0)
    x_scale = _safe_scale(x_train)
    q_scaled = (q_train - q_mean) / q_scale
    x_scaled = (x_train - x_mean) / x_scale

    estimator = MLPRegressor(
        hidden_layer_sizes=hidden_widths,
        activation='tanh',
        solver='adam',
        alpha=1e-5,
        batch_size=min(256, len(q_train)),
        learning_rate_init=1e-3,
        max_iter=maximum_iterations,
        shuffle=True,
        random_state=random_seed,
        early_stopping=False,
        tol=1e-7,
        n_iter_no_change=100,
    )
    with warnings.catch_warnings(record=True) as fit_warnings:
        warnings.simplefilter('always', ConvergenceWarning)
        estimator.fit(q_scaled, x_scaled)
    converged = not any(
        issubclass(item.category, ConvergenceWarning)
        for item in fit_warnings
    )
    model = NumpyMlpModel(
        q_mean=q_mean,
        q_scale=q_scale,
        x_mean=x_mean,
        x_scale=x_scale,
        weights=tuple(estimator.coefs_),
        biases=tuple(estimator.intercepts_),
        q_min=float(np.min(q_train)),
        q_max=float(np.max(q_train)),
    )
    predictions = np.vstack([
        model.predict_and_jacobian(value)[0]
        for value in q_validation[:, 0]
    ])
    errors = predictions - x_validation
    rmse_per_component = np.sqrt(np.mean(errors * errors, axis=0))
    metrics = {
        'arm': arm_name,
        'created_at': datetime.now().astimezone().isoformat(),
        'train_sessions': [str(Path(value)) for value in train_sessions],
        'validation_sessions': [
            str(Path(value)) for value in validation_sessions
        ],
        'train_rows': int(len(train_data)),
        'validation_rows': int(len(validation_data)),
        'hidden_widths': list(hidden_widths),
        'random_seed': random_seed,
        'camera_to_motor_offset_ms': camera_to_motor_offset_ms,
        'iterations': int(estimator.n_iter_),
        'converged': converged,
        'loss': float(estimator.loss_),
        'validation_rmse_components': rmse_per_component.tolist(),
        'validation_rmse_total': float(np.sqrt(np.mean(errors * errors))),
    }
    return model, metrics


def main(args=None) -> None:
    """Train and export one per-arm scalar-input MLP."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train-session', action='append', required=True)
    parser.add_argument('--validation-session', action='append', required=True)
    parser.add_argument('--arm', choices=('arm_1', 'arm_2'), required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--hidden', type=_positive_widths, default=(32, 32))
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--max-iterations', type=int, default=2000)
    parser.add_argument('--camera-to-motor-offset-ms', type=float, default=0.0)
    options = parser.parse_args(args)
    model, metrics = train_model(
        options.train_session,
        options.validation_session,
        options.arm,
        options.hidden,
        options.seed,
        options.max_iterations,
        options.camera_to_motor_offset_ms,
    )
    model.save(options.output, metadata={'training': metrics})
    print(json.dumps(metrics, indent=2))
