"""
Helper functions for MRAC inner-loop attitude and angular velocity metrics.
"""

from typing import Dict, Optional, Tuple

import numpy as np


def _stack_series(data: Dict[str, list], keys: Tuple[str, ...]) -> np.ndarray:
    """Stack multiple 1-D series from a dict into a (N, len(keys)) array."""
    arrays = []
    lengths = []
    for key in keys:
        series = np.asarray(data.get(key, []), dtype=float).flatten()
        arrays.append(series)
        lengths.append(len(series))

    min_len = min(lengths) if lengths else 0
    if min_len == 0:
        raise ValueError(f"Missing data for keys: {keys}")

    trimmed = [arr[:min_len] for arr in arrays]
    return np.column_stack(trimmed)


def _wrap_angle(angle: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi] range similar to Control.computeAngularError."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def calculate_attitude_tracking_error(log_data: Dict[str, Dict[str, list]]) -> Optional[float]:
    """
    Compute RMS attitude tracking error using Control.computeAngularError logic.

    Args:
        log_data: Simulation log data containing actual and reference Euler angles.

    Returns:
        RMS magnitude of the roll/pitch/yaw tracking error vector, or None if unavailable.
    """
    try:
        actual_angles = _stack_series(
            log_data["euler_angles"],
            ("roll", "pitch", "yaw"),
        )
        ref_roll_pitch = _stack_series(
            log_data["desired_euler_angles"],
            ("roll", "pitch"),
        )
        ref_yaw = np.asarray(log_data.get("user_defined_yaw", []), dtype=float).flatten()
    except (KeyError, ValueError) as exc:
        print(f"Warning: Unable to compute attitude tracking error: {exc}")
        return None

    min_len = min(len(actual_angles), len(ref_roll_pitch), len(ref_yaw))
    if min_len == 0:
        return None

    actual_angles = actual_angles[:min_len]
    ref_angles = np.column_stack([
        ref_roll_pitch[:min_len, 0],
        ref_roll_pitch[:min_len, 1],
        ref_yaw[:min_len],
    ])

    roll_error = actual_angles[:, 0] - ref_angles[:, 0]
    pitch_error = actual_angles[:, 1] - ref_angles[:, 1]
    yaw_error = _wrap_angle(actual_angles[:, 2] - ref_angles[:, 2])

    attitude_error = np.column_stack([roll_error, pitch_error, yaw_error])
    valid_mask = ~(np.isnan(attitude_error).any(axis=1) | np.isinf(attitude_error).any(axis=1))
    if not np.any(valid_mask):
        return None

    error_magnitude = np.linalg.norm(attitude_error[valid_mask], axis=1)
    result = float(np.sqrt(np.mean(error_magnitude ** 2)))
    return None if np.isnan(result) or np.isinf(result) else result


def calculate_angular_velocity_tracking_error(log_data: Dict[str, Dict[str, list]]) -> Optional[float]:
    """
    Compute RMS tracking error for angular velocity using Euler angle derivatives.

    Args:
        log_data: Simulation log with actual angular position derivatives and reference rates.

    Returns:
        RMS magnitude of the angular rate tracking error vector, or None if unavailable.
    """
    try:
        actual_rates = _stack_series(
            log_data["euler_angles_dot"],
            ("roll_dot", "pitch_dot", "yaw_dot"),
        )
        ref_roll_pitch_dot = _stack_series(
            log_data["desired_euler_angles"],
            ("roll_dot", "pitch_dot"),
        )
        ref_yaw_dot = np.asarray(log_data.get("user_defined_yaw_dot", []), dtype=float).flatten()
    except (KeyError, ValueError) as exc:
        print(f"Warning: Unable to compute angular velocity tracking error: {exc}")
        return None

    min_len = min(len(actual_rates), len(ref_roll_pitch_dot), len(ref_yaw_dot))
    if min_len == 0:
        return None

    actual_rates = actual_rates[:min_len]
    ref_rates = np.column_stack([
        ref_roll_pitch_dot[:min_len, 0],
        ref_roll_pitch_dot[:min_len, 1],
        ref_yaw_dot[:min_len],
    ])

    omega_error = actual_rates - ref_rates
    valid_mask = ~(np.isnan(omega_error).any(axis=1) | np.isinf(omega_error).any(axis=1))
    if not np.any(valid_mask):
        return None

    error_magnitude = np.linalg.norm(omega_error[valid_mask], axis=1)
    result = float(np.sqrt(np.mean(error_magnitude ** 2)))
    return None if np.isnan(result) or np.isinf(result) else result
