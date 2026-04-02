"""
Helper function for calculating RMSE-based tracking metrics.
"""

from typing import Dict, Tuple, Optional

import numpy as np


def _stack_xyz(data: Dict[str, list]) -> np.ndarray:
    """Stack x/y/z data arrays into a single (N, 3) array."""
    x = np.asarray(data.get("x", []), dtype=float).flatten()
    y = np.asarray(data.get("y", []), dtype=float).flatten()
    z = np.asarray(data.get("z", []), dtype=float).flatten()
    if not (len(x) == len(y) == len(z) and len(x) > 0):
        raise ValueError("Inconsistent xyz vector lengths for RMSE calculation")
    return np.stack([x, y, z], axis=1)


def calculate_position_velocity_rmse(log_data: Dict) -> Optional[Tuple[float, float]]:
    """
    Compute position and velocity tracking RMSE values from a simulation log.

    Args:
        log_data: Simulation log data that contains actual and desired
                  position/velocity entries.

    Returns:
        Tuple of (position_rmse, velocity_rmse) if data is valid, otherwise None.
    """
    vectors = extract_position_velocity_vectors(log_data)
    if vectors is None:
        return None

    pos_act, pos_des, vel_act, vel_des = vectors

    pos_error = pos_act - pos_des
    vel_error = vel_act - vel_des

    # Filter out NaN and inf values (consistent with attitude_utils.py)
    pos_valid_mask = ~(np.isnan(pos_error).any(axis=1) | np.isinf(pos_error).any(axis=1))
    vel_valid_mask = ~(np.isnan(vel_error).any(axis=1) | np.isinf(vel_error).any(axis=1))
    
    if not np.any(pos_valid_mask) or not np.any(vel_valid_mask):
        return None

    pos_rmse = np.sqrt(np.mean(np.sum(pos_error[pos_valid_mask] ** 2, axis=1)))
    vel_rmse = np.sqrt(np.mean(np.sum(vel_error[vel_valid_mask] ** 2, axis=1)))

    # Final check for invalid results
    if np.isnan(pos_rmse) or np.isinf(pos_rmse) or np.isnan(vel_rmse) or np.isinf(vel_rmse):
        return None

    return float(pos_rmse), float(vel_rmse)


def extract_position_velocity_vectors(log_data: Dict) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    """
    Extract stacked actual/desired position and velocity arrays from log data.

    Returns:
        Tuple of (pos_act, pos_des, vel_act, vel_des) arrays or None if data invalid.
    """
    try:
        pos_des = _stack_xyz(log_data["user_defined_position"])
        pos_act = _stack_xyz(log_data["position"])
        vel_des = _stack_xyz(log_data["user_defined_velocity"])
        vel_act = _stack_xyz(log_data["velocity"])
    except (KeyError, ValueError) as exc:
        print(f"Warning: Unable to extract position/velocity vectors: {exc}")
        return None

    return pos_act, pos_des, vel_act, vel_des
