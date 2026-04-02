"""Helpers to apply GA-tuned gain dictionaries onto controller gains objects."""

from typing import Dict, Any
import numpy as np
from scipy import linalg


def apply_gain_dict(gains, gain_dict: Dict[str, Any]) -> None:
    """Apply gain dict values to a gains object (handles ndarray -> matrix)."""
    for key, value in gain_dict.items():
        if hasattr(gains, key):
            if isinstance(value, np.ndarray):
                setattr(gains, key, np.matrix(value))
            else:
                setattr(gains, key, value)


def post_process_gains(controller_type: str, gains, gain_dict: Dict[str, Any]) -> None:
    """Controller-specific post-processing after gains are set."""
    if controller_type == 'MRAC':
        if 'Q_tran' in gain_dict and hasattr(gains, 'A_ref_tran'):
            gains.P_tran = np.matrix(linalg.solve_continuous_lyapunov(
                gains.A_ref_tran.T, -gain_dict['Q_tran']
            ))
        if 'Q_rot' in gain_dict and hasattr(gains, 'A_ref_rot'):
            gains.P_rot = np.matrix(linalg.solve_continuous_lyapunov(
                gains.A_ref_rot.T, -gain_dict['Q_rot']
            ))


def log_applied_gains(controller_type: str, gain_dict: Dict[str, Any], num_total_params: int = None, num_tuned_params: int = None) -> None:
    """Log applied GA gains in a controller-specific format."""
    if controller_type == 'PID':
        diag_keys = ['KP_tran', 'KI_tran', 'KD_tran', 'KP_rot', 'KI_rot', 'KD_rot']
        diag_gains = {key: _diagonal_values(gain_dict[key]) for key in diag_keys if key in gain_dict}
        print(
            "[GA TUNER] Applied external PID gains "
            f"(KP_tran={diag_gains.get('KP_tran', 'N/A')}, "
            f"KI_tran={diag_gains.get('KI_tran', 'N/A')}, "
            f"KD_tran={diag_gains.get('KD_tran', 'N/A')}, "
            f"KP_rot={diag_gains.get('KP_rot', 'N/A')}, "
            f"KI_rot={diag_gains.get('KI_rot', 'N/A')}, "
            f"KD_rot={diag_gains.get('KD_rot', 'N/A')})"
        )
    else:
        # Show detailed param counts if available
        if num_total_params is not None and num_tuned_params is not None:
            param_msg = f"{num_total_params} total params ({num_tuned_params} tuned)"
        elif num_total_params is not None:
            param_msg = f"{num_total_params} params"
        else:
            param_msg = f"{len(gain_dict)} gain matrices"
        print(f"[GA TUNER] Applied external {controller_type} gains ({param_msg})")


def _diagonal_values(matrix_like):
    """Return the diagonal of a gain matrix as a flat numpy array for logging."""
    return np.diag(np.asarray(matrix_like, dtype=float))
