"""Parameter selection utilities for GA tuning.

This module contains the implementation details for expanding parameter selections.
Please edit param_config.py for configuration instead of this file.
"""

from typing import Any, Dict, List, Tuple, Union


# =============================================================================
# PID Parameter Definitions
# =============================================================================

PID_PARAMETER_GROUPS = {
    "translational": [
        "KP_tran_x", "KP_tran_y", "KP_tran_z",
        "KI_tran_x", "KI_tran_y", "KI_tran_z",
        "KD_tran_x", "KD_tran_y", "KD_tran_z",
    ],
    "rotational": [
        "KP_rot_roll", "KP_rot_pitch", "KP_rot_yaw",
        "KI_rot_roll", "KI_rot_pitch", "KI_rot_yaw",
        "KD_rot_roll", "KD_rot_pitch", "KD_rot_yaw",
    ],
    "proportional": [
        "KP_tran_x", "KP_tran_y", "KP_tran_z",
        "KP_rot_roll", "KP_rot_pitch", "KP_rot_yaw",
    ],
    "integral": [
        "KI_tran_x", "KI_tran_y", "KI_tran_z",
        "KI_rot_roll", "KI_rot_pitch", "KI_rot_yaw",
    ],
    "derivative": [
        "KD_tran_x", "KD_tran_y", "KD_tran_z",
        "KD_rot_roll", "KD_rot_pitch", "KD_rot_yaw",
    ],
}

ALL_PID_PARAMETERS = PID_PARAMETER_GROUPS["translational"] + PID_PARAMETER_GROUPS["rotational"]

def expand_pid_tuning_selection(
    selection: Union[str, List[str], Dict[str, Any]]
) -> Tuple[str, ...]:
    """
    Convert PID tuning selection to a tuple of parameter names.
    
    Args:
        selection: One of:
            - "all": All 18 PID parameters
            - "translational": Tune only translational gains (9 params)
            - "rotational": Tune only rotational gains (9 params)
            - "proportional": Tune only KP gains (6 params)
            - "integral": Tune only KI gains (6 params)
            - "derivative": Tune only KD gains (6 params)
            - List of parameter names: Custom selection
            - Dict with "groups": List of group names to combine
    
    Returns:
        Tuple of parameter names to tune
    """
    if isinstance(selection, str):
        if selection == "all":
            return tuple(ALL_PID_PARAMETERS)
        elif selection in PID_PARAMETER_GROUPS:
            return tuple(PID_PARAMETER_GROUPS[selection])
        else:
            raise ValueError(
                f"Unknown PID selection: '{selection}'. "
                f"Valid options: 'all', {list(PID_PARAMETER_GROUPS.keys())}"
            )
    
    elif isinstance(selection, list):
        unknown = set(selection) - set(ALL_PID_PARAMETERS)
        if unknown:
            raise ValueError(f"Unknown PID parameters: {sorted(unknown)}")
        return tuple(selection)
    
    elif isinstance(selection, dict):
        if "groups" in selection:
            params = []
            for group_name in selection["groups"]:
                if group_name not in PID_PARAMETER_GROUPS:
                    raise ValueError(f"Unknown PID group: '{group_name}'")
                params.extend(PID_PARAMETER_GROUPS[group_name])
            seen = set()
            unique_params = [p for p in params if not (p in seen or seen.add(p))]
            return tuple(unique_params)
        elif "params" in selection:
            return expand_pid_tuning_selection(selection["params"])
        else:
            raise ValueError("Invalid PID selection dict. Use 'groups' or 'params' key.")
    
    raise ValueError(f"Invalid PID selection type: {type(selection)}")


# =============================================================================
# Matrix Selection Expansion
# =============================================================================

def _get_matrix_parameter_names(
    prefix: str,
    size: int,
    selection: Union[str, Dict[str, Any]]
) -> List[str]:
    """Generate parameter names for a matrix based on selection method."""
    all_params = [f"{prefix}_L{i+1}{j+1}" for i in range(size) for j in range(i+1)]
    
    if selection == "full":
        return all_params
    
    elif selection == "diagonal":
        return [f"{prefix}_L{i+1}{i+1}" for i in range(size)]
    
    elif isinstance(selection, dict):
        if "diagonal" in selection:
            mask = selection["diagonal"]
            if len(mask) != size:
                raise ValueError(f"Diagonal mask for {prefix} must have length {size}, got {len(mask)}")
            return [f"{prefix}_L{i+1}{i+1}" for i, m in enumerate(mask) if m]
        
        elif "selection_matrix" in selection:
            sel_matrix = selection["selection_matrix"]
            if len(sel_matrix) != size:
                raise ValueError(f"Selection matrix for {prefix} must have {size} rows")
            selected = []
            idx = 0
            for i, row in enumerate(sel_matrix):
                if len(row) != i + 1:
                    raise ValueError(f"Selection matrix row {i} must have {i+1} elements")
                for j, val in enumerate(row):
                    if val:
                        selected.append(all_params[idx])
                    idx += 1
            return selected
    
    raise ValueError(f"Invalid selection for {prefix}: {selection}")


def expand_matrix_selection(
    matrix_selection: Dict[str, Union[str, Dict[str, Any], bool]],
    matrix_sizes: Dict[str, int]
) -> Tuple[str, ...]:
    """
    Convert matrix selection to a tuple of parameter names (for SPD matrices).
    
    Args:
        matrix_selection: Dict mapping matrix names to selection specs:
            - "full": All lower-triangular Cholesky elements
            - "diagonal": Only diagonal elements
            - {"diagonal": [1, 0, 1, ...]}: Custom diagonal mask
            - {"selection_matrix": [[1], [0, 1], ...]}: Custom selection
            - True: Include all parameters of this type (for K_P_omega_ref, sigma_params)
        matrix_sizes: Dict mapping matrix names (lowercase) to their sizes
    
    Returns:
        Tuple of parameter names to tune
    """
    selected_params: List[str] = []
    for matrix_name, selection_spec in matrix_selection.items():
        matrix_name_lower = matrix_name.lower()
        
        # Handle special parameter groups
        if matrix_name_lower == "k_p_omega_ref" and selection_spec is True:
            selected_params.extend([f"K_P_omega_ref_{i}" for i in range(1, 4)])
            continue
        elif matrix_name_lower == "k_i_omega_ref" and selection_spec is True:
            selected_params.extend([f"K_I_omega_ref_{i}" for i in range(1, 4)])
            continue
        elif matrix_name_lower == "sigma_params" and selection_spec is True:
            selected_params.extend(['sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
                                  'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot'])
            continue
        
        # Get matrix size directly from the tuning class
        if matrix_name_lower not in matrix_sizes:
            raise ValueError(
                f"Unknown matrix: '{matrix_name}'. "
                f"Available matrices: {list(matrix_sizes.keys())}"
            )
        size = matrix_sizes[matrix_name_lower]
        
        params = _get_matrix_parameter_names(matrix_name_lower, size, selection_spec)
        selected_params.extend(params)
    
    return tuple(selected_params)
