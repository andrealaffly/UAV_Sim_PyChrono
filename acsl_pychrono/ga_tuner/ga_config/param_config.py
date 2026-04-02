"""Parameter configuration for GA tuning.

Selection types:
    1. "all": Tune all available parameters (use for scalar-only controllers)
    
    2. [...]: Explicit list of parameter names (use for scalar-only controllers)
    
    3. {matrix_name: selection, ...}: Matrix-based selection with Cholesky parameterization
       Required for controllers with SPD (Symmetric Positive Definite) matrices.
       Per-matrix options:
         - "full": All Cholesky elements (lower-triangular)
         - "diagonal": All diagonal elements
         - {"diagonal": [1, 0, 1, ...]}: Custom diagonal mask (1=tune, 0=skip)
         - {"selection_matrix": [[1], [0, 1], ...]}: Custom lower-triangular mask for selection

PID-only string presets:
    - "translational", "rotational", "proportional", "integral", "derivative"

Note: Controllers with SPD matrices MUST use dict-based selection (option 3).
"""

from typing import Any, Dict, Union

from .param_utils import (
    expand_pid_tuning_selection,
    expand_matrix_selection,
)

# =============================================================================
# CONFIGURATION - Edit here
# =============================================================================

TUNING_SELECTIONS: Dict[str, Union[str, list, dict]] = {
    "PID": "translational",
    
    # "MRAC": {
    #     "q_tran": "diagonal",
    #     "gamma_x_tran": {
    #         "selection_matrix": [
    #             [1],
    #             [0, 1],
    #             [0, 0, 0],
    #             [0, 1, 0, 0],
    #             [0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0],
    #         ]
    #     },
    #     "gamma_r_tran": "full",
    # },
    # "MRAC": {
    #     # Adaptive gain matrices (SPD - use Cholesky parameterization)
    #     "gamma_x_tran": "diagonal",
    #     "gamma_x_rot": "diagonal",
    #     "gamma_r_tran": "diagonal",
    #     "gamma_r_rot": "diagonal",
    #     "gamma_theta_tran": "diagonal",
    #     "gamma_theta_rot": "diagonal",
        
    #     # Lyapunov matrices (SPD - use Cholesky parameterization)
    #     "q_tran": "diagonal",
    #     "q_rot": "diagonal",
    # }
}


def get_tuned_parameters(controller_type: str):
    """Get tuned parameters for the specified controller type.
    
    Returns all parameters by default, or a subset based on TUNING_SELECTIONS.
    Automatically handles Cholesky parameterization for SPD matrices.
    """
    # Import here to avoid circular dependency
    from .. import tuning_classes
    
    if controller_type not in tuning_classes:
        raise ValueError(f"Unknown controller type: {controller_type}. Available: {list(tuning_classes.keys())}")
    
    # Get all available parameters from the tuning class
    tuning_instance = tuning_classes[controller_type]()
    bounds = tuning_instance.get_parameter_bounds()
    all_params = list(bounds.parameter_names)
    
    # Check if there's a selection config for this controller
    selection = TUNING_SELECTIONS.get(controller_type, "all")
    
    # If "all", return all parameters
    if selection == "all":
        return all_params
    
    # If it's a list, validate and return it
    if isinstance(selection, list):
        # Validate that all requested params exist
        unknown = set(selection) - set(all_params)
        if unknown:
            raise ValueError(f"Unknown parameters for {controller_type}: {sorted(unknown)}")
        return selection
    
    # If it's a dict, use matrix selection (for SPD matrices with Cholesky parameterization)
    if isinstance(selection, dict):
        # Get matrix sizes directly from tuning class (if available)
        matrix_sizes = tuning_instance.get_matrix_sizes() if hasattr(tuning_instance, 'get_matrix_sizes') else {}
        selected = expand_matrix_selection(selection, matrix_sizes)
        # Validate that all selected params exist and filter to only configured ones
        valid_selected = [p for p in selected if p in all_params]
        if len(valid_selected) != len(selected):
            missing = set(selected) - set(all_params)
            print(f"Warning: Some configured parameters not found in {controller_type}: {missing}")
        return valid_selected
    
    # If it's a string (PID presets like "translational"), use PID expansion
    if isinstance(selection, str):
        controller_upper = controller_type.upper()
        if controller_upper == "PID":
            return expand_pid_tuning_selection(selection)
        # For non-PID controllers with string selection, treat as unknown format
        print(f"Warning: String selection '{selection}' not supported for {controller_type}")
        return all_params
    
    # Fallback: unknown selection format, return all parameters
    return all_params

