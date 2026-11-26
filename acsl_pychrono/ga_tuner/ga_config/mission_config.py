"""Mission configuration for GA tuning simulations.

Configure simulation settings like trajectory type, duration, and visualization here.
Same format as used in acsl_pychrono/config/config.py
"""

from typing import Any, Dict
# Mission configuration: configure simulation settings
MISSION_CONFIG: Dict[str, Any] = {
    "trajectory_type": "piecewise_polynomial_trajectory",
    # If the trajectory_type is "piecewise_polynomial_trajectory", then choose the trajectory file to run
    # Path relative to 'current_working_directory/params/user_defined_trajectory'
    "trajectory_data_path": "rollercoaster_trajectory1p2.json",
    "simulation_duration_seconds": 31.5,
    "visualization_flag": False,
    "wrapper_flag": True,
}
