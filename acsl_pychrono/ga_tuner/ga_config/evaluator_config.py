"""Evaluator configuration for GA tuning - configure evaluation settings.

Users configure:
- Evaluator type (inner loop, outer loop, or combined)
- Log directory for simulation results
- Parallel evaluation settings
"""

from typing import Dict, Any, Literal

# Evaluator type determines which metrics are optimized
EVALUATOR_TYPE: Literal["inner", "outer", "combined"] = "combined"

# Directory where simulation logs will be saved
LOG_DIRECTORY: str = "simulation_logs/combined_diag_nsga3"

# Parallel evaluation configuration
PARALLEL_CONFIG: Dict[str, Any] = {
    "enabled": False,
    "n_workers": None,  # None = auto-detect CPU cores
    "use_processes": False,  # set this to false to use threads instead of processes if using MacOS
}

# Whether to clean up eval_* folders after saving Pareto solution logs
CLEAN_EVAL_FOLDERS: bool = True
