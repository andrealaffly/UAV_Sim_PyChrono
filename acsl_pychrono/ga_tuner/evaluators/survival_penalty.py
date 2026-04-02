"""
Survival time penalty for incomplete simulations.

This module provides utilities to detect early crashes and calculate
penalty values for simulations that didn't complete the full duration.

The penalty method works as follows:
- If the UAV crashes (t < t_max): Assign penalty values based on survival time
- If the UAV survives (t = t_max): Calculate actual performance metrics

This gives the GA a gradient to follow: a drone surviving 10 seconds is "better"
than one surviving 2 seconds, but both are "worse" than any drone completing
the full simulation. The survival time acts as a "ladder" to help the GA
climb from the infeasible (crashing) region to the feasible (performance) region.
"""

from typing import Dict, Any, List, Optional, Tuple
import numpy as np


# Configuration for penalty calculation
# Penalty is designed to be on a similar scale to normalized metrics (0-100 range)
# but clearly worse than any feasible solution.
# Formula: base_penalty * (1 + time_deficit_ratio)
# Where time_deficit_ratio = (t_max - t_survival) / t_max (ranges from 0 to 1)
# This ensures:
#   - Crash at t=0: penalty = base * 2 (e.g., 1000 * 2 = 2000)
#   - Crash at t=t_max/2: penalty = base * 1.5 (e.g., 1000 * 1.5 = 1500)
#   - Crash at t=t_max: penalty = base * 1 (e.g., 1000 * 1 = 1000)
# This gives a clear gradient while staying in a reasonable numeric range.
DEFAULT_PENALTY_BASE = 1000.0  # Base penalty value (should be > max normalized metric ~100)
DEFAULT_TIME_TOLERANCE = 0.5  # Tolerance in seconds for considering simulation complete


def get_simulation_times(log_data: Dict[str, Any]) -> Tuple[Optional[float], Optional[float]]:
    """
    Extract survival time and expected duration from simulation log data.
    
    Args:
        log_data: Simulation log dictionary containing time data
        
    Returns:
        Tuple of (survival_time, expected_duration).
        Returns (None, None) if time data cannot be extracted.
    """
    try:
        # Extract time array from log data
        time_data = log_data.get("time")
        if time_data is None:
            return None, None
        
        # Handle different formats (array, list, nested dict)
        if isinstance(time_data, np.ndarray):
            time_array = time_data.flatten()
        elif isinstance(time_data, (list, tuple)):
            time_array = np.asarray(time_data, dtype=float).flatten()
        else:
            return None, None
        
        if len(time_array) == 0:
            return None, None
        
        # Get the last recorded time (survival time)
        survival_time = float(time_array[-1])
        
        # Try to get expected duration from simulation config if available
        # For now, we'll use a default or try to infer from trajectory data
        expected_duration = _infer_expected_duration(log_data, time_array)
        
        return survival_time, expected_duration
        
    except Exception as e:
        print(f"[SURVIVAL PENALTY] Error extracting simulation times: {e}")
        return None, None


def _infer_expected_duration(log_data: Dict[str, Any], time_array: np.ndarray) -> Optional[float]:
    """
    Try to infer the expected simulation duration from log data.
    
    Args:
        log_data: Simulation log dictionary
        time_array: Array of recorded simulation times
        
    Returns:
        Expected duration in seconds, or None if cannot be determined.
    """
    # Try to get expected duration from config stored in log_data
    config = log_data.get("config")
    if config is not None:
        # Check for simulation_end_time or similar config fields
        if isinstance(config, dict):
            end_time = config.get("simulation_end_time")
            if end_time is not None:
                return float(end_time)
    
    # Check if trajectory end time is available
    trajectory = log_data.get("trajectory")
    if trajectory is not None and isinstance(trajectory, dict):
        trajectory_end_time = trajectory.get("end_time")
        if trajectory_end_time is not None:
            return float(trajectory_end_time)
    
    # Default fallback: use config default (31.5 seconds from config.py)
    # This is the expected simulation duration - do NOT infer from crashed data
    # as that would make the crash look like completion
    return 31.5


def check_simulation_complete(
    log_data: Dict[str, Any],
    expected_duration: Optional[float] = None,
    tolerance: float = DEFAULT_TIME_TOLERANCE,
) -> Tuple[bool, float, float]:
    """
    Check if a simulation completed successfully or crashed early.
    
    Args:
        log_data: Simulation log dictionary
        expected_duration: Expected simulation duration in seconds (None = auto-detect)
        tolerance: Time tolerance for considering simulation complete (seconds)
        
    Returns:
        Tuple of (is_complete, survival_time, expected_duration).
        is_complete: True if simulation ran to completion
        survival_time: How long the simulation actually ran
        expected_duration: Expected total duration
    """
    survival_time, detected_duration = get_simulation_times(log_data)
    
    if survival_time is None:
        # Cannot determine times - treat as crashed immediately
        return False, 0.0, expected_duration or 31.5
    
    # Use provided or detected duration
    t_max = expected_duration if expected_duration is not None else (detected_duration or 31.5)
    
    # Check if simulation completed within tolerance
    is_complete = survival_time >= (t_max - tolerance)
    
    return is_complete, survival_time, t_max


def calculate_survival_penalty(
    n_objectives: int,
    survival_time: float,
    expected_duration: float,
    penalty_base: float = DEFAULT_PENALTY_BASE,
) -> List[float]:
    """
    Calculate penalty values for a crashed simulation based on survival time.
    
    The penalty decreases as survival time increases, giving the GA a gradient
    to follow towards solutions that survive longer.
    
    Formula: penalty = base * (1 + time_deficit_ratio)
    Where time_deficit_ratio = (t_max - t_survival) / t_max
    
    This creates penalties in range [base, 2*base]:
    - Immediate crash (t=0): penalty = 2 * base (e.g., 2000)
    - Half-way crash (t=t_max/2): penalty = 1.5 * base (e.g., 1500)
    - Near completion: penalty = ~base (e.g., ~1000)
    
    The penalty is designed to be on a similar scale to normalized metrics
    (typically 0-100) but clearly worse, ensuring:
    1. All crashed solutions are dominated by all successful solutions
    2. Longer-surviving crashes are preferred over shorter ones (gradient for GA)
    3. Numeric stability in GA operations (no 7 orders of magnitude gaps)
    
    Args:
        n_objectives: Number of objectives in the optimization
        survival_time: How long the UAV survived (seconds)
        expected_duration: Expected simulation duration (seconds)
        penalty_base: Base penalty value (should be >> max normalized metric)
        
    Returns:
        List of penalty values, one for each objective.
    """
    # Calculate time deficit ratio (0 = completed, 1 = immediate crash)
    time_deficit_ratio = max(0.0, (expected_duration - survival_time) / expected_duration)
    
    # Calculate penalty: base * (1 + deficit_ratio)
    # Range: [base, 2*base] giving clear gradient
    penalty = penalty_base * (1.0 + time_deficit_ratio)
    
    # Apply same penalty to all objectives
    penalties = [penalty] * n_objectives
    
    return penalties


def apply_survival_penalty_if_crashed(
    log_data: Dict[str, Any],
    n_objectives: int,
    expected_duration: Optional[float] = None,
    penalty_base: float = DEFAULT_PENALTY_BASE,
    tolerance: float = DEFAULT_TIME_TOLERANCE,
) -> Tuple[bool, Optional[List[float]]]:
    """
    Check if simulation crashed and return penalty if so.
    
    Args:
        log_data: Simulation log dictionary
        n_objectives: Number of objectives in the optimization
        expected_duration: Expected simulation duration (None = auto-detect)
        penalty_base: Base penalty value for calculation
        tolerance: Time tolerance for considering simulation complete
        
    Returns:
        Tuple of (is_crashed, penalty_values).
        If is_crashed is False, penalty_values will be None (calculate actual metrics).
        If is_crashed is True, penalty_values contains the penalty for each objective.
    """
    is_complete, survival_time, t_max = check_simulation_complete(
        log_data, expected_duration, tolerance
    )
    
    if is_complete:
        # Simulation completed - no penalty, calculate actual metrics
        return False, None
    
    # Simulation crashed - calculate survival penalty
    time_deficit_ratio = (t_max - survival_time) / t_max
    print(f"[SURVIVAL PENALTY] Crashed at t={survival_time:.2f}s / {t_max:.2f}s (deficit ratio: {time_deficit_ratio:.1%})")
    
    penalties = calculate_survival_penalty(
        n_objectives, survival_time, t_max, penalty_base
    )
    
    print(f"[SURVIVAL PENALTY] Penalty: {penalties[0]:.1f} (range: {penalty_base:.0f} to {2*penalty_base:.0f})")
    
    return True, penalties


class SurvivalPenaltyConfig:
    """Configuration for survival time penalty calculations."""
    
    def __init__(
        self,
        enabled: bool = True,
        penalty_base: float = DEFAULT_PENALTY_BASE,
        time_tolerance: float = DEFAULT_TIME_TOLERANCE,
        expected_duration: Optional[float] = None,
    ):
        """
        Initialize survival penalty configuration.
        
        Args:
            enabled: Whether to apply survival penalties
            penalty_base: Base penalty value (should be >> max normalized metric ~100)
            time_tolerance: Tolerance in seconds for considering simulation complete
            expected_duration: Expected simulation duration (None = auto-detect to 31.5s)
        """
        self.enabled = enabled
        self.penalty_base = penalty_base
        self.time_tolerance = time_tolerance
        self.expected_duration = expected_duration
    
    def __repr__(self):
        return (
            f"SurvivalPenaltyConfig(enabled={self.enabled}, "
            f"penalty_base={self.penalty_base:.0f}, "
            f"tolerance={self.time_tolerance}s, "
            f"expected_duration={self.expected_duration})"
        )
