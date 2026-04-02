"""
Outer Loop Metrics Calculator

Computes performance metrics for  outer loop (position/translational control).

Metrics:
- position_error: RMS position tracking error
- velocity_error: RMS velocity tracking error
- translational_control_effort: RMS control effort (total thrust from all motors)
"""

import numpy as np
import pickle
from typing import Dict, Optional
from dataclasses import dataclass

from .translational_utils import calculate_position_velocity_rmse


@dataclass
class OuterLoopMetrics:
    """Container for outer loop performance metrics."""
    position_error: float  # RMS position tracking error
    velocity_error: float  # RMS velocity tracking error
    translational_control_effort: float  # RMS control effort (total thrust)
    
    def to_dict(self):
        """Convert metrics to dictionary."""
        return {
            'position_error': self.position_error,
            'velocity_error': self.velocity_error,
            'translational_control_effort': self.translational_control_effort
        }
    
    def to_list(self):
        """Convert metrics to list for multi-objective optimization."""
        return [
            self.position_error,
            self.velocity_error,
            self.translational_control_effort
        ]


class OuterLoopMetricsCalculator:
    """Calculator for outer loop performance metrics."""
    
    def __init__(self):
        """Initialize the outer loop metrics calculator."""
        pass
    
    def calculate_metrics(self, log_data: Dict) -> OuterLoopMetrics:
        """
        Calculate outer loop performance metrics from simulation log.
        
        Args:
            log_data: Simulation log data dictionary
            
        Returns:
            OuterLoopMetrics object with computed metrics
        """
        try:
            tracking_errors = calculate_position_velocity_rmse(log_data)
            if tracking_errors is None:
                position_error = velocity_error = 999.0
            else:
                position_error, velocity_error = tracking_errors
            
            # Calculate performance metrics
            translational_control_effort = self._calculate_translational_control_effort(log_data)
            
            return OuterLoopMetrics(
                position_error=position_error,
                velocity_error=velocity_error,
                translational_control_effort=translational_control_effort
            )
            
        except Exception as e:
            print(f"Error calculating outer loop metrics: {e}")
            return self._create_default_metrics()
    
    def _calculate_translational_control_effort(self, log_data: Dict) -> float:
        """Calculate RMS control effort (total thrust from all motors)."""
        thrust_data = log_data.get('thrust_motors_N', {})
        if not thrust_data:
            return 999.0

        total_effort = 0.0
        for motor_data in thrust_data.values():
            if isinstance(motor_data, list):
                arr = np.array(motor_data).astype(float).flatten()
            else:
                try:
                    arr = np.array(motor_data, dtype=float).flatten()
                except Exception:
                    continue

            if arr.size == 0:
                continue
            
            # Filter out NaN and inf values
            valid_arr = arr[~(np.isnan(arr) | np.isinf(arr))]
            if valid_arr.size == 0:
                continue

            total_effort += np.mean(valid_arr ** 2)

        if total_effort == 0.0 or np.isnan(total_effort) or np.isinf(total_effort):
            return 999.0

        result = float(np.sqrt(total_effort))
        return 999.0 if np.isnan(result) or np.isinf(result) else result
    
    def _create_default_metrics(self) -> OuterLoopMetrics:
        """Create default metrics for failed calculations."""
        return OuterLoopMetrics(
            position_error=999.0,
            velocity_error=999.0,
            translational_control_effort=999.0
        )


def load_simulation_log(log_path: str) -> Optional[Dict]:
    """Load simulation log from file."""
    try:
        if log_path is None:
            print(f"Error loading log None: log_path is None")
            return None
        
        if log_path.endswith('.pkl'):
            with open(log_path, 'rb') as f:
                return pickle.load(f)
        else:
            # Handle other formats if needed
            return None
    except Exception as e:
        print(f"Error loading log {log_path}: {e}")
        return None
