"""
 Inner Loop Metrics Calculator

Computes performance metrics for inner loop (attitude/rotational control).

Metrics:
- attitude_tracking_error: RMS error in roll/pitch/yaw tracking
- angular_velocity_tracking_error: RMS error in angular rate tracking
- rotational_control_effort: RMS control effort (moments u2, u3, u4)
"""

from typing import Dict, Any, Optional
from dataclasses import dataclass
import numpy as np

from .attitude_utils import (
    calculate_attitude_tracking_error,
    calculate_angular_velocity_tracking_error
)


@dataclass
class InnerLoopMetrics:
    """
    Container for inner loop performance metrics.
    Each metric is computed separately for sensitivity analysis and multi-objective optimization.
    """
    attitude_tracking_error: Optional[float] = None      # RMS error in roll, pitch, yaw tracking
    angular_velocity_tracking_error: Optional[float] = None  # RMS error in omega tracking
    rotational_control_effort: Optional[float] = None                # RMS of control moments (u2, u3, u4)

    @staticmethod
    def _format_metric(value: Optional[float]) -> str:
        """Format metric for string output."""
        if value is None:
            return "N/A"
        return f"{value:.6f}"

    def __repr__(self):
        return (f"InnerLoopMetrics("
                f"attitude_error={self._format_metric(self.attitude_tracking_error)}, "
                f"omega_error={self._format_metric(self.angular_velocity_tracking_error)}, "
                f"rotational_control_effort={self._format_metric(self.rotational_control_effort)}, "
                )
    
    def to_dict(self):
        """Convert metrics to dictionary."""
        data = {
            'attitude_tracking_error': self.attitude_tracking_error,
            'angular_velocity_tracking_error': self.angular_velocity_tracking_error,
            'rotational_control_effort': self.rotational_control_effort
        }

        return data
    
    def to_list(self):
        """Convert metrics to list for multi-objective optimization."""
        metrics = [
            self.attitude_tracking_error,
            self.angular_velocity_tracking_error,
            self.rotational_control_effort
        ]

        return metrics


class InnerLoopMetricsCalculator:
    """
    Calculator for inner loop performance metrics.
    
    This class provides a centralized implementation of metric calculations
    to ensure consistency across different parts of the codebase.
    """
    
    def __init__(self):
        """Initialize the metrics calculator."""
    def compute_all_metrics(self, log_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Compute all inner loop metrics from simulation log data.
        
        Args:
            log_data: Simulation log data dictionary
        
        Returns:
            Dictionary with the available metrics
        """
        metrics = {}
        
        try:
            metrics['attitude_tracking_error'] = self.compute_attitude_tracking_error(log_data)
            metrics['angular_velocity_tracking_error'] = self.compute_angular_velocity_tracking_error(log_data)
            metrics['rotational_control_effort'] = self.compute_rotational_control_effort(log_data)
        except Exception as e:
            print(f"Error computing inner loop metrics: {e}")
            # Return high penalty values if computation fails
            metrics = {
                'attitude_tracking_error': 999.0,
                'angular_velocity_tracking_error': 999.0,
                'rotational_control_effort': 999.0
            }
        
        return metrics
    
    def compute_metrics_object(self, log_data: Dict[str, Any]) -> InnerLoopMetrics:
        """
        Compute all metrics and return as InnerLoopMetrics object.
        
        Args:
            log_data: Simulation log data dictionary
            
        Returns:
            InnerLoopMetrics object with all metrics populated
        """
        metrics_dict = self.compute_all_metrics(log_data)
        
        return InnerLoopMetrics(
            attitude_tracking_error=metrics_dict['attitude_tracking_error'],
            angular_velocity_tracking_error=metrics_dict['angular_velocity_tracking_error'],
            rotational_control_effort=metrics_dict['rotational_control_effort'],
        )
    
    def compute_attitude_tracking_error(self, log_data: Dict[str, Any]) -> float:
        """
        This computes the roll/pitch difference with wrapped yaw error
        and measures the RMS magnitude of the resulting attitude error vector.
        """
        error_value = calculate_attitude_tracking_error(log_data)
        if error_value is None:
            return 999.0
        return error_value
    
    def compute_angular_velocity_tracking_error(self, log_data: Dict[str, Any]) -> float:
        """
        This compares the logged Euler angle derivatives (angular_position_dot)
        with the reference derivatives (roll_dot, pitch_dot, yaw_dot)
        """
        error_value = calculate_angular_velocity_tracking_error(log_data)
        if error_value is None:
            return 999.0
        return error_value
    
    def compute_rotational_control_effort(self, log_data: Dict[str, Any]) -> float:
        """
        Compute RMS control moment effort (u2, u3, u4).
        
        This measures the control effort used by the inner loop controller.
        Lower moment effort indicates more efficient control.
        u2, u3, u4 are the roll, pitch, and yaw moments respectively.
        """
        try:
            # Extract control moments from control_input
            u2 = np.array(log_data.get('control_input', {}).get('U2', [])).flatten()
            u3 = np.array(log_data.get('control_input', {}).get('U3', [])).flatten()
            u4 = np.array(log_data.get('control_input', {}).get('U4', [])).flatten()
            
            # Ensure all arrays have the same length
            min_length = min(len(u2), len(u3), len(u4))
            
            if min_length == 0:
                return 999.0
            
            # Truncate to minimum length
            u2 = u2[:min_length]
            u3 = u3[:min_length]
            u4 = u4[:min_length]
            
            # Stack into matrix
            moments = np.column_stack([u2, u3, u4])
            
            # Remove NaN and inf values
            valid_mask = ~(np.isnan(moments).any(axis=1) | np.isinf(moments).any(axis=1))
            moments_clean = moments[valid_mask]
            
            if len(moments_clean) == 0:
                return 999.0
            
            # Compute RMS moment effort (Euclidean norm at each timestep, then RMS)
            moment_magnitude = np.linalg.norm(moments_clean, axis=1)
            rms_moment = np.sqrt(np.mean(moment_magnitude ** 2))
            
            # Return penalty if result is invalid
            if np.isnan(rms_moment) or np.isinf(rms_moment):
                return 999.0
            
            return float(rms_moment)
            
        except Exception as e:
            print(f"Error computing rotational_control_effort: {e}")
            return 999.0
