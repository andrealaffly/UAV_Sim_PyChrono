"""
TwoLayerMRAC controller parameter tuning implementation.

This module defines parameter bounds, encoding/decoding for TwoLayerMRAC controller gains.
TwoLayerMRAC has similar structure to MRAC but with additional second-layer parameters.
"""

from typing import Dict, List, Any, Optional
import numpy as np

from ...core.controller_tuning import ControllerTuningInterface
from ...core.parameter_bounds import ParameterBounds
from ...encoding.parameter_converter import (
    ControllerParameterConverter,
)
from ...encoding.cholesky_utils import build_cholesky_parameter_bounds


class TwoLayerMRACTuning(ControllerTuningInterface):
    """
    TwoLayerMRAC controller tuning interface.
    
    Handles:
    - Parameter bounds for Gamma matrices, Q matrices, reference model gains
    - Two-layer specific parameters (second layer adaptive rates)
    - Encoding/decoding between parameter vectors and gain structures
    - Cholesky parameterization for positive-definite matrices
    
    Note: TwoLayerMRAC has same parameter structure as MRAC for the first layer,
    plus additional Gamma_second_layer parameters.
    """
    
    def __init__(self, tuning_config: Dict[str, Any] = None):
        """
        Initialize TwoLayerMRAC tuning interface.
        
        Args:
            tuning_config: Optional configuration (unused, kept for interface compatibility)
        """
        # Load baseline gains from TwoLayerMRAC implementation
        self._baseline_gains = self._load_default_two_layer_mrac_gains()

        # Define parameter bounds using same approach as MRAC
        self.bounds: Dict[str, List[float]] = {}
        self.gamma_parameter_groups: Dict[str, List[str]] = {}

        scale_factor = 10.0
        log_scale_span = np.log(scale_factor)
        vector_factor = 5.0
        
        # Tighter bounds for Q matrices
        q_scale_factor = 5.0
        q_log_scale_span = np.log(q_scale_factor)

        def _positive_bounds(value: float, factor: float = 10.0,
                             *, min_lower: float = 1e-6,
                             max_upper: Optional[float] = None) -> List[float]:
            lower = max(min_lower, value / factor)
            upper = max(lower * 2.0, value * factor)
            if max_upper is not None:
                upper = min(upper, max_upper)
                if upper <= lower:
                    upper = lower * 1.5
            return [lower, upper]

        # Build matrix configs from baseline gains (detect Gamma/Q matrices)
        for matrix_name, value in self._baseline_gains.items():
            # Only process SPD matrices (Gamma and Q matrices)
            if not (matrix_name.startswith('Gamma_') or matrix_name in ('Q_tran', 'Q_rot')):
                continue
            if not isinstance(value, np.ndarray) or value.ndim != 2:
                continue
            
            prefix = matrix_name.replace('Gamma_', 'gamma_').lower() if matrix_name.startswith('Gamma_') else matrix_name.lower()
            baseline_matrix = value

            is_q_matrix = matrix_name.startswith('Q_')
            current_log_span = q_log_scale_span if is_q_matrix else log_scale_span
            current_off_diag_factor = 2.0 if is_q_matrix else vector_factor
            
            diag_bound_fn = lambda value, span=current_log_span: [value - span, value + span]
            matrix_bounds, parameter_names = build_cholesky_parameter_bounds(
                baseline_matrix,
                prefix,
                diag_bound_fn,
                off_diag_factor=current_off_diag_factor,
                min_off_diag_span=0.5 if is_q_matrix else 1.0,
                log_diagonals=True,
            )

            for name, bounds in matrix_bounds.items():
                self.bounds[name] = bounds
            self.gamma_parameter_groups[prefix] = parameter_names

        # Reference model gains
        kp_ref = np.diag(self._baseline_gains['K_P_omega_ref'])
        for i, value in enumerate(kp_ref, start=1):
            self.bounds[f"K_P_omega_ref_{i}"] = _positive_bounds(float(value), factor=5.0)

        ki_ref = np.diag(self._baseline_gains['K_I_omega_ref'])
        for i, value in enumerate(ki_ref, start=1):
            self.bounds[f"K_I_omega_ref_{i}"] = _positive_bounds(float(value), factor=10.0)

        # E-modification parameters
        for name in ['sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
                     'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot']:
            base_value = float(self._baseline_gains.get(name, 0.1))
            self.bounds[name] = _positive_bounds(base_value, factor=3.0,
                                                 min_lower=0.05, max_upper=1.5)

        # Cache ordered parameter names
        self._parameter_names = list(self.bounds.keys())

        # Instantiate ParameterBounds and converter
        lower_bounds = [self.bounds[name][0] for name in self._parameter_names]
        upper_bounds = [self.bounds[name][1] for name in self._parameter_names]
        self._parameter_bounds = ParameterBounds(
            lower_bounds,
            upper_bounds,
            self._parameter_names,
        )
        self.converter = ControllerParameterConverter(self._parameter_bounds, baseline_gains=self._baseline_gains)
    
    def _load_default_two_layer_mrac_gains(self) -> Dict[str, Any]:
        """Load baseline gains from TwoLayerMRAC implementation."""
        from acsl_pychrono.control.TwoLayerMRAC import TwoLayerMRACGains
        from acsl_pychrono.simulation.flight_params import FlightParams
        
        # Create default flight params
        flight_params = FlightParams()
        gains_obj = TwoLayerMRACGains(flight_params)
        
        # Extract relevant gains (same as MRAC plus two-layer specific)
        defaults = {
            # Translational Gamma matrices
            'Gamma_x_tran': np.asarray(gains_obj.Gamma_x_tran),
            'Gamma_r_tran': np.asarray(gains_obj.Gamma_r_tran),
            'Gamma_Theta_tran': np.asarray(gains_obj.Gamma_Theta_tran),
            
            # Rotational Gamma matrices
            'Gamma_x_rot': np.asarray(gains_obj.Gamma_x_rot),
            'Gamma_r_rot': np.asarray(gains_obj.Gamma_r_rot),
            'Gamma_Theta_rot': np.asarray(gains_obj.Gamma_Theta_rot),
            
            # Q matrices
            'Q_tran': np.asarray(gains_obj.Q_tran),
            'Q_rot': np.asarray(gains_obj.Q_rot),
            
            # Reference model gains
            'K_P_omega_ref': np.asarray(gains_obj.K_P_omega_ref),
            'K_I_omega_ref': np.asarray(gains_obj.K_I_omega_ref),
        }
        
        # Add e-modification parameters if they exist
        for param in ['sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
                      'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot']:
            if hasattr(gains_obj, param):
                defaults[param] = float(getattr(gains_obj, param))
            else:
                defaults[param] = 0.1  # Default value
        
        return defaults
    
    def get_parameter_bounds(self) -> ParameterBounds:
        """Get parameter bounds for all TwoLayerMRAC parameters."""
        return self._parameter_bounds
    
    def get_parameter_names(self) -> List[str]:
        """Get ordered list of parameter names."""
        return self._parameter_names
    
    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """Get parameter grouping information."""
        return self._parameter_bounds.get_parameter_groups()
    
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Convert parameter vector to TwoLayerMRAC gain structure.
        
        Args:
            parameter_vector: Flat list of parameter values
            
        Returns:
            Dictionary of TwoLayerMRAC gains
        """
        gains = self.converter.vector_to_gains(parameter_vector)
        
        # Convert numpy matrices to arrays
        normalized_gains = {}
        for key, value in gains.items():
            if isinstance(value, np.matrix):
                normalized_gains[key] = np.asarray(value)
            else:
                normalized_gains[key] = value
        return normalized_gains
    
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """
        Convert TwoLayerMRAC gain structure to parameter vector.
        
        Args:
            gains: Dictionary of TwoLayerMRAC gains
            
        Returns:
            Flat list of parameter values
        """
        prepared_gains = {}
        for key, value in gains.items():
            if isinstance(value, np.ndarray):
                prepared_gains[key] = np.asmatrix(value)
            elif isinstance(value, (list, tuple)):
                prepared_gains[key] = np.asmatrix(value)
            else:
                prepared_gains[key] = value
        
        return self.converter.gains_to_vector(prepared_gains)
    
    def get_default_gains(self) -> Dict[str, Any]:
        """Get default TwoLayerMRAC gains from baseline tuning."""
        defaults = {}
        for key, value in self._baseline_gains.items():
            if isinstance(value, np.ndarray):
                defaults[key] = value.copy()
            else:
                defaults[key] = float(value)
        return defaults
