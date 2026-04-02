"""
PID controller parameter tuning implementation.

This module defines parameter bounds, encoding/decoding for PID controller gains.
"""

from typing import Dict, List, Any
import numpy as np

from ...core.controller_tuning import ControllerTuningInterface
from ...core.parameter_bounds import ParameterBounds


class PIDTuning(ControllerTuningInterface):
    """
    PID controller tuning interface.
    
    Handles:
    - Parameter bounds for KP, KI, KD (translational and rotational)
    - Encoding/decoding between parameter vectors and gain matrices
    - Parameter grouping for selective tuning
    """
    
    def __init__(self, tuning_config: Dict[str, Any] = None):
        """
        Initialize PID tuning interface.
        
        Args:
            tuning_config: Optional configuration (unused, kept for interface compatibility)
        """
        # Axis labels used across translational and rotational controllers
        self._trans_axes = ['x', 'y', 'z']
        self._rot_axes = ['roll', 'pitch', 'yaw']
        control_terms = ['KP', 'KI', 'KD']

        # Define parameter bounds for translational and rotational gains
        translational_bounds = {
            'KP': [0.0, 150.0],
            'KI': [0.0, 150.0],
            'KD': [0.0, 150.0],
        }
        rotational_bounds = {
            'KP': [0.0, 200.0],
            'KI': [0.0, 150.0],
            'KD': [0.0, 150.0],
        }

        # Populate ordered bounds dictionary
        self.bounds = {}
        for axis in self._trans_axes:
            for term in control_terms:
                name = f"{term}_tran_{axis}"
                self.bounds[name] = translational_bounds[term]
        for axis in self._rot_axes:
            for term in control_terms:
                name = f"{term}_rot_{axis}"
                self.bounds[name] = rotational_bounds[term]

        # Parameter grouping helpers
        translational_params = [name for name in self.bounds if "tran" in name]
        rotational_params = [name for name in self.bounds if "rot" in name]
        self.groups = {
            'translational': translational_params,
            'rotational': rotational_params,
            'proportional_tran': [f"KP_tran_{axis}" for axis in self._trans_axes],
            'integral_tran': [f"KI_tran_{axis}" for axis in self._trans_axes],
            'derivative_tran': [f"KD_tran_{axis}" for axis in self._trans_axes],
            'proportional_rot': [f"KP_rot_{axis}" for axis in self._rot_axes],
            'integral_rot': [f"KI_rot_{axis}" for axis in self._rot_axes],
            'derivative_rot': [f"KD_rot_{axis}" for axis in self._rot_axes],
        }
        for axis in self._trans_axes:
            self.groups[f"tran_{axis}"] = [f"{term}_tran_{axis}" for term in control_terms]
        for axis in self._rot_axes:
            self.groups[f"rot_{axis}"] = [f"{term}_rot_{axis}" for term in control_terms]
    
    def get_parameter_bounds(self) -> ParameterBounds:
        """Get parameter bounds for all PID gains."""
        names = list(self.bounds.keys())
        lower_bounds = [self.bounds[name][0] for name in names]
        upper_bounds = [self.bounds[name][1] for name in names]
        
        return ParameterBounds(
            lower_bounds,
            upper_bounds,
            names,
            parameter_groups=self.groups
        )
    
    def get_parameter_names(self) -> List[str]:
        """Get ordered list of parameter names."""
        return list(self.bounds.keys())
    
    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """Get parameter groupings for selective tuning."""
        return self.groups
    
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Convert parameter vector to PID gain matrices.
        
        Args:
            parameter_vector: Flat list of parameter values
            
        Returns:
            Dictionary of gain matrices (KP_tran, KI_tran, etc.)
        """
        names = self.get_parameter_names()
        if len(parameter_vector) != len(names):
            raise ValueError(f"Expected {len(names)} parameters, got {len(parameter_vector)}")
        
        # Extract individual gains
        param_dict = dict(zip(names, parameter_vector))
        kp_tran = [param_dict[f"KP_tran_{axis}"] for axis in self._trans_axes]
        ki_tran = [param_dict[f"KI_tran_{axis}"] for axis in self._trans_axes]
        kd_tran = [param_dict[f"KD_tran_{axis}"] for axis in self._trans_axes]
        kp_rot = [param_dict[f"KP_rot_{axis}"] for axis in self._rot_axes]
        ki_rot = [param_dict[f"KI_rot_{axis}"] for axis in self._rot_axes]
        kd_rot = [param_dict[f"KD_rot_{axis}"] for axis in self._rot_axes]

        return {
            'KP_tran': np.diag(kp_tran),
            'KI_tran': np.diag(ki_tran),
            'KD_tran': np.diag(kd_tran),
            'KP_rot': np.diag(kp_rot),
            'KI_rot': np.diag(ki_rot),
            'KD_rot': np.diag(kd_rot),
        }
    
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """
        Convert PID gain matrices to parameter vector.
        
        Args:
            gains: Dictionary of gain matrices
            
        Returns:
            Flat list of parameter values
        """
        def _diag(matrix: Any) -> np.ndarray:
            return np.diag(np.asarray(matrix))

        kp_tran = _diag(gains['KP_tran'])
        ki_tran = _diag(gains['KI_tran'])
        kd_tran = _diag(gains['KD_tran'])
        kp_rot = _diag(gains['KP_rot'])
        ki_rot = _diag(gains['KI_rot'])
        kd_rot = _diag(gains['KD_rot'])

        vector: List[float] = []

        for idx, axis in enumerate(self._trans_axes):
            vector.extend([
                float(kp_tran[idx]),
                float(ki_tran[idx]),
                float(kd_tran[idx]),
            ])

        for idx, axis in enumerate(self._rot_axes):
            vector.extend([
                float(kp_rot[idx]),
                float(ki_rot[idx]),
                float(kd_rot[idx]),
            ])

        return vector
    
    def get_default_gains(self) -> Dict[str, Any]:
        """Get default PID gains."""
        return {
            'KP_tran': np.diag([77.0, 84.0, 35.0]),
            'KI_tran': np.diag([129.0, 53.0, 53.0]),
            'KD_tran': np.diag([34.0, 29.0, 6.0]),
            'KP_rot': np.diag([100.0, 100.0, 50.0]),
            'KI_rot': np.diag([20.0, 20.0, 10.0]),
            'KD_rot': np.diag([50.0, 50.0, 50.0]),
        }
