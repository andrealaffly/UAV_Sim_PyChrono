"""
Parameter bounds and constraints for genetic algorithm optimization.
"""

from typing import List, Optional, Union, Dict, Tuple
import numpy as np

class ParameterBounds:
    """
    Class to handle parameter bounds and constraints for optimization.
    """
    
    def __init__(self,
                 lower_bounds: Union[List[float], np.ndarray],
                 upper_bounds: Union[List[float], np.ndarray],
                 parameter_names: Optional[List[str]] = None,
                 constraints: Optional[Dict] = None,
                 parameter_groups: Optional[Dict[str, List[str]]] = None):
        """
        Initialize parameter bounds.
        
        Args:
            lower_bounds: Lower bounds for each parameter
            upper_bounds: Upper bounds for each parameter
            parameter_names: Names for each parameter
            constraints: Additional constraints on parameters
            parameter_groups: Optional grouping information for parameters
        """
        # Convert to numpy arrays
        self.lower_bounds = np.array(lower_bounds)
        self.upper_bounds = np.array(upper_bounds)
        
        # Validate bounds
        if len(self.lower_bounds) != len(self.upper_bounds):
            raise ValueError("Lower and upper bounds must have the same length")
        
        if not all(self.lower_bounds <= self.upper_bounds):
            raise ValueError("Lower bounds must be less than or equal to upper bounds")
        
        self.n_parameters = len(self.lower_bounds)
        
        # Set parameter names
        if parameter_names is None:
            parameter_names = [f"param_{i}" for i in range(self.n_parameters)]
        elif len(parameter_names) != self.n_parameters:
            raise ValueError("Number of parameter names must match number of parameters")
        
        self.parameter_names = list(parameter_names)

        # Store parameter groups, always expose an 'all' group
        if parameter_groups is None:
            self.parameter_groups = {'all': self.parameter_names.copy()}
        else:
            self.parameter_groups = {key: list(value) for key, value in parameter_groups.items()}
            if 'all' not in self.parameter_groups:
                self.parameter_groups['all'] = self.parameter_names.copy()

        # Store constraints
        self.constraints = constraints or {}
    
    def validate_parameters(self, parameters: Union[List[float], np.ndarray]) -> bool:
        """
        Check if parameters satisfy bounds and constraints.
        
        Args:
            parameters: Parameters to validate
            
        Returns:
            Whether parameters are valid
        """
        parameters = np.array(parameters)
        
        # Check length
        if len(parameters) != self.n_parameters:
            return False
        
        # Check bounds
        if not all(parameters >= self.lower_bounds) or not all(parameters <= self.upper_bounds):
            return False
        
        # Check additional constraints
        return self._check_constraints(parameters)
    
    def _check_constraints(self, parameters: np.ndarray) -> bool:
        """
        Check if parameters satisfy additional constraints.
        
        Args:
            parameters: Parameters to validate
            
        Returns:
            Whether parameters satisfy constraints
        """
        if not self.constraints:
            return True
        
        # Example constraints:
        # 'sum': {'min': 0, 'max': 1}  # Sum of parameters must be between 0 and 1
        # 'ratio': {'pairs': [(0,1)], 'min': 0.5, 'max': 2}  # Ratio between parameters must be between 0.5 and 2
        
        for constraint_type, constraint_params in self.constraints.items():
            if constraint_type == 'sum':
                param_sum = np.sum(parameters)
                if param_sum < constraint_params.get('min', -np.inf) or \
                   param_sum > constraint_params.get('max', np.inf):
                    return False
            
            elif constraint_type == 'ratio':
                for i, j in constraint_params.get('pairs', []):
                    ratio = parameters[i] / parameters[j] if parameters[j] != 0 else np.inf
                    if ratio < constraint_params.get('min', -np.inf) or \
                       ratio > constraint_params.get('max', np.inf):
                        return False
        
        return True
    
    def clip_parameters(self, parameters: Union[List[float], np.ndarray]) -> np.ndarray:
        """
        Clip parameters to be within bounds.
        
        Args:
            parameters: Parameters to clip
            
        Returns:
            Clipped parameters
        """
        return np.clip(parameters, self.lower_bounds, self.upper_bounds)
    
    def get_parameter_range(self, param_index: int) -> Tuple[float, float]:
        """
        Get the range for a specific parameter.
        
        Args:
            param_index: Index of the parameter
            
        Returns:
            Tuple of (lower_bound, upper_bound)
        """
        if param_index < 0 or param_index >= self.n_parameters:
            raise ValueError(f"Invalid parameter index: {param_index}")
        
        return self.lower_bounds[param_index], self.upper_bounds[param_index]
    
    def get_parameter_name(self, param_index: int) -> str:
        """
        Get the name of a specific parameter.
        
        Args:
            param_index: Index of the parameter
            
        Returns:
            Parameter name
        """
        if param_index < 0 or param_index >= self.n_parameters:
            raise ValueError(f"Invalid parameter index: {param_index}")
        
        return self.parameter_names[param_index]
    
    def get_random_parameters(self, n_samples: int = 1) -> np.ndarray:
        """
        Generate random parameters within bounds.
        
        Args:
            n_samples: Number of parameter sets to generate
            
        Returns:
            Array of random parameters
        """
        parameters = np.random.uniform(
            self.lower_bounds,
            self.upper_bounds,
            size=(n_samples, self.n_parameters)
        )
        
        if n_samples == 1:
            return parameters[0]
        return parameters
    
    def get_parameter_info(self) -> Dict:
        """
        Get complete parameter information.
        
        Returns:
            Dictionary with parameter information
        """
        return {
            'n_parameters': self.n_parameters,
            'parameter_names': self.parameter_names,
            'lower_bounds': self.lower_bounds.tolist(),
            'upper_bounds': self.upper_bounds.tolist(),
            'constraints': self.constraints,
            'parameter_groups': self.parameter_groups
        }
    
    def __str__(self) -> str:
        """String representation of parameter bounds."""
        info = []
        for i in range(self.n_parameters):
            info.append(f"{self.parameter_names[i]}: "
                       f"[{self.lower_bounds[i]:.3f}, {self.upper_bounds[i]:.3f}]")
        
        constraint_str = "\nConstraints:" if self.constraints else ""
        for c_type, c_params in self.constraints.items():
            constraint_str += f"\n  {c_type}: {c_params}"
        
        return "Parameter Bounds:\n" + "\n".join(info) + constraint_str
    
    def __len__(self) -> int:
        """Number of parameters."""
        return self.n_parameters

    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """
        Get parameter grouping information.

        Returns:
            Dictionary mapping group names to parameter name lists
        """
        return {key: value.copy() for key, value in self.parameter_groups.items()}
