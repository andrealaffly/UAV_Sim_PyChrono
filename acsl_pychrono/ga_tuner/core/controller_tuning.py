"""
Abstract base class for controller parameter tuning.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Union, Optional
import numpy as np

from .parameter_bounds import ParameterBounds

class ControllerTuningInterface(ABC):

    @abstractmethod
    def get_parameter_bounds(self) -> ParameterBounds:
        """
        Get bounds for all tunable parameters.
        
        Returns:
            Dictionary with parameter names as keys and [min, max] bounds as values
        """
        pass
    
    @abstractmethod
    def get_parameter_names(self) -> List[str]:
        """
        Get names of all tunable parameters.
        
        Returns:
            List of parameter names
        """
        pass
    
    @abstractmethod
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Convert parameter vector to controller gain structure.
        
        Args:
            parameter_vector: List of parameter values in defined order
            
        Returns:
            Dictionary containing controller gain structure
        """
        pass
    
    @abstractmethod
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """
        Convert controller gain structure to parameter vector.
        
        Args:
            gains: Dictionary containing controller gain structure
            
        Returns:
            List of parameter values in defined order
        """
        pass
    
    @abstractmethod
    def get_default_gains(self) -> Dict[str, Any]:
        """
        Get default controller gains.
        
        Returns:
            Dictionary containing default gain structure
        """
        pass
    
    def validate_parameters(self, parameter_vector: List[float]) -> bool:
        """
        Validate parameter vector against bounds and constraints.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Whether parameters are valid
        """
        bounds = self.get_parameter_bounds()
        
        if len(parameter_vector) != bounds.n_parameters:
            return False
        
        for i, value in enumerate(parameter_vector):
            if value < bounds.lower_bounds[i] or value > bounds.upper_bounds[i]:
                return False
        
        return True
    
    def clip_parameters(self, parameter_vector: List[float]) -> List[float]:
        """
        Clip parameter values to be within bounds.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Clipped parameter vector
        """
        bounds = self.get_parameter_bounds()
        
        clipped = []
        for i, value in enumerate(parameter_vector):
            clipped.append(np.clip(value, bounds.lower_bounds[i], bounds.upper_bounds[i]))
        
        return clipped
    
    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """
        Get parameter grouping information.
        
        Returns:
            Dictionary mapping group names to lists of parameter names
        """
        return {'all': self.get_parameter_names()}
