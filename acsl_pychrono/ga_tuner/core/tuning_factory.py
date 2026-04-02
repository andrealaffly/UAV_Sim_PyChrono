"""
Factory for creating controller tuning interfaces.
"""

from typing import Dict, Any, Optional, Type, List
from .controller_tuning import ControllerTuningInterface


class ControllerTuningFactory:    
    """
    Factory for creating controller tuning interfaces.
    """
    
    @classmethod
    def _get_registry(cls) -> Dict[str, Type[ControllerTuningInterface]]:
        """Get the tuning registry from ga_tuner.__init__.py"""
        from .. import tuning_classes
        return tuning_classes
    
    @classmethod
    def register_tuner(cls, 
                      controller_type: str,
                      tuner_class: Type[ControllerTuningInterface]):
        """
        Register a new tuning implementation (deprecated).
        
        Instead of using this method, add the tuning class to the tuning_classes
        registry in ga_tuner/__init__.py.
        
        Args:
            controller_type: Type identifier for the controller
            tuner_class: Class implementing ControllerTuningInterface
        """
        print(
            "Warning: ControllerTuningFactory.register_tuner() is deprecated. "
            "Add tuning classes to ga_tuner.__init__.py tuning_classes registry instead."
        )
        if not issubclass(tuner_class, ControllerTuningInterface):
            raise ValueError(f"Tuner class must implement ControllerTuningInterface")
        
        # This will modify the main registry
        registry = cls._get_registry()
        registry[controller_type.upper()] = tuner_class
    
    @classmethod
    def create_tuner(cls,
                    controller_type: str,
                    tuning_config: Optional[Dict[str, Any]] = None) -> ControllerTuningInterface:
        """
        Create a tuning interface for the specified controller type.
        
        Automatically looks up the tuning class from the main registry.
        
        Args:
            controller_type: Type of controller to tune
            tuning_config: Optional configuration for tuning
            
        Returns:
            Configured tuning interface
            
        Raises:
            ValueError: If controller type is not supported
        """
        registry = cls._get_registry()
        tuner_class = registry.get(controller_type.upper())
        if tuner_class is None:
            raise ValueError(
                f"Unsupported controller type: {controller_type}. "
                f"Available types: {list(registry.keys())}"
            )
        
        return tuner_class(tuning_config)
    
    @classmethod
    def get_available_controllers(cls) -> List[str]:
        """
        Get list of supported controller types from the main registry.
        
        Returns:
            List of controller type identifiers
        """
        registry = cls._get_registry()
        return list(registry.keys())
    
    @classmethod
    def get_tuner_info(cls, controller_type: str) -> Dict[str, Any]:
        """
        Get information about a tuning implementation.
        
        Args:
            controller_type: Controller type identifier
            
        Returns:
            Dictionary containing tuner information
        """
        registry = cls._get_registry()
        tuner_class = registry.get(controller_type.upper())
        if tuner_class is None:
            raise ValueError(f"Unknown controller type: {controller_type}")
        
        # Create temporary instance to get info
        tuner = tuner_class()
        
        return {
            'parameter_names': tuner.get_parameter_names(),
            'parameter_groups': tuner.get_parameter_groups(),
            'parameter_bounds': tuner.get_parameter_bounds(),
            'cost_functions': tuner.get_cost_functions()
        }
