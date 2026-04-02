"""Registry helpers for GA tuning classes."""

from typing import Dict, Type, Optional, Any

from ...core.controller_tuning import ControllerTuningInterface


def get_registry() -> Dict[str, Type[ControllerTuningInterface]]:
    """Return the tuning registry from ga_tuner.__init__.py."""
    from ... import tuning_classes
    return tuning_classes


def get_tuning_class(controller_type: str) -> Type[ControllerTuningInterface]:
    """Lookup the tuning class for a controller type (case-insensitive)."""
    registry = get_registry()
    
    # Create case-insensitive lookup
    normalized_registry = {k.upper(): v for k, v in registry.items()}
    tuning_cls = normalized_registry.get(controller_type.upper())
    
    if tuning_cls is None:
        raise ValueError(
            f"Unsupported controller type: {controller_type}. "
            f"Available: {list(registry.keys())}"
        )
    return tuning_cls


def get_param_key(controller_type: str) -> str:
    """Return the external param key for a controller type."""
    return f"{controller_type.lower()}_params"


def get_tuning_instance(controller_type: str, tuning_config: Optional[Dict[str, Any]] = None) -> ControllerTuningInterface:
    """Create a tuning instance for a controller type.
    
    Args:
        controller_type: Controller type string
        tuning_config: Optional configuration dict (e.g., {'search_space_type': 'global'})
    
    Returns:
        Tuning instance
    """
    tuning_cls = get_tuning_class(controller_type)
    return tuning_cls(tuning_config=tuning_config)
