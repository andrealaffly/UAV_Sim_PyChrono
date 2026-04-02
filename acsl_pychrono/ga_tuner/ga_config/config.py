"""Structured configuration helpers for GA tuning runs.

This file integrates all modular configurations from separate config files.
Please modify the individual config files (param_config.py, mission_config.py, etc.)
under the subfolder ga_config for customization.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Dict, Literal, Mapping, MutableMapping, Optional, Sequence

from .ga_config import GAConfig
from .metrics_config import MetricsConfig
from .param_config import get_tuned_parameters
from .mission_config import MISSION_CONFIG
from .evaluator_config import EVALUATOR_TYPE, LOG_DIRECTORY, PARALLEL_CONFIG, CLEAN_EVAL_FOLDERS

@dataclass(frozen=True)
class EvaluatorConfig:
    """Configuration for the fitness evaluator. Values loaded from evaluator_config.py."""
    evaluator_type: Literal["inner", "outer", "combined"]
    log_directory: str
    parallel_config: Dict[str, Any]
    metrics: MetricsConfig
    clean_eval_folders: bool = True

def _default_evaluator_config() -> EvaluatorConfig:
    """Create default EvaluatorConfig with required arguments."""
    return EvaluatorConfig(
        evaluator_type=EVALUATOR_TYPE,
        log_directory=LOG_DIRECTORY,
        parallel_config=PARALLEL_CONFIG,
        metrics=MetricsConfig(),
        clean_eval_folders=CLEAN_EVAL_FOLDERS,
    )

@dataclass(frozen=True)
class TuningConfig:
    tuned_parameters: Sequence[str] = field(default_factory=tuple)
    mission_overrides: Dict[str, Any] = field(default_factory=dict)
    evaluator: EvaluatorConfig = field(default_factory=_default_evaluator_config)
    ga: GAConfig = field(default_factory=GAConfig)
    n_objectives_inner: int = 3
    n_objectives_outer: int = 3
    n_objectives_combined: int = 6  # inner (3) + outer (3)

    def get_objective_count(self, evaluator_type: Optional[str] = None) -> int:
        evaluator = evaluator_type or self.evaluator.evaluator_type
        evaluator = evaluator.lower()
        if evaluator == "inner":
            return self.n_objectives_inner
        elif evaluator == "outer":
            return self.n_objectives_outer
        elif evaluator == "combined":
            return self.n_objectives_combined
        else:
            raise ValueError(f"Unknown evaluator type '{evaluator}'. Use 'inner', 'outer', or 'combined'")


_BASE_TUNING_CONFIG = TuningConfig()


def tuning_config_from_mapping(
    data: Mapping[str, Any], base: Optional[TuningConfig] = None
) -> TuningConfig:
    """Build a :class:`TuningConfig` by applying a mapping of overrides."""
    base_config = base or _BASE_TUNING_CONFIG

    tuned_parameters = tuple(data.get("tuned_parameters", base_config.tuned_parameters))
    mission_overrides = dict(base_config.mission_overrides)
    mission_overrides.update(data.get("mission_overrides", {}))

    evaluator = _merge_dataclass(EvaluatorConfig, base_config.evaluator, data.get("evaluator"))
    ga = _merge_dataclass(GAConfig, base_config.ga, data.get("ga"))

    n_objectives_inner = data.get("n_objectives_inner", base_config.n_objectives_inner)
    n_objectives_outer = data.get("n_objectives_outer", base_config.n_objectives_outer)
    n_objectives_combined = data.get("n_objectives_combined", base_config.n_objectives_combined)

    return TuningConfig(
        tuned_parameters=tuned_parameters,
        mission_overrides=mission_overrides,
        evaluator=evaluator,
        ga=ga,
        n_objectives_inner=n_objectives_inner,
        n_objectives_outer=n_objectives_outer,
        n_objectives_combined=n_objectives_combined,
    )

def _merge_dataclass(cls, base_value, overrides: Optional[MutableMapping[str, Any]]):
    """Recursively merge dataclass with overrides, handling nested dataclasses."""
    if overrides is None:
        return base_value
    
    init_kwargs = {}
    for field_info in fields(cls):
        field_name = field_info.name
        base_field_value = getattr(base_value, field_name)
        
        if field_name in overrides:
            override_value = overrides[field_name]
            
            # Check if this field is a dataclass that needs recursive merging
            if hasattr(field_info.type, '__dataclass_fields__') and isinstance(override_value, Mapping):
                # Recursively merge nested dataclass
                init_kwargs[field_name] = _merge_dataclass(
                    field_info.type, 
                    base_field_value, 
                    override_value
                )
            elif isinstance(override_value, Mapping) and isinstance(base_field_value, dict):
                # Regular dict merging
                init_kwargs[field_name] = dict(override_value)
            else:
                # Direct value assignment
                init_kwargs[field_name] = override_value
        else:
            # No override, keep base value
            if isinstance(base_field_value, dict):
                init_kwargs[field_name] = dict(base_field_value)
            else:
                init_kwargs[field_name] = base_field_value
    
    return cls(**init_kwargs)


# Default multi-objective inner loop tuning config
def get_default_tuning_config() -> TuningConfig:
    """
    Get the default tuning config.
    This is created fresh each time to ensure configuration changes are picked up.
    """
    # Import here to avoid circular import
    from .param_config import get_tuned_parameters
    
    ga_config = GAConfig()
    return TuningConfig(
        tuned_parameters=get_tuned_parameters(ga_config.controller_type),
        mission_overrides=MISSION_CONFIG,
        evaluator=EvaluatorConfig(
            evaluator_type=EVALUATOR_TYPE,
            log_directory=LOG_DIRECTORY,
            parallel_config=PARALLEL_CONFIG,
            metrics=MetricsConfig(),
        ),
        ga=ga_config,
    )

# For backward compatibility - creates config on first access
class _LazyTuningConfig:
    def __getattr__(self, name):
        return getattr(get_default_tuning_config(), name)
    
    def __repr__(self):
        return repr(get_default_tuning_config())

TUNING_CONFIG = _LazyTuningConfig()
