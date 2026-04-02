"""
Genetic Algorithm Tuning Package for UAV Control Systems

Architecture:
-------------
- Tuners (controller-specific): Handle parameter encoding/decoding for each controller type
- Evaluators (controller-agnostic): Measure UAV performance (inner/outer loop) 
- Algorithms: Genetic algorithm implementations (DEAP, pymoo)
- Integration: Factory functions to connect everything together

To add a new controller:
1. Create tuners/NewController/new_controller_tuning.py
2. Add to tuning_classes registry below
"""

from .algorithms.base_ga import BaseGATuner
from .algorithms.deap_ga import DEAPGATuner
from .algorithms.pymoo_ga import PymooGATuner
from .core.fitness_evaluator import FitnessEvaluator
from .core.parameter_bounds import ParameterBounds
from .core.optimization_result import OptimizationResult
from .integration.uav_integration import UAVModelAdapter, create_uav_ga_tuner, summarize_tuner_result, save_pareto_front_logs
from .ga_config import GAConfig

# Import tuning interfaces
from .tuners.PID import PIDTuning
from .tuners.MRAC import MRACTuning
from .tuners.TwoLayerMRAC import TwoLayerMRACTuning
from .tuners.HybridTwoLayerMRAC import HybridTwoLayerMRACTuning
from .tuners.FunnelMRAC import FunnelMRACTuning
from .tuners.HybridMRAC import HybridMRACTuning
from .tuners.NonAdaptiveEBCI import NonAdaptiveEBCITuning

# Import evaluators
from .evaluators import InnerLoopEvaluator, OuterLoopEvaluator, CombinedEvaluator

tuning_classes = {
    'PID': PIDTuning,
    'MRAC': MRACTuning,
    'TwoLayerMRAC': TwoLayerMRACTuning,
    'HybridTwoLayerMRAC': HybridTwoLayerMRACTuning,
    'FunnelMRAC': FunnelMRACTuning,
    'HybridMRAC': HybridMRACTuning,
    'NonAdaptiveEBCI': NonAdaptiveEBCITuning,
}

evaluator_classes = {
    'inner': InnerLoopEvaluator, 
    'outer': OuterLoopEvaluator,
    'combined': CombinedEvaluator, 
}


def instantiateTuner(controller_type: str, evaluator_type: str, 
                     tuning_config=None, **evaluator_kwargs):
    """
    Factory function to instantiate tuning interface and evaluator.
    
    Args:
        controller_type: Controller to tune ('PID', 'MRAC', etc.)
        evaluator_type: UAV loop to evaluate ('inner', 'outer', or 'combined')
        tuning_config: Optional tuning configuration
        **evaluator_kwargs: Evaluator args (uav_adapter, log_directory, parallel_config, etc.)
    
    Returns:
        Tuple of (tuning_interface, evaluator)
    
    Example:
        # Tune PID on outer loop (position control)
        tuning, evaluator = instantiateTuner('PID', 'outer', uav_adapter=adapter, ...)
    """
    if controller_type not in tuning_classes:
        raise ValueError(
            f"Unknown controller type: {controller_type}. "
            f"Available: {list(tuning_classes.keys())}"
        )
    
    if evaluator_type not in evaluator_classes:
        raise ValueError(
            f"Unknown evaluator type: {evaluator_type}. "
            f"Available: {list(evaluator_classes.keys())}"
        )
    
    TuningClass = tuning_classes[controller_type]
    tuning = TuningClass(tuning_config)

    EvaluatorClass = evaluator_classes[evaluator_type]
    evaluator = EvaluatorClass(controller_type=controller_type, **evaluator_kwargs)
    
    return tuning, evaluator


__all__ = [
    # Core GA components
    'BaseGATuner',
    'DEAPGATuner', 
    'PymooGATuner',
    'FitnessEvaluator',
    'ParameterBounds',
    'OptimizationResult',
    'GAConfig',
    
    # Integration & utilities
    'UAVModelAdapter',
    'create_uav_ga_tuner',
    'summarize_tuner_result',
    'save_pareto_front_logs',
    
    # Tuning interfaces
    'PIDTuning',
    'MRACTuning',
    'TwoLayerMRACTuning',
    
    # Evaluators
    'InnerLoopEvaluator',
    'OuterLoopEvaluator',
    'CombinedEvaluator',
    
    # Registries & factory
    'tuning_classes',
    'evaluator_classes',
    'instantiateTuner',
]
