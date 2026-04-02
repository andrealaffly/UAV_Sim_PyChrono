"""
Integration module for connecting the GA tuner with the UAV simulation model.
"""

from pathlib import Path
from typing import Dict, Any, Literal, Optional, List, Sequence, Tuple, Union
from copy import deepcopy
import os
import shutil
import numpy as np

from acsl_pychrono.simulation.simulation import Simulation
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.config.config import MissionConfig, WrapperParams, SimulationConfig

from ..evaluators import InnerLoopEvaluator, OuterLoopEvaluator, CombinedEvaluator
from ..algorithms.deap_ga import DEAPGATuner
from ..algorithms.pymoo_ga import PymooGATuner
from ..core.parameter_bounds import ParameterBounds
from ..ga_config import GAConfig
from ..ga_config.config import EvaluatorConfig
from ..ga_config.metrics_config import MetricsConfig

class UAVModelAdapter:
    """
    Adapter class that connects the GA tuner with the UAV simulation model.
    """
    
    def __init__(self, 
                 base_config=None,  # Optional[Dict[str, Any]]
                 log_base_dir="simulation_logs"):  # str
        """
        Initialize the UAV model adapter.
        
        Args:
            base_config: Base configuration for simulations
            log_base_dir: Base directory for simulation logs
        """
        
        base = base_config or {}
        self.base_config = {
            'mission': base.get('mission', MissionConfig()),
            'wrapper_params': base.get('wrapper_params', WrapperParams()),
        }
        self.log_base_dir = Path(log_base_dir)
        self.log_base_dir.mkdir(parents=True, exist_ok=True)
        self.num_tuned_params = None  # Track partial tuning
    
    
    def create_simulation_config(self,
                               parameters: List[float],
                               log_dir: str,
                               controller_type: str):
        """
        Create simulation configuration with given parameters and controller type.
        """

        Path(log_dir).mkdir(parents=True, exist_ok=True)

        mission_cfg = self._clone_mission_config()
        wrapper_params = self._build_wrapper_params(controller_type, parameters)

        mission_cfg.controller_type = controller_type
        mission_cfg.wrapper_batch_dir = log_dir

        sim_config = SimulationConfig(
            mission_config=mission_cfg,
            wrapper_params=wrapper_params
        )
        return {
            'simulation_config': sim_config,
            'log_directory': log_dir,
            'parameters': parameters,
            'controller_type': controller_type
        }
    
    def _clone_mission_config(self):
        """Clone the base MissionConfig to avoid cross-run mutation."""
        return deepcopy(self.base_config['mission'])

    def _build_wrapper_params(self, controller_type: str, parameters: List[float]):
        """Create wrapper params with external parameters for GA tuning."""
        wrapper_params = deepcopy(self.base_config['wrapper_params'])
        controller_params = list(parameters)

        param_key = f"{controller_type.lower()}_params"
        wrapper_params.external_controller_params = {param_key: controller_params}
        
        # Add metadata for partial tuning tracking
        if self.num_tuned_params is not None:
            wrapper_params.external_controller_params['_num_tuned'] = self.num_tuned_params

        return wrapper_params
    
    def run_simulation(self, config):  # Dict[str, Any] -> Dict[str, Any] or None
        """
        Run UAV simulation with given configuration.
        
        Args:
            config: Simulation configuration
            
        Returns:
            Log data dictionary if simulation completed successfully, None otherwise
        """
        # Import here to avoid circular import with executor module
        from acsl_pychrono.executor.simulate_mission import simulateMission
        
        sim_config = config['simulation_config']
        simulation = Simulation(sim_config)
        git_info = Logging.getGitRepoInfo()
        log_dict = simulateMission(simulation, git_info)
        if log_dict is None:
            print("Warning: simulateMission finished without producing log data.")
        return log_dict
    
    def get_simulation_status(self):  # -> Dict[str, Any]
        """Get status of UAV simulation system."""
        return {
            'base_config': self.base_config,
            'log_base_dir': self.log_base_dir
        }


class _PymooGATunerAdapter(PymooGATuner):
    """Adapter to satisfy BaseGATuner abstract interface (methods not used by Pymoo)."""

    def _initialize_population(self):
        return []

    def _select_parents(self, population, fitnesses):
        return []

    def _crossover(self, parents):
        return []

    def _mutate(self, offspring):
        return []

    def _survive(self, population, offspring, population_fitnesses, offspring_fitnesses):
        return []


class _PartialEvaluatorMixin:
    """Mixin that maps a partial parameter vector onto the full parameter set."""

    def __init__(self, tuned_indices: Sequence[int], template_vector: Sequence[float], **kwargs):
        super().__init__(**kwargs)
        self._tuned_indices = list(tuned_indices)
        self._template = list(template_vector)

    def evaluate_individual(self, parameters, use_cache: bool = True):
        full_vector = self._template.copy()
        for idx, value in zip(self._tuned_indices, parameters):
            full_vector[idx] = float(value)
        return super().evaluate_individual(full_vector, use_cache)


class PartialInnerLoopEvaluator(_PartialEvaluatorMixin, InnerLoopEvaluator):
    """Partial-parameter adapter for the inner-loop evaluator."""


class PartialOuterLoopEvaluator(_PartialEvaluatorMixin, OuterLoopEvaluator):
    """Partial-parameter adapter for the outer-loop evaluator."""


class PartialCombinedEvaluator(_PartialEvaluatorMixin, CombinedEvaluator):
    """Partial-parameter adapter for the combined evaluator."""


def _build_partial_bounds_generic(
    tuning,
    tuned_subset: Optional[Sequence[str]] = None,
) -> Tuple[ParameterBounds, Sequence[float], Sequence[int], Sequence[str]]:
    """
    Build parameter bounds for partial parameter tuning (works for both PID and MRAC).
    
    Args:
        tuning: Tuning instance (PIDTuning or MRACTuning)
        tuned_subset: Optional list of parameter names to tune (None = all)
        
    Returns:
        Tuple of (partial_bounds, default_vector, tuned_indices, all_names)
    """
    base_bounds = tuning.get_parameter_bounds()
    all_names = list(base_bounds.parameter_names)
    default_gains = tuning.get_default_gains()
    default_vector = tuning.gains_to_vector(default_gains)

    lower_bounds: List[float] = []
    upper_bounds: List[float] = []
    tuned_indices: List[int] = []
    tuned_names: List[str] = []
    name_to_index = {name: idx for idx, name in enumerate(all_names)}

    if tuned_subset:
        unknown_params = set(tuned_subset) - set(all_names)
        if unknown_params:
            raise ValueError(f"Unknown parameters requested: {sorted(unknown_params)}")
        subset = set(tuned_subset)
    else:
        subset = set(all_names)

    for name in all_names:
        if name in subset:
            idx = name_to_index[name]
            tuned_indices.append(idx)
            tuned_names.append(name)
            lower_bounds.append(float(base_bounds.lower_bounds[idx]))
            upper_bounds.append(float(base_bounds.upper_bounds[idx]))

    partial_bounds = ParameterBounds(lower_bounds, upper_bounds, tuned_names)
    return partial_bounds, default_vector, tuned_indices, all_names


# Backward compatibility alias
_build_partial_bounds = _build_partial_bounds_generic


def _apply_mission_overrides(adapter: UAVModelAdapter, overrides: Optional[Dict[str, Any]]):
    """Apply mission-level overrides to the adapter base config."""
    if not overrides:
        return

    mission_cfg = adapter.base_config.get("mission")
    if mission_cfg is None:
        adapter.base_config["mission"] = dict(overrides)
        return

    if isinstance(mission_cfg, dict):
        mission_cfg.update(dict(overrides))
    else:
        for key, value in dict(overrides).items():
            setattr(mission_cfg, key, value)


def create_uav_ga_tuner(
    ga_config: GAConfig,
    evaluator_config: EvaluatorConfig,
    tuned_parameters: Optional[Sequence[str]] = None,
    mission_overrides: Optional[Dict[str, Any]] = None,
    uav_adapter: Optional[UAVModelAdapter] = None,
):
    """
    Factory function to create a GA tuner configured for UAV simulations.
    
    Args:
        ga_config: GA algorithm configuration
        evaluator_config: Evaluator configuration (type, metrics, log directory, etc.)
        tuned_parameters: Optional list of parameter names to tune (None = all parameters)
        mission_overrides: Optional dict of mission config overrides
        uav_adapter: Optional UAV adapter instance (created if None)
        
    Returns:
        Configured GA tuner
    """
    controller = ga_config.controller_type
    algorithm_name = ga_config.algorithm.upper()
    evaluator_type = evaluator_config.evaluator_type
    metrics_config = evaluator_config.metrics
    log_directory = evaluator_config.log_directory
    parallel_config = evaluator_config.parallel_config
    
    # Create UAV adapter if not provided
    if uav_adapter is None:
        uav_adapter = UAVModelAdapter()
    
    # Apply mission overrides
    if mission_overrides:
        _apply_mission_overrides(uav_adapter, mission_overrides)
    
    # Determine objective count based on evaluator type
    if metrics_config.multi_objective:
        objective_count = {
            "inner": 3,      # attitude, angular_velocity, rotational_effort
            "outer": 3,      # position, velocity, translational_effort
            "combined": 6,   # inner (3) + outer (3)
        }.get(evaluator_type, 3)
    else:
        objective_count = 1

    # Select tuning interface based on controller type (registry-backed)
    from .ga_utils import get_tuning_instance
    
    # Build tuning config from GAConfig
    tuning_config = {}
    if hasattr(ga_config, 'search_space_type'):
        tuning_config['search_space_type'] = ga_config.search_space_type
    if hasattr(ga_config, 'custom_reference_vector') and ga_config.custom_reference_vector is not None:
        tuning_config['custom_reference_vector'] = ga_config.custom_reference_vector
    
    try:
        tuning = get_tuning_instance(controller, tuning_config=tuning_config)
    except Exception as exc:
        raise ValueError(f"Unknown controller_type: {controller}.") from exc
    
    # Build partial bounds if tuning subset of parameters
    parameter_bounds, default_vector, tuned_indices, all_names = _build_partial_bounds_generic(
        tuning, tuned_parameters
    )
    tuned_names = list(parameter_bounds.parameter_names)
    
    # Determine if we're using partial tuning (subset of parameters)
    is_partial_tuning = tuned_parameters and len(tuned_parameters) < len(all_names)
    
    # Track partial tuning info in adapter
    if is_partial_tuning:
        uav_adapter.num_tuned_params = len(tuned_parameters)
    
    # Select evaluator based on evaluator_type (controller-agnostic!)
    if evaluator_type == "inner":
        evaluator_cls = PartialInnerLoopEvaluator if is_partial_tuning else InnerLoopEvaluator
        metric_weights = {
            'attitude_tracking_error': metrics_config.inner_loop_weights.attitude_tracking_error,
            'angular_velocity_tracking_error': metrics_config.inner_loop_weights.angular_velocity_tracking_error,
            'rotational_control_effort': metrics_config.inner_loop_weights.rotational_control_effort,
        }
    elif evaluator_type == "outer":
        evaluator_cls = PartialOuterLoopEvaluator if is_partial_tuning else OuterLoopEvaluator
        metric_weights = {
            'position_error': metrics_config.outer_loop_weights.position_error,
            'velocity_error': metrics_config.outer_loop_weights.velocity_error,
            'translational_control_effort': metrics_config.outer_loop_weights.translational_control_effort,
        }
    elif evaluator_type == "combined":
        evaluator_cls = PartialCombinedEvaluator if is_partial_tuning else CombinedEvaluator
        metric_weights = {
            # Inner loop
            'attitude_tracking_error': metrics_config.inner_loop_weights.attitude_tracking_error,
            'angular_velocity_tracking_error': metrics_config.inner_loop_weights.angular_velocity_tracking_error,
            'rotational_control_effort': metrics_config.inner_loop_weights.rotational_control_effort,
            # Outer loop
            'position_error': metrics_config.outer_loop_weights.position_error,
            'velocity_error': metrics_config.outer_loop_weights.velocity_error,
            'translational_control_effort': metrics_config.outer_loop_weights.translational_control_effort,
        }
    else:
        raise ValueError(f"Unknown evaluator_type: {evaluator_type}. Use 'inner', 'outer', or 'combined'")
    
    # Create evaluator
    evaluator_kwargs = {
        'uav_adapter': uav_adapter,
        'controller_type': controller,
        'log_directory': log_directory,
        'parallel_config': parallel_config,
        'multi_objective': metrics_config.multi_objective,
        'metric_weights': metric_weights,
        'normalize_metrics': metrics_config.normalize_metrics,
        'metrics_config': metrics_config,
    }
    
    # Configure survival penalty from GAConfig
    from ..evaluators.survival_penalty import SurvivalPenaltyConfig
    
    survival_penalty_settings = getattr(ga_config, 'survival_penalty', None)
    if survival_penalty_settings is not None:
        survival_penalty_config = SurvivalPenaltyConfig(
            enabled=survival_penalty_settings.enabled,
            penalty_base=survival_penalty_settings.penalty_base,
            time_tolerance=survival_penalty_settings.time_tolerance,
            expected_duration=survival_penalty_settings.expected_duration,
        )
        evaluator_kwargs['survival_penalty_config'] = survival_penalty_config
    
    # Add partial evaluator arguments if using partial tuning
    if is_partial_tuning:
        evaluator_kwargs.update({
            'tuned_indices': tuned_indices,
            'template_vector': default_vector,
        })
    
    fitness_evaluator = evaluator_cls(**evaluator_kwargs)
    
    # Build seed population from default gains if enabled
    seed_population = []
    
    if getattr(ga_config, 'seed_with_default_gains', True):
        # Extract the tuned subset of default gains as the primary seed
        default_tuned_subset = [default_vector[i] for i in tuned_indices]
        seed_population.append(default_tuned_subset)
        print(f"[SEEDING] Added default gains as seed ({len(default_tuned_subset)} parameters)")
    
    # Add any additional custom seed vectors
    additional_seeds = getattr(ga_config, 'additional_seed_vectors', None)
    if additional_seeds:
        for i, seed in enumerate(additional_seeds):
            if len(seed) == len(tuned_indices):
                seed_population.append(list(seed))
                print(f"[SEEDING] Added additional seed {i+1} ({len(seed)} parameters)")
            else:
                print(f"[SEEDING] Warning: Additional seed {i+1} has wrong dimension ({len(seed)} != {len(tuned_indices)}), skipping")
    
    # Build tuner kwargs from ga_config
    tuner_kwargs = {
        'population_size': ga_config.population_size,
        'n_generations': ga_config.num_generations,
        'random_seed': ga_config.random_seed,
    }
    
    # Create appropriate tuner
    if algorithm_name == 'DEAP':
        tuner_kwargs.update({
            'crossover_prob': ga_config.crossover_rate,
            'mutation_prob': ga_config.mutation_rate,
            'selection_method': ga_config.selection_method,
            'tournament_size': ga_config.tournament_size,
            'multi_objective': metrics_config.multi_objective,
            'n_objectives': objective_count,
        })
        tuner = DEAPGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    elif algorithm_name == 'PYMOO':
        tuner_kwargs.update({
            'algorithm': ga_config.pymoo_variant,
            'n_objectives': objective_count,
        })
        if ga_config.pymoo_algorithm_params:
            tuner_kwargs['algorithm_params'] = dict(ga_config.pymoo_algorithm_params)
        
        # Add seed population for Pymoo (injects hand-tuned gains into initial population)
        if seed_population:
            tuner_kwargs['seed_population'] = seed_population
        
        tuner = _PymooGATunerAdapter(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm_name}. Use 'DEAP' or 'PYMOO'")
    
    # Fit normalizer from default gains if normalization is enabled and supported
    if metrics_config.normalize_metrics and hasattr(fitness_evaluator, 'fit_normalizer_from_reference'):
        default_tuned_subset = [default_vector[i] for i in tuned_indices]
        success = fitness_evaluator.fit_normalizer_from_reference(default_tuned_subset)
        if not success:
            print("[WARNING] Failed to fit normalizer from reference - normalization disabled")
    
    # Attach metadata for result summarization
    tuner._tuned_names = tuned_names
    tuner._all_names = all_names
    tuner._evaluator_type = evaluator_type
    
    return tuner


def summarize_tuner_result(tuner, result) -> None:
    """
    Print a summary of GA tuner results.
    
    Args:
        tuner: The tuner returned by create_uav_ga_tuner
        result: The OptimizationResult from tuner.optimize()
    """
    tuned_names = getattr(tuner, '_tuned_names', [])
    all_names = getattr(tuner, '_all_names', tuned_names)
    evaluator_type = getattr(tuner, '_evaluator_type', 'outer')
    
    print("GA tuner run complete")
    print(f"Tuned parameters ({len(tuned_names)} / {len(all_names)} total): {list(tuned_names)}")
    
    if evaluator_type == "inner":
        metrics_label = "Inner loop metrics [attitude_error, angular_rate_error, rotational_effort]"
    elif evaluator_type == "combined":
        metrics_label = "Combined metrics [attitude_error, angular_rate_error, rotational_effort, position_error, velocity_error, translational_effort]"
    elif evaluator_type == "outer":
        metrics_label = "Outer loop metrics [position_error, velocity_error, translational_effort]"

    # Get normalizer to convert back to raw metrics
    evaluator = getattr(tuner, 'fitness_evaluator', None)
    normalizer = getattr(evaluator, 'normalizer', None) if evaluator else None

    if result.pareto_front is not None and len(result.pareto_front) > 0:
        print(f"\nPareto front ({len(result.pareto_front)} solutions):")
        for idx, (params, metrics) in enumerate(zip(result.pareto_front, result.pareto_fitnesses)):
            tuned_values = [f"{v:.4f}" for v in params]
            # Convert normalized metrics to raw values
            if normalizer and hasattr(normalizer, 'inverse_transform'):
                raw_metrics = normalizer.inverse_transform(np.array(metrics))
                metrics_str = [f"{m:.4f}" for m in raw_metrics]
            else:
                metrics_str = [f"{m:.4f}" for m in metrics]
            print(f"  Solution {idx + 1}:")
            print(f"    Tuned values: [{', '.join(tuned_values)}]")
            print(f"    {metrics_label}: [{', '.join(metrics_str)}]")
    elif result.best_parameters is not None:
        print(f"\nBest solution:")
        print(f"  Parameters: {result.best_parameters}")
        print(f"  Fitness: {result.get_best_fitness()}")


def save_pareto_front_logs(tuner, result, max_solutions: int = None, clean_eval_folders: bool = True):
    """Re-run simulations for Pareto front and save .mat logs in a single pareto_solutions folder.
    
    Args:
        tuner: The GA tuner instance
        result: The optimization result
        max_solutions: Maximum number of solutions to save (None = all)
        clean_eval_folders: If True, remove all eval_* folders after saving Pareto logs
    """
    
    solutions = result.pareto_front if result.pareto_front else ([result.best_parameters] if result.best_parameters is not None else [])
    if not solutions:
        return
    if max_solutions:
        solutions = solutions[:max_solutions]
    
    evaluator = tuner.fitness_evaluator
    log_dir = evaluator.log_directory
    
    # Create pareto_solutions folder
    pareto_dir = os.path.join(log_dir, "pareto_solutions")
    os.makedirs(pareto_dir, exist_ok=True)
    
    print(f"\nSaving logs for {len(solutions)} Pareto solutions in: {pareto_dir}")
    
    for i, params in enumerate(solutions):
        # Expand partial params to full vector if using partial evaluator
        if hasattr(evaluator, '_tuned_indices') and hasattr(evaluator, '_template'):
            full_params = list(evaluator._template)
            for idx, value in zip(evaluator._tuned_indices, params):
                full_params[idx] = float(value)
        else:
            full_params = list(params)
        
        # Run simulation with pareto_solutions as the log directory
        config = evaluator.uav_adapter.create_simulation_config(
            full_params,
            pareto_dir,
            controller_type=evaluator.controller_type,
        )
        evaluator.uav_adapter.run_simulation(config)
    
    # Clean up eval_* folders if requested
    if clean_eval_folders:
        print("Cleaning up evaluation folders...")
        removed_count = 0
        for item in os.listdir(log_dir):
            item_path = os.path.join(log_dir, item)
            if os.path.isdir(item_path) and item.startswith("eval_"):
                shutil.rmtree(item_path)
                removed_count += 1
        if removed_count > 0:
            print(f"  Removed {removed_count} evaluation folder(s)")
    
    print(f"Done. Pareto solution logs saved in: {pareto_dir}")
