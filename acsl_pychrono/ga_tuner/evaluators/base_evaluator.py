"""
Base evaluator for UAV simulation-based fitness evaluation
"""

from abc import ABC, abstractmethod
from typing import List, Union, Dict, Any, Optional, Tuple
import numpy as np
import os
import pickle
from datetime import datetime

from ..core.fitness_evaluator import FitnessEvaluator
from .survival_penalty import (
    SurvivalPenaltyConfig,
    apply_survival_penalty_if_crashed,
    DEFAULT_PENALTY_BASE,
)


def failure_penalty(multi_objective: bool, n_objectives: int = 3) -> Union[float, List[float]]:
    """
    Return an INF penalty for failed evaluations.
    
    Args:
        multi_objective: Whether using multi-objective optimization
        n_objectives: Number of objectives (used when multi_objective=True)
        
    Returns:
        List of inf values for multi-objective, single inf for single-objective
    """
    return [float("inf")] * n_objectives if multi_objective else float("inf")


# Threshold for detecting diverged/unstable simulations
# Any raw metric exceeding this value is considered physically unreasonable
DIVERGENCE_THRESHOLD = 1e10


def check_metrics_diverged(metrics_array: np.ndarray) -> bool:
    """
    Check if any metric values indicate a diverged/unstable simulation.
    
    Args:
        metrics_array: Array of raw metric values (before normalization)
        
    Returns:
        True if any metric exceeds DIVERGENCE_THRESHOLD or is inf/nan
    """
    for value in metrics_array:
        if np.isnan(value) or np.isinf(value) or abs(value) > DIVERGENCE_THRESHOLD:
            return True
    return False


def _log_objectives(params: List[float], metrics: List[float], label: str) -> None:
    """Print objective vectors consistently."""
    trimmed_params = params[:3]
    joined = ", ".join(f"{m:.6f}" for m in metrics)
    print(f"[{label}] Parameters {trimmed_params} -> Objectives: {joined}")


class UAVSimulationEvaluator(FitnessEvaluator):
    """
    Base class for UAV simulation-based fitness evaluation.
    
    This provides an interface between genetic algorithms and the UAV model.
    Evaluators are controller-agnostic - they measure UAV performance metrics
    (e.g., position tracking, attitude tracking) regardless of the controller type.
    
    Supports survival time penalty: if a simulation crashes early, penalty values
    are assigned based on how long the UAV survived. This gives the GA a gradient
    to follow towards solutions that survive longer, without adding a 7th objective.
    """
    
    def __init__(self, 
                 uav_adapter,  # UAVModelAdapter
                 controller_type: str,
                 log_directory="simulation_logs",
                 parallel_config=None,
                 survival_penalty_config: Optional[SurvivalPenaltyConfig] = None):
        """
        Initialize UAV simulation evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            controller_type: Type of controller being tuned ('PID', 'MRAC', etc.)
            log_directory: Directory to store simulation logs
            parallel_config: Optional parallel configuration dict
            survival_penalty_config: Configuration for survival time penalty.
                                     If None, default config is used (enabled=True).
        """
        self.uav_adapter = uav_adapter
        self.controller_type = controller_type
        self.log_directory = log_directory
        os.makedirs(log_directory, exist_ok=True)
        
        # Configure survival time penalty
        self.survival_penalty_config = survival_penalty_config or SurvivalPenaltyConfig()
        if self.survival_penalty_config.enabled:
            print(f"[SURVIVAL PENALTY] Enabled: {self.survival_penalty_config}")
        
        # Parse parallel configuration
        if parallel_config is None:
            parallel_config = {}
        
        parallel_enabled = parallel_config.get('enabled', False)
        n_workers = parallel_config.get('n_workers', None)  # None = auto-detect CPU cores
        use_processes = parallel_config.get('use_processes', False)
        
        # Call parent constructor with evaluation function
        super().__init__(
            evaluation_function=self._evaluate_parameters,
            n_objectives=1,  # Default to single objective
            parallel=parallel_enabled,
            n_workers=n_workers,
            use_processes=use_processes
        )
        
        # Print parallelization status
        if parallel_enabled:
            worker_type = 'processes' if use_processes else 'threads'
            print(f"[PARALLEL] Enabled with {n_workers} {worker_type}")
            if not use_processes:
                print("[PARALLEL] Using threads")
            else:
                print("[PARALLEL] Using processes")
        else:
            print("[PARALLEL] Disabled - running sequentially")
    
    def _create_eval_log_dir(self, prefix: str) -> str:
        """Create a timestamped log directory for a single evaluation."""
        while True:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            eval_log_dir = os.path.join(self.log_directory, f"{prefix}_{timestamp}")
            try:
                os.makedirs(eval_log_dir, exist_ok=False)
                return eval_log_dir
            except FileExistsError:
                # If a collision occurs, wait for the next second.
                import time
                time.sleep(0.25)

    def _simulate_with_logs(self, parameters: List[float], prefix: str) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
        """
        Run a simulation and attach log metadata to the config.
        Returns (config, log_data).
        """
        eval_log_dir = self._create_eval_log_dir(prefix)

        config = self.uav_adapter.create_simulation_config(
            parameters,
            eval_log_dir,
            controller_type=self.controller_type,
        )

        log_data = self.uav_adapter.run_simulation(config)
        config["log_data"] = log_data

        log_path = self._find_log_file(eval_log_dir)
        config["log_path"] = log_path

        return config, log_data

    def _get_log_data_from_config(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Retrieve inline log data or load it from disk if available.
        """
        if "log_data" in config and config["log_data"] is not None:
            return config["log_data"]

        log_path = config.get("log_path")
        if log_path and os.path.exists(log_path):
            try:
                with open(log_path, "rb") as f:
                    return pickle.load(f)
            except Exception as e:
                print(f"Warning: Failed to load log data from {log_path}: {e}")
        return None

    def _evaluate_parameters(self, parameters: List[float]) -> Union[float, List[float]]:
        """
        Evaluate fitness for given parameters by running UAV simulation.
        
        Args:
            parameters: Parameter values to evaluate
            
        Returns:
            Fitness value (lower is better), or list of objectives for multi-objective
        """
        config, log_data = self._simulate_with_logs(parameters, prefix="eval")

        if log_data is None:
            return self._get_fallback_fitness()

        # Compute fitness from simulation results
        fitness = self._compute_fitness(config, parameters)
        
        if fitness is None:
            return self._get_fallback_fitness()
            
        return fitness

    
    def _find_log_file(self, log_dir: str) -> Optional[str]:
        """Find the simulation log file in the given directory."""
        if not os.path.exists(log_dir):
            return None
            
        # Look for the specific log file name
        log_path = os.path.join(log_dir, "log_dict.pkl")
        if os.path.exists(log_path):
            return log_path
        
        # The evaluator might create a subdirectory with timestamp, so we need to find it
        try:
            subdirs = [d for d in os.listdir(log_dir) if os.path.isdir(os.path.join(log_dir, d))]
            if subdirs:
                # Use the most recent subdirectory
                subdirs.sort()
                latest_subdir = subdirs[-1]
                log_path = os.path.join(log_dir, latest_subdir, "log_dict.pkl")
                if os.path.exists(log_path):
                    return log_path
            
            # Fallback: look for any .pkl file
            log_files = [f for f in os.listdir(log_dir) if f.endswith('.pkl')]
            if log_files:
                return os.path.join(log_dir, log_files[-1])
                
        except Exception as e:
            print(f"Error finding log file in {log_dir}: {e}")
            
        return None

    def _check_and_apply_survival_penalty(
        self,
        log_data: Dict[str, Any],
        n_objectives: int,
    ) -> Tuple[bool, Optional[List[float]]]:
        """
        Check if simulation crashed early and apply survival penalty if so.
        
        Args:
            log_data: Simulation log dictionary
            n_objectives: Number of objectives (for penalty dimensions)
            
        Returns:
            Tuple of (is_crashed, penalty_values).
            If is_crashed is False, calculate actual metrics.
            If is_crashed is True, use penalty_values instead of metrics.
        """
        if not self.survival_penalty_config.enabled:
            return False, None
        
        return apply_survival_penalty_if_crashed(
            log_data,
            n_objectives=n_objectives,
            expected_duration=self.survival_penalty_config.expected_duration,
            penalty_base=self.survival_penalty_config.penalty_base,
            tolerance=self.survival_penalty_config.time_tolerance,
        )

    @abstractmethod
    def _get_fallback_fitness(self) -> Union[float, List[float]]:
        """Return fallback fitness for failed simulations."""
        pass
    
    @abstractmethod
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness value from simulation configuration and results."""
        pass

    @abstractmethod
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness from log data dictionary."""
        pass
