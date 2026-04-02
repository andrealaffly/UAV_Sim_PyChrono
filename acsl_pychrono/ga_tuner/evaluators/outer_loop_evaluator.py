"""
Outer loop (position/translational) evaluator for UAV controllers.
"""

from typing import Dict, Any, Union, List, Optional
import numpy as np

from .base_evaluator import UAVSimulationEvaluator, failure_penalty, check_metrics_diverged, _log_objectives
from ..metrics import OuterLoopMetrics, OuterLoopMetricsCalculator, MetricNormalizer
from ..metrics.translational_utils import calculate_position_velocity_rmse
from ..ga_config.metrics_config import MetricsConfig


class OuterLoopEvaluator(UAVSimulationEvaluator):
    """
    Outer loop (position/translational) fitness evaluator for UAV controllers.
    
    Measures:
    - Position tracking error (x, y, z)
    - Velocity tracking error
    - Translational control effort (forces)
    
    """
    
    def __init__(self, 
                 uav_adapter,
                 controller_type: str,
                 log_directory: str = "simulation_logs",
                 parallel_config: Optional[Dict[str, Any]] = None,
                 multi_objective: bool = False,
                 metric_weights: Optional[Dict[str, float]] = None,
                 normalize_metrics: bool = True,
                 metrics_config: Optional[MetricsConfig] = None,
                 survival_penalty_config=None):
        """
        Initialize outer loop evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            controller_type: Type of controller being tuned ('PID', 'MRAC', etc.)
            log_directory: Directory to store simulation logs
            parallel_config: Optional parallel configuration dict
            multi_objective: Whether to use multi-objective optimization
            metric_weights: Weights for different metrics (for single-objective mode)
            normalize_metrics: Whether to normalize metrics
            metrics_config: Metrics configuration
            survival_penalty_config: Configuration for survival time penalty
        """
        super().__init__(uav_adapter, controller_type, log_directory, parallel_config,
                         survival_penalty_config=survival_penalty_config)
        
        if metrics_config is None or metric_weights is None:
            raise ValueError("metrics_config and metric_weights must be provided for OuterLoopEvaluator")
        
        self.multi_objective = multi_objective
        self.normalize_metrics = normalize_metrics
        self.metric_weights = metric_weights
        
        # Metric names for outer loop
        self.metric_names = [
            'position_error',
            'velocity_error',
            'translational_control_effort'
        ]
        
        # Update n_objectives for multi-objective mode (critical for parallel evaluation!)
        if self.multi_objective:
            self.n_objectives = len(self.metric_names)
            print(f"[OUTER LOOP] Multi-objective mode: {self.n_objectives} objectives")
        else:
            print(f"[OUTER LOOP] Single-objective mode: weighted sum")
        
        # Initialize normalizer
        self.normalizer = MetricNormalizer(metric_names=self.metric_names)
        self.normalizer_fitted = False
        self.collected_metrics = []
        
        # Initialize metrics calculator
        self.metrics_calculator = OuterLoopMetricsCalculator()
    
    def _get_fallback_fitness(self) -> Union[float, List[float]]:
        """Return fallback fitness for failed simulations."""
        return failure_penalty(self.multi_objective, len(self.metric_names))
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness value from simulation results."""
        log_data = self._get_log_data_from_config(config)
        if log_data is not None:
            return self._compute_fitness_from_log_data(log_data, parameters)
        
        print(f"Warning: Log data not found, using fallback fitness")
        return self._get_fallback_fitness()
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness metrics from log data dictionary."""
        # Check for early crash and apply survival penalty if needed
        is_crashed, penalty = self._check_and_apply_survival_penalty(
            log_data, n_objectives=len(self.metric_names)
        )
        
        if is_crashed:
            # Return penalty values - don't calculate actual metrics for crashed simulations
            if self.multi_objective:
                return penalty
            else:
                # For single-objective, return weighted sum of penalties
                return sum(penalty) / len(penalty)
        
        # Simulation completed - calculate actual outer loop metrics
        metrics = self.metrics_calculator.calculate_metrics(log_data)
        
        # Convert metrics to array for normalization
        metrics_array = np.array([
            metrics.position_error,
            metrics.velocity_error,
            metrics.translational_control_effort
        ])
        
        return self._process_metric_array(
            metrics_array,
            self.metric_names,
            units=["m", "m/s", "N"],
            parameters=parameters,
        )
    
    def _process_metric_array(
        self,
        metrics_array: np.ndarray,
        metric_names: List[str],
        units: Optional[List[str]] = None,
        parameters: Optional[List[float]] = None,
    ) -> Union[float, List[float]]:
        """Handle metric logging, normalization, and output."""
        
        # Check for diverged/unstable simulation BEFORE normalization
        if check_metrics_diverged(metrics_array):
            print(f"\n[DIVERGENCE DETECTED] Raw metrics contain unreasonable values (>1e10 or inf/nan):")
            for idx, name in enumerate(metric_names):
                unit = f" {units[idx]}" if units and idx < len(units) else ""
                print(f"  {name.replace('_', ' ').title():30s} {metrics_array[idx]:.6e}{unit}")
            print(f"  -> Returning failure penalty")
            return self._get_fallback_fitness()
        
        self.collected_metrics.append(metrics_array.copy())

        if (
            parameters is not None
            and hasattr(self, '_tuned_indices')
            and hasattr(self, '_template')
            and len(parameters) == len(self._template)
        ):
            ga_parameters = [parameters[i] for i in self._tuned_indices]
            print(f"\n[Metrics - Tuned Parameters (n={len(ga_parameters)})] {ga_parameters[:5]}...:")
        else:
            print(f"\n[Metrics - Raw]:")

        for idx, name in enumerate(metric_names):
            unit = f" {units[idx]}" if units and idx < len(units) else ""
            print(f"  {name.replace('_', ' ').title():30s} {metrics_array[idx]:.6f}{unit}")

        # Apply normalization if enabled and normalizer is fitted
        if self.normalize_metrics and self.normalizer_fitted:
            metrics_array = self.normalizer.transform(metrics_array.reshape(1, -1))[0]
            print(f"[Metrics - Normalized]:")
            for idx, name in enumerate(metric_names):
                print(f"  {name.replace('_', ' ').title():30s} {metrics_array[idx]:.6f}")

        if self.multi_objective:
            print(f"  [MODE: Multi-Objective - returning {len(metrics_array)} objectives for Pareto]")
            return metrics_array.tolist()

        total = sum(self.metric_weights[name] * metrics_array[idx] for idx, name in enumerate(metric_names))
        print(f"  [MODE: Single-Objective - weighted sum]")
        print(f"  Weighted Fitness:                {total:.6f}")
        return float(total)

    def fit_normalizer(self):
        """Fit the normalizer using collected metrics from all evaluations."""
        if len(self.collected_metrics) > 0:
            metrics_matrix = np.array(self.collected_metrics)
            self.normalizer.fit(metrics_matrix)
            self.normalizer_fitted = True
            print(f"\n[NORMALIZER] Fitted with {len(self.collected_metrics)} samples")

    def fit_normalizer_from_reference(self, default_parameters: List[float]) -> bool:
        """
        Fit the normalizer using default gains as reference.
        
        Args:
            default_parameters: Full parameter vector from default gains
            
        Returns:
            True if reference evaluation succeeded, False otherwise
        """
        print("\n[NORMALIZER] Evaluating default parameters for normalization reference...")
        
        reference_result = self.evaluate_individual(default_parameters, use_cache=False)
        
        if reference_result is None:
            print("[NORMALIZER] WARNING: Reference evaluation returned None")
            return False
        
        reference_metrics = np.array(reference_result if isinstance(reference_result, list) else [reference_result])
        
        if np.any(np.isinf(reference_metrics)) or np.any(np.isnan(reference_metrics)):
            print("[NORMALIZER] WARNING: Reference evaluation produced inf/nan values")
            return False
        
        self.normalizer.fit_from_reference(reference_metrics)
        self.normalizer_fitted = True
        return True
