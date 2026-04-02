"""
Combined (inner + outer loop) evaluator for UAV controllers.
"""

from typing import Dict, Any, Union, List, Optional
import numpy as np
import traceback

from .base_evaluator import UAVSimulationEvaluator, failure_penalty, check_metrics_diverged
from ..metrics import (
    InnerLoopMetricsCalculator, 
    OuterLoopMetricsCalculator,
    MetricNormalizer
)
from ..ga_config.metrics_config import MetricsConfig


class CombinedEvaluator(UAVSimulationEvaluator):
    """
    Combined (inner + outer loop) fitness evaluator for UAV controllers.
    
    Measures both:
    - Inner loop: Attitude tracking, angular velocity tracking, rotational control effort
    - Outer loop: Position tracking, velocity tracking, translational control effort
    """
    
    def __init__(self, 
                 uav_adapter,
                 controller_type: str,
                 log_directory: str = "simulation_logs",
                 parallel_config: Optional[Dict[str, Any]] = None,
                 multi_objective: bool = True,
                 metric_weights: Optional[Dict[str, float]] = None,
                 normalize_metrics: bool = True,
                 metrics_config: Optional[MetricsConfig] = None,
                 survival_penalty_config=None):
        """
        Initialize combined evaluator.
        
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
        
        if metrics_config is None:
            raise ValueError("metrics_config must be provided for CombinedEvaluator")
        if not multi_objective and metric_weights is None:
            raise ValueError("metric_weights must be provided for single-objective CombinedEvaluator")
        
        self.multi_objective = multi_objective
        self.normalize_metrics = normalize_metrics
        self.metric_weights = metric_weights
        
        # Metric names for combined loop (inner + outer)
        self.metric_names = [
            # Inner loop (attitude/rotational)
            'attitude_tracking_error',
            'angular_velocity_tracking_error',
            'rotational_control_effort',
            # Outer loop (position/translational)
            'position_error',
            'velocity_error',
            'translational_control_effort'
        ]
        
        # Update n_objectives for multi-objective mode
        if self.multi_objective:
            self.n_objectives = len(self.metric_names)
            print(f"[COMBINED] Multi-objective mode: {self.n_objectives} objectives")
        else:
            print(f"[COMBINED] Single-objective mode: weighted sum")
        
        # Initialize normalizer
        self.normalizer = MetricNormalizer(metric_names=self.metric_names)
        self.normalizer_fitted = False
        self.collected_metrics = []
        
        # Initialize metrics calculators
        self.inner_metrics_calculator = InnerLoopMetricsCalculator()
        self.outer_metrics_calculator = OuterLoopMetricsCalculator()
    
    def _get_fallback_fitness(self) -> Union[float, List[float]]:
        """Return fallback fitness for failed simulations."""
        return failure_penalty(self.multi_objective, len(self.metric_names))
    
    def _evaluate_parameters(self, parameters: List[float]) -> Union[float, List[float]]:
        """
        Override the base evaluation method to use combined metrics.
        
        Args:
            parameters: Parameter values to evaluate
            
        Returns:
            Combined metrics (list if multi_objective=True, float otherwise)
        """
        try:
            config, log_data = self._simulate_with_logs(parameters, prefix="eval")

            if log_data is None:
                print(f"Warning: Simulation failed for parameters {parameters[:3]}")
                return self._get_fallback_fitness()

            # Compute combined fitness from log data
            return self._compute_fitness_from_log_data(log_data, parameters)
            
        except Exception as e:
            print(f"Error in combined evaluation: {e}")
            traceback.print_exc()
            return self._get_fallback_fitness()
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], 
                                      parameters: List[float]) -> Union[float, List[float]]:
        """
        Compute fitness values from simulation log data.
        
        Args:
            log_data: Simulation log data
            parameters: Parameter values used
            
        Returns:
            Fitness values (list for multi-objective, float for single-objective)
        """
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
        
        # Simulation completed - calculate actual metrics
        # Calculate inner loop metrics
        inner_metrics_dict = self.inner_metrics_calculator.compute_all_metrics(log_data)
        
        # Calculate outer loop metrics
        outer_metrics_obj = self.outer_metrics_calculator.calculate_metrics(log_data)
        outer_metrics_dict = outer_metrics_obj.to_dict()
        
        # Extract metric values in consistent order
        metric_values = [
            # Inner loop
            inner_metrics_dict['attitude_tracking_error'],
            inner_metrics_dict['angular_velocity_tracking_error'],
            inner_metrics_dict['rotational_control_effort'],
            # Outer loop
            outer_metrics_dict['position_error'],
            outer_metrics_dict['velocity_error'],
            outer_metrics_dict['translational_control_effort']
        ]
        
        # Check for divergence
        if check_metrics_diverged(metric_values):
            print(f"Warning: Metrics diverged for parameters {parameters[:3]}")
            return self._get_fallback_fitness()
        
        # Print metrics for visibility
        print(f"\n[Combined Metrics - Parameters {parameters[:3]}...]")
        metric_names_display = [
            'Attitude Error',
            'Angular Velocity Error', 
            'Rotational Effort',
            'Position Error',
            'Velocity Error',
            'Translational Effort'
        ]
        units = ['rad', 'rad/s', 'N·m', 'm', 'm/s', 'N']
        
        for i, (name, value, unit) in enumerate(zip(metric_names_display, metric_values, units)):
            print(f"  {name:30s} {value:10.4f} {unit}")
        
        # Collect metrics for normalization fitting (if not fitted yet)
        if not self.normalizer_fitted:
            self.collected_metrics.append(metric_values)
        
        # Normalize if requested
        if self.normalize_metrics and self.normalizer_fitted:
            metric_values = self.normalizer.transform(np.array(metric_values))
            print(f"\n[Normalized Metrics]:")
            for name, value in zip(metric_names_display, metric_values):
                print(f"  {name:30s} {value:10.4f}")

        # Return based on optimization mode
        if self.multi_objective:
            # Multi-objective: return all metrics
            print(f"  [MODE: Multi-Objective - returning {len(metric_values)} objectives]\n")
            return metric_values
        else:
            # Single-objective: weighted sum
            weighted_sum = sum(
                metric_values[i] * self.metric_weights.get(name, 1.0)
                for i, name in enumerate(self.metric_names)
            )
            print(f"  [MODE: Single-Objective]")
            print(f"  Weighted Fitness: {weighted_sum:.6f}\n")
            return weighted_sum
    
    def fit_normalizer(self):
        """
        Fit the normalizer using collected metrics from initial population.
        Should be called after evaluating the initial population.
        """
        if len(self.collected_metrics) == 0:
            print("Warning: No metrics collected for normalization fitting")
            return
        
        # Stack collected metrics
        metrics_array = np.array(self.collected_metrics)
        
        # Fit normalizer (uses 95th percentile)
        self.normalizer.fit(metrics_array)
        self.normalizer_fitted = True
        
        print(f"Normalizer fitted with {len(self.collected_metrics)} samples")
        print(f"Reference values: {self.normalizer.reference_values}")
    
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
    
    def get_metric_names(self) -> List[str]:
        """Get list of metric names being evaluated."""
        return self.metric_names
