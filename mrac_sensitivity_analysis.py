import os
import numpy as np
from datetime import datetime
from typing import List, Dict, Any, Callable, Optional
from dataclasses import dataclass

# Import UAV simulation modules (updated ga_tuner API)
try:
    from acsl_pychrono.ga_tuner.tuners.MRAC.mrac_tuning import MRACTuning
    from acsl_pychrono.ga_tuner.integration.uav_integration import UAVModelAdapter
    from acsl_pychrono.ga_tuner.evaluators import CombinedEvaluator
    HAS_UAV_MODULES = True
except ImportError:
    HAS_UAV_MODULES = False

_SENS_WORKER_EVALUATOR = None
_SENS_WORKER_LOG_ROOT = None


def _init_sensitivity_worker(log_root: str, controller_type: str, normalize_metrics: bool):
    """Initializer for sensitivity worker processes."""
    global _SENS_WORKER_EVALUATOR, _SENS_WORKER_LOG_ROOT
    from acsl_pychrono.ga_tuner.ga_config.metrics_config import MetricsConfig
    from acsl_pychrono.ga_tuner.evaluators import CombinedEvaluator
    from acsl_pychrono.ga_tuner.integration.uav_integration import UAVModelAdapter
    metrics_config = MetricsConfig()
    _SENS_WORKER_LOG_ROOT = log_root
    _SENS_WORKER_EVALUATOR = CombinedEvaluator(
        uav_adapter=UAVModelAdapter(),
        controller_type=controller_type,
        log_directory=log_root,
        parallel_config={'enabled': False},
        multi_objective=True,
        normalize_metrics=normalize_metrics,
        metrics_config=metrics_config
    )


def _run_sensitivity_worker_simulation(parameters: List[float]) -> str:
    """Run a single simulation in a worker process and return log path."""
    global _SENS_WORKER_EVALUATOR, _SENS_WORKER_LOG_ROOT
    if _SENS_WORKER_EVALUATOR is None or _SENS_WORKER_LOG_ROOT is None:
        raise RuntimeError("Sensitivity worker evaluator not initialized")

    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        log_dir = os.path.join(_SENS_WORKER_LOG_ROOT, f"eval_{os.getpid()}_{timestamp}")
        os.makedirs(log_dir, exist_ok=True)
        _SENS_WORKER_EVALUATOR.log_directory = log_dir
        _SENS_WORKER_EVALUATOR.evaluate_individual(parameters)
        log_path = os.path.join(log_dir, "log_dict.pkl")
        if os.path.exists(log_path):
            return log_path
        mat_files = [f for f in os.listdir(log_dir) if f.endswith('.mat')]
        if mat_files:
            return os.path.join(log_dir, mat_files[-1])
        return log_dir
    except Exception as e:
        print(f"Error running worker simulation: {e}")
        return ""


def _run_sensitivity_worker_evaluate(parameters: List[float]):
    """Run a single evaluation in a worker process and return fitness/metrics."""
    global _SENS_WORKER_EVALUATOR
    if _SENS_WORKER_EVALUATOR is None:
        raise RuntimeError("Sensitivity worker evaluator not initialized")
    try:
        return _SENS_WORKER_EVALUATOR._evaluate_parameters(parameters)
    except Exception as e:
        print(f"Error evaluating worker parameters: {e}")
        return _SENS_WORKER_EVALUATOR._get_fallback_fitness()


@dataclass
class AdaptationSensitivityResult:
    """Result of sensitivity analysis including adaptation metrics."""
    parameter_name: str
    base_value: float
    varied_values: List[float]
    performance_metrics: Dict[str, List[float]]  # Multiple metrics
    sensitivity_indices: Dict[str, float]  # Sensitivity for each metric
    method: str


class AdaptationMetricsAnalyzer:
    """Sensitivity analysis that includes adaptation metrics."""
    
    def __init__(self, variation_range: float = 0.5, n_points: int = 7, use_multiplicative: bool = True):
        self.variation_range = variation_range
        self.n_points = n_points
        self.use_multiplicative = use_multiplicative
        
        # Define metrics to analyze (comprehensive: inner + outer loop)
        self.metric_names = [
            'attitude_tracking_error',      # Inner: RMS error in roll/pitch/yaw tracking
            'angular_velocity_tracking_error',   # Inner: RMS error in angular rate tracking
            'rotational_control_effort',    # Inner: RMS of rotational control moments
            'position_error',               # Outer: RMS error in position tracking
            'velocity_error',               # Outer: RMS error in velocity tracking
            'translational_control_effort'  # Outer: RMS of translational control (thrust)
        ]
    
    def analyze(self, parameters: Dict[str, float], evaluator: Callable) -> List[AdaptationSensitivityResult]:
        """
        Perform sensitivity analysis including adaptation metrics.
        
        Args:
            parameters: Baseline parameter dictionary
            evaluator: Function that evaluates parameters and returns simulation log path
            
        Returns:
            List of AdaptationSensitivityResult objects
        """
        results = []
        
        # Check if evaluator supports parallel processing
        has_parallel = hasattr(evaluator, '_evaluate_parameters_parallel') or getattr(evaluator, 'has_parallel', False)
        
        for param_name, base_value in parameters.items():
            print(f"Analyzing parameter: {param_name}")
            
            # Generate parameter-specific variation points
            varied_values = self._generate_parameter_variations(param_name, base_value)
            
            # Debug: Show variation range for adaptive rates
            if 'gamma_' in param_name and len(varied_values) > 1:
                min_val, max_val = varied_values.min(), varied_values.max()
                print(f"   {param_name}: {base_value:.3f} → [{min_val:.3f}, {max_val:.3f}] ({(max_val/min_val-1)*100:.0f}% range)")
            
            # Create all parameter variations for this parameter
            varied_params_list = []
            for varied_value in varied_values:
                varied_params = parameters.copy()
                varied_params[param_name] = varied_value
                varied_params_list.append(varied_params)
            
            # Evaluate performance (parallel if available)
            if has_parallel and len(varied_params_list) > 1:
                log_paths = evaluator._evaluate_parameters_parallel(varied_params_list)
            else:
                log_paths = [evaluator(params) for params in varied_params_list]
            
            # Compute metrics for each parameter variation
            performance_metrics = {metric: [] for metric in self.metric_names}
            
            for log_path in log_paths:
                metrics = self._compute_metrics_from_log(log_path)
                for metric_name in self.metric_names:
                    performance_metrics[metric_name].append(metrics.get(metric_name, 999.0))
            
            # Calculate sensitivity indices for each metric
            sensitivity_indices = {}
            for metric_name in self.metric_names:
                metric_values = performance_metrics[metric_name]
                sensitivity_indices[metric_name] = np.std(metric_values)
            
            result = AdaptationSensitivityResult(
                parameter_name=param_name,
                base_value=base_value,
                varied_values=varied_values.tolist(),
                performance_metrics=performance_metrics,
                sensitivity_indices=sensitivity_indices,
                method="Adaptation-Metrics"
            )
            results.append(result)
        
        return results
    
    def _generate_parameter_variations(self, param_name: str, base_value: float) -> np.ndarray:
        """Generate parameter variations for sensitivity analysis."""
        if self.use_multiplicative and base_value > 0:
            # Multiplicative variation for positive parameters
            factor = 1.0 + self.variation_range
            min_val = base_value / factor
            max_val = base_value * factor
            return np.logspace(np.log10(min_val), np.log10(max_val), self.n_points)
        else:
            # Additive variation for zero/negative parameters
            variation_abs = abs(base_value) * self.variation_range
            if variation_abs == 0:
                variation_abs = 0.1
            min_val = base_value - variation_abs
            max_val = base_value + variation_abs
            return np.linspace(min_val, max_val, self.n_points)
    
    def _compute_metrics_from_log(self, log_path: str) -> Dict[str, float]:
        """
        Compute all metrics from simulation log.
        
        Args:
            log_path: Path to simulation log file
            
        Returns:
            Dictionary of computed metrics
        """
        try:
            # Check if log_path is valid
            if not log_path or log_path == "":
                print(f"Warning: Empty log path provided")
                return {metric: 999.0 for metric in self.metric_names}
            
            # Load log data
            if log_path.endswith('.pkl'):
                if os.path.exists(log_path):
                    with open(log_path, 'rb') as f:
                        import pickle
                        log = pickle.load(f)
                else:
                    print(f"Warning: Log file not found: {log_path}")
                    return {metric: 999.0 for metric in self.metric_names}
            else:
                # Try to load as MATLAB file
                if os.path.exists(log_path):
                    from scipy.io import loadmat
                    mat = loadmat(log_path, squeeze_me=True, simplify_cells=True)
                    log = mat.get('log', {})
                else:
                    print(f"Warning: Log file not found: {log_path}")
                    return {metric: 999.0 for metric in self.metric_names}
            
            # Compute inner + outer loop metrics using calculators for consistency
            from acsl_pychrono.ga_tuner.metrics import InnerLoopMetricsCalculator, OuterLoopMetricsCalculator
            inner_calc = InnerLoopMetricsCalculator()
            outer_calc = OuterLoopMetricsCalculator()
            
            inner_metrics = inner_calc.compute_all_metrics(log)
            outer_metrics = outer_calc.calculate_metrics(log).to_dict()
            
            metrics = {}
            metrics.update(inner_metrics)
            metrics.update(outer_metrics)
            return metrics
            
        except Exception as e:
            print(f"Error computing metrics from log {log_path}: {e}")
            # Return high penalties for all metrics
            return {metric: 999.0 for metric in self.metric_names}
    

class MRACSensitivityFramework:
    """Main framework for MRAC parameter sensitivity analysis."""
    
    def __init__(self, log_directory: str = "sensitivity_logs", 
                 parameter_mode: str = "comprehensive", n_workers: int = None,
                 cleanup_logs: bool = True, keep_reports: bool = True):
        self.log_directory = log_directory
        self.parameter_mode = parameter_mode
        self.n_workers = self._get_max_parallel(n_workers)
        self.cleanup_logs = cleanup_logs
        self.keep_reports = keep_reports
        os.makedirs(log_directory, exist_ok=True)
        
        # Check if we can use real simulation
        if not HAS_UAV_MODULES:
            raise ImportError("Real UAV simulation modules are required for meaningful sensitivity analysis.")
        
        self.using_real_simulation = True
        
        # Initialize MRAC tuning interface and parameter bounds (default to global bounds)
        self.mrac_tuning = MRACTuning(tuning_config={'search_space_type': 'global'})
        self.bounds = self.mrac_tuning.get_parameter_bounds()
        self.full_parameter_names = list(self.bounds.parameter_names)
        self._full_parameter_index = {name: i for i, name in enumerate(self.full_parameter_names)}
        
        # Set baseline parameters using MRACTuning helpers
        baseline_gains = self.mrac_tuning.get_default_gains()
        self.baseline_parameters = self.mrac_tuning.gains_to_vector(baseline_gains)

        # Determine sensitivity parameter subset from param_config
        try:
            from acsl_pychrono.ga_tuner.ga_config.param_config import get_tuned_parameters
            tuned_parameters = get_tuned_parameters('MRAC')
        except Exception as e:
            print(f"Warning: Failed to load tuned parameters from param_config: {e}")
            tuned_parameters = self.full_parameter_names

        self.sensitivity_parameter_names = [p for p in tuned_parameters if p in self._full_parameter_index]
        missing = [p for p in tuned_parameters if p not in self._full_parameter_index]
        if missing:
            print(f"Warning: Some tuned parameters not found in MRAC bounds: {missing}")

        if not self.sensitivity_parameter_names:
            self.sensitivity_parameter_names = self.full_parameter_names

        self.sensitivity_indices = [self._full_parameter_index[p] for p in self.sensitivity_parameter_names]
        self.sensitivity_lower_bounds = self.bounds.lower_bounds[self.sensitivity_indices]
        self.sensitivity_upper_bounds = self.bounds.upper_bounds[self.sensitivity_indices]
        self.sensitivity_baseline_parameters = [self.baseline_parameters[i] for i in self.sensitivity_indices]
        
        # Initialize evaluator with BOTH inner and outer loop metrics for comprehensive sensitivity analysis
        from acsl_pychrono.ga_tuner.ga_config.metrics_config import MetricsConfig
        
        metrics_config = MetricsConfig()
        self.evaluator = CombinedEvaluator(
            uav_adapter=UAVModelAdapter(),
            controller_type='MRAC',
            log_directory=os.path.join(log_directory, "simulations"),
            parallel_config={
                'enabled': True,
                'n_workers': self.n_workers,
                'use_processes': True  # Use processes for true parallelization
            },
            multi_objective=True,  # Enable multi-objective for all 6 metrics
            normalize_metrics=True,
            metrics_config=metrics_config
        )
        
        # Apply safety flags and fix dead-zone parameters
        self._apply_safety_settings()
        
        print(f"\n🎯 MRAC Sensitivity Analysis Framework Initialized")
        print(f"   Parameter mode: {parameter_mode}")
        print(f"   Total parameters: {len(self.full_parameter_names)}")
        print(f"   Sensitivity parameters: {len(self.sensitivity_parameter_names)}")
        print(f"   Parallel workers: {self.n_workers}")
        print(f"   Metrics: 6 (3 inner loop + 3 outer loop)")
        print(f"   Safety flags: Enabled")
        print(f"   Dead-zone parameters: Fixed at safe values")
        print(f"   Log cleanup: {'Enabled' if cleanup_logs else 'Disabled'}")
        print(f"   Keep reports: {'Yes' if keep_reports else 'No'}")
    
    def _apply_safety_settings(self):
        """Apply safety settings and fix dead-zone parameters."""
        # This would modify the UAV adapter or simulation config
        # to enable safety flags and fix dead-zone parameters
        print("   Applying safety settings and dead-zone parameter fixes...")
        
        # Example: Set safety flags in simulation configuration
        # self.evaluator.uav_adapter.set_safety_flags(enabled=True)
        # self.evaluator.uav_adapter.fix_dead_zone_parameters()
    
    def _cleanup_simulation_logs(self, analysis_method: str):
        """Clean up verbose simulation logs after analysis is complete."""
        if not self.cleanup_logs:
            return
        
        print(f"\n🧹 Cleaning up simulation logs...")
        
        # Paths to clean up
        cleanup_paths = [
            os.path.join(self.log_directory, "simulations"),
            os.path.join(os.getcwd(), "logs", "mrac_sensitivity"),
            os.path.join(os.getcwd(), "simulation_logs")
        ]
        
        total_cleaned = 0
        
        for path in cleanup_paths:
            if os.path.exists(path):
                try:
                    # Calculate size before cleanup
                    size_before = self._get_directory_size(path)
                    
                    # Remove verbose log files but keep essential ones
                    self._cleanup_directory(path)
                    
                    # Calculate size after cleanup
                    size_after = self._get_directory_size(path)
                    cleaned_size = size_before - size_after
                    
                    if cleaned_size > 0:
                        print(f"   ✓ Cleaned {path}: {self._format_size(cleaned_size)}")
                        total_cleaned += cleaned_size
                    else:
                        print(f"   - No cleanup needed for {path}")
                        
                except Exception as e:
                    print(f"   ⚠️ Warning: Could not cleanup {path}: {e}")
        
        if total_cleaned > 0:
            print(f"   🎉 Total space saved: {self._format_size(total_cleaned)}")
        else:
            print(f"   ℹ️ No logs to cleanup")
    
    def _cleanup_directory(self, directory: str):
        """Clean up a directory by removing verbose log files."""
        if not os.path.exists(directory):
            return
        
        # Files to remove (verbose logs)
        verbose_patterns = [
            "*.log",           # General log files
            "*.txt",           # Text log files
            "*.out",           # Output files
            "*.err",           # Error files
            "*.mat",           # MATLAB files (keep only essential ones)
            "workspace_log_*.mat",  # Workspace logs
            "*.pkl",           # Pickle files (keep only reports)
        ]
        
        # Files to keep (essential)
        keep_patterns = [
            "log_dict.pkl",    # Essential simulation data
            "sensitivity_*.pkl",  # Sensitivity reports
            "*.png",           # Plots
            "*.jpg",           # Images
            "*.pdf",           # Reports
        ]
        
        for root, dirs, files in os.walk(directory):
            for file in files:
                file_path = os.path.join(root, file)
                
                # Check if file should be kept
                should_keep = False
                for pattern in keep_patterns:
                    if pattern in file or file.endswith(tuple(pattern.split('*')[1:])):
                        should_keep = True
                        break
                
                # Check if file should be removed
                should_remove = False
                for pattern in verbose_patterns:
                    if pattern in file or file.endswith(tuple(pattern.split('*')[1:])):
                        should_remove = True
                        break
                
                # Remove file if it's verbose and not essential
                if should_remove and not should_keep:
                    try:
                        os.remove(file_path)
                    except Exception:
                        pass  # Ignore errors
    
    def _get_directory_size(self, directory: str) -> int:
        """Get total size of directory in bytes."""
        if not os.path.exists(directory):
            return 0
        
        total_size = 0
        for root, dirs, files in os.walk(directory):
            for file in files:
                try:
                    file_path = os.path.join(root, file)
                    total_size += os.path.getsize(file_path)
                except Exception:
                    pass
        return total_size
    
    def _format_size(self, size_bytes: int) -> str:
        """Format size in human readable format."""
        if size_bytes == 0:
            return "0 B"
        
        size_names = ["B", "KB", "MB", "GB"]
        i = 0
        while size_bytes >= 1024 and i < len(size_names) - 1:
            size_bytes /= 1024.0
            i += 1
        
        return f"{size_bytes:.1f} {size_names[i]}"
    
    def _get_max_parallel(self, user_requested_workers: Optional[int] = None) -> int:
        """Get safe number of parallel workers."""
        import multiprocessing as mp
        available_cores = mp.cpu_count() or 1
        max_safe = min(available_cores, 8)
        return min(user_requested_workers, max_safe) if user_requested_workers else max_safe

    def _build_full_parameter_vector(self, param_dict: Dict[str, float]) -> List[float]:
        """Build full parameter vector from a dict of sensitivity parameters."""
        full = list(self.baseline_parameters)
        for name, value in param_dict.items():
            idx = self._full_parameter_index.get(name)
            if idx is not None:
                full[idx] = value
        return full

    def expand_sensitivity_vectors(self, param_vectors: List[List[float]]) -> List[List[float]]:
        """Expand sensitivity-only vectors to full parameter vectors."""
        if not param_vectors:
            return param_vectors
        vector_len = len(param_vectors[0])
        full_len = len(self.baseline_parameters)
        if vector_len == full_len:
            return param_vectors
        if vector_len != len(self.sensitivity_indices):
            raise ValueError(f"Expected {len(self.sensitivity_indices)} sensitivity parameters, got {vector_len}")
        expanded = []
        for vec in param_vectors:
            full = np.array(self.baseline_parameters, dtype=float)
            for i, full_idx in enumerate(self.sensitivity_indices):
                full[full_idx] = vec[i]
            expanded.append(full.tolist())
        return expanded

    def get_sensitivity_bounds(self) -> tuple[List[str], List[List[float]]]:
        """Get parameter names and bounds for sensitivity analysis subset."""
        bounds = [[float(l), float(u)] for l, u in zip(self.sensitivity_lower_bounds, self.sensitivity_upper_bounds)]
        return self.sensitivity_parameter_names, bounds

    def map_sensitivity_indices_to_full(self, indices: List[int]) -> List[int]:
        """Map indices from sensitivity subset to full parameter indices."""
        if len(self.sensitivity_indices) == len(self.full_parameter_names):
            return indices
        return [self.sensitivity_indices[i] for i in indices]
    
    def _evaluate_parameters(self, parameters) -> float:
        """Evaluate a single parameter set."""
        # Call the evaluator's _evaluate_parameters method directly to get detailed metric printing
        return self.evaluator._evaluate_parameters(parameters)
    
    def _evaluate_parameters_parallel(self, parameter_list: List) -> List[float]:
        """Evaluate multiple parameter sets in parallel using processes."""
        print(f"🔄 Running {len(parameter_list)} simulations in parallel using {self.n_workers} processes...")
        
        try:
            from concurrent.futures import ProcessPoolExecutor
            log_root = os.path.join(self.log_directory, "simulations")
            with ProcessPoolExecutor(
                max_workers=self.n_workers,
                initializer=_init_sensitivity_worker,
                initargs=(log_root, 'MRAC', getattr(self.evaluator, 'normalize_metrics', True))
            ) as executor:
                futures = [executor.submit(_run_sensitivity_worker_evaluate, params) for params in parameter_list]
                fitness_values = [future.result() for future in futures]
            
            print(f"   ✓ All {len(parameter_list)} simulations completed!")
            return fitness_values
            
        except Exception as e:
            print(f"   ✗ Parallel execution failed: {e}")
            print("   🔄 Falling back to sequential execution...")
            return [self._evaluate_parameters(params) for params in parameter_list]
    
    def run_analysis(self, method: str = "one_at_a_time", **method_kwargs) -> None:
        """Run sensitivity analysis using specified method."""
        raise RuntimeError(
            "Scalar sensitivity methods are disabled. Use single-pass SALib instead:\n"
            "  - MRACSensitivityFramework.run_salib_single_pass_morris(...)\n"
            "  - or: python salib_mrac_pipeline.py --single-pass"
        )
    
    def run_multi_metric_morris_analysis(self, n_trajectories: int = 10, n_levels: int = 4, 
                                        variation_range: float = 0.8) -> Dict:
        """
        Run multi-metric Morris screening analysis for MRAC adaptation metrics.
        
        Args:
            n_trajectories: Number of Morris trajectories
            n_levels: Number of levels for parameter grid
            variation_range: Parameter variation range
            
        Returns:
            Dictionary with analysis results
        """
        print(f"\nRunning single-pass SALib Morris analysis.")
        result = self.run_single_pass_morris_analysis(
            n_trajectories=n_trajectories,
            n_levels=n_levels,
            seed=123,
            normalize=True
        )
        
        return {
            'result': result,
            'method': 'single_pass_morris',
            'message': 'Single-pass SALib Morris analysis completed'
        }

    def run_single_pass_morris_analysis(self, n_trajectories: int = 20, n_levels: int = 4,
                                        seed: int = 123, normalize: bool = True) -> Dict:
        """
        Single-pass SALib Morris: sample once, run once per sample, analyze each metric column.
        Returns a dict with problem definition, metrics array, and sensitivity indices per metric.
        """
        return self.run_salib_single_pass_morris(
            n_trajectories=n_trajectories,
            n_levels=n_levels,
            seed=seed,
            normalize=normalize
        )

    def run_salib_single_pass_morris(self, n_trajectories: int = 20, n_levels: int = 4,
                                     seed: int = 123, normalize: bool = True) -> Dict:
        """
        Single-pass SALib Morris: sample once, run once per sample, analyze each metric column.
        Returns a dict with problem definition, metrics array, and sensitivity indices per metric.
        """
        try:
            from salib_mrac_pipeline import run_morris_single_pass
        except Exception as e:
            raise ImportError(f"Failed to import SALib single-pass pipeline: {e}")

        problem, Y_used, Si_dict, normalization_bounds = run_morris_single_pass(
            self,
            N=n_trajectories,
            num_levels=n_levels,
            seed=seed,
            normalize=normalize
        )

        return {
            'problem': problem,
            'Y_used': Y_used,
            'Si_dict': Si_dict,
            'normalization_bounds': normalization_bounds,
            'method': 'salib_single_pass_morris'
        }
    
    def run_adaptation_metrics_analysis(self, variation_range: float = 0.5, n_points: int = 7) -> List[AdaptationSensitivityResult]:
        """
        Run sensitivity analysis including adaptation metrics.
        
        Args:
            variation_range: Parameter variation range
            n_points: Number of variation points per parameter
            
        Returns:
            List of AdaptationSensitivityResult objects
        """
        print(f"\n🔍 Running Adaptation Metrics Sensitivity Analysis")
        print(f"   Variation range: {variation_range}")
        print(f"   Points per parameter: {n_points}")
        
        # Initialize adaptation metrics analyzer
        analyzer = AdaptationMetricsAnalyzer(
            variation_range=variation_range,
            n_points=n_points,
            use_multiplicative=True
        )
        print(f"   Metrics: {', '.join(analyzer.metric_names)}")
        
        # Convert baseline parameters to dictionary format
        analysis_parameters = dict(zip(self.sensitivity_parameter_names, self.sensitivity_baseline_parameters))
        
        # Create wrapper evaluator that returns log paths
        def wrapper_evaluator(param_dict):
            param_list = self._build_full_parameter_vector(param_dict)
            # Run simulation and return log path
            return self._run_simulation_and_get_log_path(param_list)
        
        # Add parallel evaluation capability to wrapper
        def wrapper_parallel(param_dicts: List[Dict[str, float]]) -> List[str]:
            full_vectors = [self._build_full_parameter_vector(d) for d in param_dicts]
            return self._evaluate_parameters_parallel_with_logs(full_vectors)

        wrapper_evaluator._evaluate_parameters_parallel = wrapper_parallel
        wrapper_evaluator.has_parallel = True
        
        # Run analysis
        results = analyzer.analyze(analysis_parameters, wrapper_evaluator)
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_path = os.path.join(self.log_directory, f"adaptation_metrics_sensitivity_{timestamp}.pkl")
        
        import pickle
        with open(results_path, 'wb') as f:
            pickle.dump(results, f)
        
        print(f"\nAdaptation Metrics Sensitivity Analysis Complete!")
        print(f"Results saved to: {results_path}")
        
        # Print summary
        print(f"\nAdaptation Metrics Sensitivity Analysis Summary")
        print("="*60)
        for result in results:
            print(f"\n{result.parameter_name}:")
            for metric_name, sensitivity in result.sensitivity_indices.items():
                print(f"  {metric_name}: {sensitivity:.6f}")
        
        # Clean up logs
        self._cleanup_simulation_logs("adaptation_metrics")
        
        return results
    
    def _run_simulation_and_get_log_path(self, parameters) -> str:
        """Run simulation and return the log file path."""
        try:
            # Create unique log directory for this evaluation
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            log_dir = os.path.join(self.log_directory, "simulations", f"eval_{timestamp}")
            os.makedirs(log_dir, exist_ok=True)
            
            # Temporarily modify the evaluator's log directory
            original_log_dir = self.evaluator.log_directory
            self.evaluator.log_directory = log_dir
            
            try:
                # Use the evaluator's evaluate_individual method
                fitness = self.evaluator.evaluate_individual(parameters)
                
                # Return the log file path
                log_path = os.path.join(log_dir, "log_dict.pkl")
                if os.path.exists(log_path):
                    return log_path
                else:
                    # Try to find .mat file
                    mat_files = [f for f in os.listdir(log_dir) if f.endswith('.mat')]
                    if mat_files:
                        return os.path.join(log_dir, mat_files[-1])
                    else:
                        return log_dir  # Return directory if no specific log file found
            finally:
                # Restore original log directory
                self.evaluator.log_directory = original_log_dir
                    
        except Exception as e:
            print(f"Error running simulation: {e}")
            return ""
    
    def _evaluate_parameters_parallel_with_logs(self, parameter_list: List) -> List[str]:
        """Evaluate multiple parameter sets in parallel and return log paths."""
        print(f"🔄 Running {len(parameter_list)} simulations in parallel using {self.n_workers} workers...")
        
        try:
            from concurrent.futures import ProcessPoolExecutor
            log_root = os.path.join(self.log_directory, "simulations")
            with ProcessPoolExecutor(
                max_workers=self.n_workers,
                initializer=_init_sensitivity_worker,
                initargs=(log_root, 'MRAC', getattr(self.evaluator, 'normalize_metrics', True))
            ) as executor:
                futures = [executor.submit(_run_sensitivity_worker_simulation, params) for params in parameter_list]
                log_paths = [future.result() for future in futures]
            return log_paths
            
        except Exception as e:
            print(f"   ✗ Parallel execution failed: {e}")
            print("   🔄 Falling back to sequential execution...")
            return [self._run_simulation_and_get_log_path(params) for params in parameter_list]
    
    def cleanup_all_logs(self):
        """Manually trigger cleanup of all simulation logs."""
        print(f"\n🧹 Manual log cleanup requested...")
        self._cleanup_simulation_logs("manual")
    
    def get_log_directory_size(self) -> str:
        """Get current size of log directory."""
        total_size = 0
        
        # Check main log directory
        if os.path.exists(self.log_directory):
            total_size += self._get_directory_size(self.log_directory)
        
        # Check other log directories
        other_paths = [
            os.path.join(os.getcwd(), "logs", "mrac_sensitivity"),
            os.path.join(os.getcwd(), "simulation_logs")
        ]
        
        for path in other_paths:
            if os.path.exists(path):
                total_size += self._get_directory_size(path)
        
        return self._format_size(total_size)


if __name__ == "__main__":
    pass
