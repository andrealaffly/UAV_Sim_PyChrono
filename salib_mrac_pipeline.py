"""
SALib MRAC sensitivity analysis (single-pass Morris + Sobol subset).
Runs one simulation per sample, returns 6 metrics, and analyzes each metric column.
"""

import argparse
import os
import sys
from typing import List, Tuple, Dict
from datetime import datetime

import numpy as np

try:
    from SALib.sample import morris as morris_sampler
    from SALib.analyze import morris as morris_analyze
    from SALib.sample import sobol as sobol_sampler
    from SALib.analyze import sobol as sobol_analyze
    HAS_SALIB = True
except ImportError:
    HAS_SALIB = False

# Add project root to path for imports when executed directly
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from mrac_sensitivity_analysis import MRACSensitivityFramework

# Comprehensive metrics: Inner loop (attitude) + Outer loop (position/trajectory)
METRIC_NAMES = [
    # Inner loop metrics (attitude/rotational control)
    'attitude_tracking_error',           # RMS error in roll/pitch/yaw tracking
    'angular_velocity_tracking_error',   # RMS error in angular rate tracking
    'rotational_control_effort',         # RMS of control moments (u2, u3, u4)
    # Outer loop metrics (position/translational control)
    'position_error',                    # RMS error in position tracking (x, y, z)
    'velocity_error',                    # RMS error in velocity tracking
    'translational_control_effort'       # RMS of translational control (thrust u1)
]

def _extract_bounds(framework: MRACSensitivityFramework) -> Tuple[List[str], List[List[float]]]:
    """Build SALib problem bounds from MRACSensitivityFramework bounds."""
    if hasattr(framework, 'get_sensitivity_bounds'):
        try:
            return framework.get_sensitivity_bounds()
        except Exception as e:
            print(f"Warning: Failed to get sensitivity bounds from framework: {e}")
    names = list(framework.bounds.parameter_names)

    # Case 1: attributes lower_bounds / upper_bounds
    lows = getattr(framework.bounds, 'lower_bounds', None)
    highs = getattr(framework.bounds, 'upper_bounds', None)
    if lows is not None and highs is not None:
        bounds = [[float(l), float(h)] for l, h in zip(lows, highs)]
        return names, bounds

    # Case 2: attribute bounds or iterable of tuples
    maybe = getattr(framework.bounds, 'bounds', None)
    if maybe is not None:
        try:
            bounds = [[float(l), float(h)] for (l, h) in maybe]
            return names, bounds
        except Exception as e:
            print(f"Warning: Failed to extract bounds from framework.bounds.bounds: {e}")
            print("Falling back to baseline-based bounds estimation")

    # Fallback: use +/- 50% around baseline
    base = list(framework.baseline_parameters)
    bounds = []
    for v in base:
        if v == 0:
            bounds.append([-0.1, 0.1])
        else:
            r = abs(v) * 0.5
            bounds.append([float(v - r), float(v + r)])
    return names, bounds


def _evaluate_list(framework: MRACSensitivityFramework, param_vectors: List[List[float]]) -> np.ndarray:
    """
    Evaluate many parameter vectors, using framework parallel if available.
    Returns array of shape (n_samples, 6) for combined inner+outer loop metrics.
    """
    if hasattr(framework, 'expand_sensitivity_vectors'):
        try:
            param_vectors = framework.expand_sensitivity_vectors(param_vectors)
        except Exception as e:
            print(f"Warning: Failed to expand sensitivity vectors: {e}")
    try:
        results = framework._evaluate_parameters_parallel(param_vectors)
    except Exception as e:
        print(f"Warning: Parallel evaluation failed ({e}), falling back to sequential evaluation")
        try:
            results = [framework._evaluate_parameters(vec) for vec in param_vectors]
        except Exception as e2:
            print(f"Error: Sequential evaluation also failed: {e2}")
            raise RuntimeError(f"Both parallel and sequential parameter evaluation failed. Parallel error: {e}, Sequential error: {e2}")
    
    # Convert to numpy array - should be shape (n_samples, 6) for combined evaluator
    results_array = np.array(results)
    
    if results_array.ndim == 1:
        results_array = results_array.reshape(-1, 1)

    if results_array.shape[1] != len(METRIC_NAMES):
        raise RuntimeError(
            f"Expected {len(METRIC_NAMES)} metrics per sample, got shape {results_array.shape}. "
            "Ensure CombinedEvaluator is in multi-objective mode."
        )
    
    return results_array


def normalize_metrics(metrics_array: np.ndarray, normalization_bounds=None) -> Tuple[np.ndarray, Dict]:
    """
    Normalize metrics using min-max normalization to [0, 1].
    
    Args:
        metrics_array: Array of shape (n_samples, n_metrics)
        normalization_bounds: Optional dict with 'mins' and 'maxs' arrays to use for normalization.
                            If None, compute from metrics_array.
        
    Returns:
        Tuple of (normalized array, normalization_bounds dict)
    """
    normalized = np.zeros_like(metrics_array, dtype=float)
    n_metrics = metrics_array.shape[1]
    
    # Compute or use provided normalization bounds
    if normalization_bounds is None:
        mins = np.zeros(n_metrics)
        maxs = np.zeros(n_metrics)
        
        print("\n[NORMALIZATION] Computing normalization bounds:")
        for i in range(n_metrics):
            # Filter out inf values
            valid_mask = ~np.isinf(metrics_array[:, i])
            valid_values = metrics_array[valid_mask, i]
            
            if len(valid_values) > 0:
                mins[i] = np.min(valid_values)
                maxs[i] = np.max(valid_values)
                metric_range = maxs[i] - mins[i]
                
                metric_name = METRIC_NAMES[i] if i < len(METRIC_NAMES) else f"metric_{i}"
                print(f"  {metric_name:40s}: [{mins[i]:.6f}, {maxs[i]:.6f}] (range={metric_range:.6e})")
            else:
                # All inf, use dummy bounds
                mins[i] = 0.0
                maxs[i] = 1.0
                metric_name = METRIC_NAMES[i] if i < len(METRIC_NAMES) else f"metric_{i}"
                print(f"  {metric_name:40s}: All values are inf")
        
        normalization_bounds = {'mins': mins, 'maxs': maxs}
    else:
        mins = normalization_bounds['mins']
        maxs = normalization_bounds['maxs']
        print("\n[NORMALIZATION] Using provided normalization bounds")
    
    # Apply normalization
    small_variance_count = 0
    for i in range(n_metrics):
        metric_range = maxs[i] - mins[i]
        
        if metric_range > 1e-10:
            # Standard min-max normalization
            normalized[:, i] = (metrics_array[:, i] - mins[i]) / metric_range
        else:
            # Range is too small - metrics don't vary, just use raw values
            # This prevents division by zero and preserves relative differences
            metric_name = METRIC_NAMES[i] if i < len(METRIC_NAMES) else f"metric_{i}"
            print(f"  WARNING: {metric_name} has near-zero variance - using raw values instead of normalization")
            normalized[:, i] = metrics_array[:, i]
            small_variance_count += 1
        
        # Handle inf values: set to 1.0 (worst case in normalized space)
        inf_mask = np.isinf(metrics_array[:, i])
        if np.any(inf_mask):
            normalized[inf_mask, i] = 1.0
    
    # Warning about suspicious results
    if small_variance_count > 0:
        print(f"\n⚠️  WARNING: {small_variance_count}/{n_metrics} metrics have near-zero variance!")
        print("   This may indicate:")
        print("   1. Parameter variations are too small to affect the metrics")
        print("   2. Sample size (N) is too small - try increasing --morris-N and --sobol-n")
        print("   3. The parameters being tested don't significantly affect these metrics")
        print("   → Sensitivity indices may be unreliable. Consider increasing sample sizes.")
    
    return normalized, normalization_bounds


def _build_problem(names: List[str], bounds: List[List[float]]) -> dict:
    return {
        "num_vars": len(names),
        "names": names,
        "bounds": bounds,
    }


def run_morris_single_pass(framework: MRACSensitivityFramework, N: int, num_levels: int, seed: int = 123,
                           normalize: bool = True) -> Tuple[dict, np.ndarray, Dict[str, dict], Dict]:
    """
    Single-pass Morris workflow:
      1) Sample once (Morris)
      2) Run one simulation per sample, collect 6 metrics per run
      3) Analyze each metric column independently with SALib.analyze.morris

    Returns:
        problem: SALib problem definition
        Y_used: Metrics array (normalized if requested) of shape (n_samples, 6)
        Si_dict: Dictionary mapping metric names to Morris sensitivity results
        normalization_bounds: Dict of normalization bounds (empty if normalize=False)
    """
    names, bounds = _extract_bounds(framework)
    problem = _build_problem(names, bounds)

    print("\n" + "="*80)
    print("SINGLE-PASS MORRIS: GENERATE SAMPLES")
    print("="*80)

    X = morris_sampler.sample(problem, N=N, num_levels=num_levels, optimal_trajectories=None,
                              local_optimization=False, seed=seed)
    print(f"Generated {len(X)} parameter sets for Morris screening")

    print("\n" + "="*80)
    print("SINGLE-PASS MORRIS: RUN SIMULATIONS (ONE RUN PER SAMPLE)")
    print("="*80)

    Y_raw = _evaluate_list(framework, X.tolist())
    print(f"Collected metrics from {len(Y_raw)} simulations")
    print(f"Metrics shape: {Y_raw.shape}")

    normalization_bounds = {}
    if normalize:
        print("\n" + "="*80)
        print("SINGLE-PASS MORRIS: NORMALIZE METRICS")
        print("="*80)
        Y_used, normalization_bounds = normalize_metrics(Y_raw)
    else:
        Y_used = Y_raw

    print("\n" + "="*80)
    print("SINGLE-PASS MORRIS: ANALYZE EACH METRIC COLUMN")
    print("="*80)

    Si_dict = {}
    for i, metric_name in enumerate(METRIC_NAMES):
        print(f"\n--- Morris Analysis: {metric_name} ---")
        Y_metric = Y_used[:, i]
        Si = morris_analyze.analyze(problem, X, Y_metric, conf_level=0.95,
                                    print_to_console=False, num_levels=num_levels)
        Si_dict[metric_name] = Si

        mu_star = np.asarray(Si.get("mu_star", []))
        sigma = np.asarray(Si.get("sigma", []))
        order = np.argsort(mu_star)[::-1]
        print(f"Top 5 parameters by μ*:")
        for rank, idx in enumerate(order[:5], 1):
            print(f"  {rank}. {names[idx]:30s}: μ*={mu_star[idx]:.6f}, σ={sigma[idx]:.6f}")

    return problem, Y_used, Si_dict, normalization_bounds


def select_top_by_mu_star(Si: dict, names: List[str], top_k: int) -> List[int]:
    """Select top-k parameters by Morris mu_star."""
    mu_star = np.asarray(Si.get("mu_star", []))
    ranks = np.argsort(mu_star)[::-1]
    return list(ranks[: max(1, min(top_k, len(names)))])


def run_sobol_subset_multi_metric(framework: MRACSensitivityFramework, full_problem: dict, selected_indices: List[int], sobol_n: int, normalization_bounds: Dict) -> Tuple[dict, np.ndarray, Dict[str, dict]]:
    """
    Run Sobol analysis on selected parameters for combined inner+outer loop metrics.
    
    Args:
        framework: Sensitivity analysis framework
        full_problem: Full SALib problem definition
        selected_indices: Indices of selected parameters
        sobol_n: Number of Sobol samples
        normalization_bounds: Normalization bounds from Morris phase (for consistency)
    
    Returns:
        sub_problem: SALib problem for selected parameters
        Y_normalized: Normalized metrics array
        Si_dict: Dictionary mapping metric names to Sobol sensitivity results
    """
    names = full_problem["names"]
    bounds = full_problem["bounds"]

    sub_names = [names[i] for i in selected_indices]
    sub_bounds = [bounds[i] for i in selected_indices]

    sub_problem = _build_problem(sub_names, sub_bounds)

    print("\n" + "="*80)
    print("STEP 5: RUN SOBOL ANALYSIS ON SELECTED PARAMETERS")
    print("="*80)
    print(f"Selected parameters: {sub_names}")
    
    # Sample in subspace
    X_sub = sobol_sampler.sample(sub_problem, sobol_n, calc_second_order=True)
    print(f"\nGenerated {len(X_sub)} Sobol samples")

    # Lift to full parameter vectors using baseline for non-selected
    baseline = np.array(framework.baseline_parameters, dtype=float)
    X_full = np.tile(baseline, (X_sub.shape[0], 1))
    if hasattr(framework, 'map_sensitivity_indices_to_full'):
        full_indices = framework.map_sensitivity_indices_to_full(selected_indices)
    else:
        full_indices = selected_indices
    for col_idx, full_idx in enumerate(full_indices):
        X_full[:, full_idx] = X_sub[:, col_idx]

    print("\nRunning simulations...")
    Y_raw = _evaluate_list(framework, X_full.tolist())
    
    if normalization_bounds is None:
        print("\nNormalizing metrics from Sobol run (no Morris bounds)...")
    else:
        print("\nNormalizing metrics with Morris bounds (for consistency)...")
    Y_normalized, _ = normalize_metrics(Y_raw, normalization_bounds=normalization_bounds)

    print("\nComputing Sobol indices for each metric...")
    Si_dict = {}
    for i, metric_name in enumerate(METRIC_NAMES):
        print(f"\n--- Sobol Analysis: {metric_name} ---")
        
        Y_metric = Y_normalized[:, i]
        
        Si = sobol_analyze.analyze(sub_problem, Y_metric, calc_second_order=True, print_to_console=False)
        Si_dict[metric_name] = Si
        
        # Print results
        s1 = np.asarray(Si.get('S1', []))
        st = np.asarray(Si.get('ST', []))
        
        print(f"First-order (S1) and Total (ST) indices:")
        for j, name in enumerate(sub_names):
            print(f"  {name:30s}: S1={s1[j]:.6f}, ST={st[j]:.6f}")
    
    return sub_problem, Y_normalized, Si_dict


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Comprehensive MRAC Sensitivity Analysis: analyzes how MRAC parameters affect both inner and outer loop performance",
        epilog="Note: Uses CombinedEvaluator to measure 6 metrics across attitude and trajectory tracking!"
    )
    parser.add_argument('--output-dir', default='mrac_sensitivity_results', help='Output directory for results')
    parser.add_argument('--morris-N', type=int, default=20, help='Morris N (base trajectories/samples)')
    parser.add_argument('--num-levels', type=int, default=4, help='Morris grid levels (default: 4)')
    parser.add_argument('--top-k', type=int, default=8, help='Select top-k parameters for Sobol')
    parser.add_argument('--sobol-n', type=int, default=256, help='Sobol base N')
    parser.add_argument('--parameter-mode', choices=['comprehensive', 'adaptive_only', 'reference_only', 'pid_only', 'modification_only'], default='comprehensive')
    parser.add_argument('--n-workers', type=int, default=None, help='Parallel workers (default auto)')
    parser.add_argument('--no-cleanup', action='store_true', help='Disable log cleanup after run')
    parser.add_argument('--no-normalize', action='store_true',
                       help='Disable metric normalization in single-pass mode')
    parser.add_argument('--morris-only', action='store_true',
                       help='Run Morris only and skip Sobol analysis')
    parser.add_argument('--sobol-only', action='store_true',
                       help='Run Sobol only (no Morris). Uses param_config selection by default')
    parser.add_argument('--sobol-from-param-config', action='store_true',
                       help='Use all parameters from param_config.py for Sobol (no top-k selection)')
    parser.add_argument('--selection-metric', choices=METRIC_NAMES, default='position_error',
                       help='Which metric to use for parameter selection (default: position_error)')
    args = parser.parse_args()

    if not HAS_SALIB:
        print('SALib is not installed. Install with: pip install SALib')
        return 1

    os.makedirs(args.output_dir, exist_ok=True)

    try:
        framework = MRACSensitivityFramework(
            log_directory=args.output_dir,
            parameter_mode=args.parameter_mode,
            n_workers=args.n_workers,
            cleanup_logs=not args.no_cleanup,
            keep_reports=True,
        )
    except ImportError as e:
        print(f"Failed to init MRACSensitivityFramework: {e}")
        return 1

    if args.sobol_only:
        print('\n' + '='*80)
        print('SOBOL-ONLY MRAC SENSITIVITY ANALYSIS')
        print('Analyzing: parameters from param_config.py')
        print('Metrics: 6 comprehensive metrics (3 inner loop + 3 outer loop)')
        print('='*80)
        names, bounds = _extract_bounds(framework)
        problem = _build_problem(names, bounds)
        Si_morris_dict = None
        Y_normalized = None
        normalization_bounds = None
    else:
        # Morris screening (single-pass)
        print('\n' + '='*80)
        print('COMPREHENSIVE MRAC SENSITIVITY ANALYSIS')
        print('Analyzing: ALL MRAC parameters (inner + outer loop)')
        print('Metrics: 6 comprehensive metrics (3 inner loop + 3 outer loop)')
        print('='*80)

        problem, Y_normalized, Si_morris_dict, normalization_bounds = run_morris_single_pass(
            framework,
            N=args.morris_N,
            num_levels=args.num_levels,
            normalize=not args.no_normalize
        )

        names = problem['names']
        
        # Print summary for selected metric
        print(f"\n{'='*80}")
        print(f"PARAMETER SELECTION BASED ON: {args.selection_metric}")
        print('='*80)
        
        Si_selected = Si_morris_dict[args.selection_metric]
        mu = np.asarray(Si_selected.get('mu_star', []))
        mu_conf = np.asarray(Si_selected.get('mu_star_conf', []))

        print(f'\nMorris μ* for {args.selection_metric} (ranked):')
        order = np.argsort(mu)[::-1]
        for r, idx in enumerate(order[:20], 1):  # Show top 20
            print(f"{r:2d}. {names[idx]:<30} μ*={mu[idx]: .6f} ± {mu_conf[idx]:.6f}")

    if args.morris_only:
        print("\nMorris-only mode enabled: skipping Sobol analysis.")
        sub_problem = None
        Y_sobol_normalized = None
        Si_sobol_dict = None
        top_indices = []
    else:
        if args.sobol_only or args.sobol_from_param_config:
            top_indices = list(range(len(problem['names'])))
            print(f'\nUsing all {len(top_indices)} parameters from param_config.py for Sobol analysis')
        else:
            # Select top-k based on selected metric
            top_indices = select_top_by_mu_star(Si_selected, names, args.top_k)
            print(f'\nTop {args.top_k} parameters selected for Sobol analysis:')
            for i, idx in enumerate(top_indices, 1):
                print(f"  {i}. {names[idx]}")

        # Sobol on subset for all metrics (using same normalization bounds as Morris)
        sub_problem, Y_sobol_normalized, Si_sobol_dict = run_sobol_subset_multi_metric(
            framework, problem, top_indices, args.sobol_n, normalization_bounds
        )

    # Save results
    try:
        import pickle
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if not args.sobol_only:
            morris_path = os.path.join(args.output_dir, f'salib_morris_multi_metric_{timestamp}.pkl')
            with open(morris_path, 'wb') as f:
                pickle.dump({
                    'problem': problem,
                    'Y_normalized': Y_normalized,
                    'Si_dict': Si_morris_dict,
                    'metric_names': METRIC_NAMES
                }, f)
            print(f"\n✓ Morris results saved to {morris_path}")

        if not args.morris_only:
            sobol_path = os.path.join(args.output_dir, f'salib_sobol_multi_metric_{timestamp}.pkl')
            with open(sobol_path, 'wb') as f:
                pickle.dump({
                    'problem': sub_problem,
                    'Y_normalized': Y_sobol_normalized,
                    'Si_dict': Si_sobol_dict,
                    'selected_indices': top_indices,
                    'selected_parameter_names': [problem['names'][i] for i in top_indices],
                    'selection_metric': args.selection_metric,
                    'metric_names': METRIC_NAMES
                }, f)
            print(f"✓ Sobol results saved to {sobol_path}")

        print(f"\n✓ All results saved in {args.output_dir}")
    except Exception as e:
        print(f"Error: Failed to save results to pickle files: {e}")
        print("Results are still available in memory but not persisted")

    # Check if results look suspicious
    # Sobol data quality check removed by request.
    
    # Check normalization ranges
    ranges = normalization_bounds['maxs'] - normalization_bounds['mins']
    if np.any(ranges < 1e-6):
        print("\n ! Some metrics have very small variance")
        print("   This reduces sensitivity analysis reliability.")
        print("   Consider using different parameters or increasing parameter variation range.")
    
    # Optional cleanup
    if not args.no_cleanup:
        framework._cleanup_simulation_logs('salib_mrac')

    return 0


if __name__ == '__main__':
    sys.exit(main())
