# GA Tuner Module

Genetic algorithm-based parameter tuning for UAV control systems (PID and MRAC variants).

## Installation

```bash
pip install numpy scipy deap pymoo
```

Also requires `pychrono` - see project installation guide.

## Quick Start

```bash
python acsl_pychrono/ga_tuner/run_tuner.py
```

Or in Python:

```python
from acsl_pychrono.ga_tuner import create_uav_ga_tuner, summarize_tuner_result, GAConfig

tuner = create_uav_ga_tuner(ga_config=GAConfig())
result = tuner.optimize()
summarize_tuner_result(tuner, result)
```

## Configuration

All user-defined knobs live in `ga_config/`. Change these to customize the tuner with the controller, parameters, and simulation you want:

| File | Purpose |
|------|---------|
| `ga_config.py` | Algorithm (DEAP/PYMOO), population size, generations, controller type, search space type |
| `param_config.py` | Which parameters to tune (per-controller selections) |
| `evaluator_config.py` | Evaluator type (inner/outer/combined), parallel workers, log directory |
| `metrics_config.py` | Single vs multi-objective, metric weights, normalization |
| `mission_config.py` | Trajectory, duration, visualization flags |

### Pick a controller
- Set `controller_type` in `ga_config.py` to one of:
  - `"PID"`, `"MRAC"`, `"TwoLayerMRAC"`, `"HybridTwoLayerMRAC"`, `"HybridMRAC"`, `"FunnelMRAC"`, `"NonAdaptiveEBCI"`.
- Use the same key in `param_config.py` under `TUNING_SELECTIONS` to select parameters for that controller.
- Choose `EVALUATOR_TYPE` in `evaluator_config.py` — `"inner"` (attitude), `"outer"` (position), or `"combined"` (all metrics).

### Choose which parameters to tune
`param_config.py` contains `TUNING_SELECTIONS` and is expanded by `param_utils.py`:
- PID: use `"all"`, `"translational"`, `"rotational"`, `"proportional"`, `"integral"`, `"derivative"`, a list of parameter names, or `{"groups": [...]}` combining groups from `PID_PARAMETER_GROUPS`.
- MRAC-family (SPD matrices): select per-matrix using `"full"`, `"diagonal"`, `{"diagonal": [mask...]}`, or `{"selection_matrix": [...]}` (lower-triangular masks).
- The resulting parameter subset is used automatically by `TUNING_CONFIG`.

### Optimization mode
- Set `multi_objective` in `metrics_config.py`:
  - `True` → returns a Pareto front (multiple trade-off solutions)
  - `False` → weighted sum single objective (weights configured in `metrics_config.py`)

### Search space & normalization
- `ga_config.search_space_type` controls bounds for MRAC-family tuners:
  - `"local"` → bounds relative to reference solution (Default: hand-tuned gains from existing controller)
  - `"global"` → absolute bounds
  - bounds for adaptive gains are defined in `ga_tuner/tuners/base_mrac_tuning.py`
- Normalization uses a reference evaluation (default hand-tuned gains) when enabled.

## Output

Multi-objective mode returns a Pareto front, for example, with combined metrics evaluator, it will return:
```

Pareto front (1 solutions):
  Solution 1:
    Tuned values: [-1.4193, -0.8763, ...]
    Combined metrics [attitude_error, angular_rate_error, rotational_effort, position_error, velocity_error, translational_effort]: [0.0123, 0.1676, 0.1585, 0.0561, 0.0681, 7.2547]
  Solution 2:  
    ...
```

Single-objective mode returns one best solution with a scalar fitness value.

## Troubleshooting

**RuntimeWarning: overflow/invalid value** - Expected during exploration. Bad parameters get penalty values (`inf`).

**Large control effort values** - Numerical instability from unstable parameters. Automatically penalized.
