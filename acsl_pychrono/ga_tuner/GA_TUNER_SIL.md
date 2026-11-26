# GA Tuner - Architecture and SIL Integration

_Author: Giorgio A. Orlando - 2026-07-24_

This document covers two things:

1. A map of the `ga_tuner/` package's architecture (algorithms, tuners,
   evaluators, config, metrics, encoding) - useful context for anyone working
   in this module, SIL or not.
2. The **SIL-integration subsystem** added under `integration/sil/` that lets
   the GA optimize gains against the *real C++ flightstack* running in Docker,
   instead of only the Python controller reimplementation. This is new work
   (all files below were added or touched between 2026-07-14 and 2026-07-20)
   and **supersedes** the "GA tuning can't drive SIL yet" limitation
   previously described in
   [`SIL_USAGE_GUIDE.md`](../../../../SIL_USAGE_GUIDE.md) (now corrected to
   point here).

For the general non-SIL quick-start (install, `TUNING_SELECTIONS`,
multi-objective vs. single-objective, output format), see
[`README.md`](README.md) in this same directory - this document assumes you've
read that first and focuses on what's specific to SIL mode.

## 1. Why this exists

Before this work, the GA tuner had two evaluation paths:

- **Default (pure-Python)**: `uav_integration.UAVModelAdapter` instantiates
  candidate gains into the Python controller reimplementations under
  `acsl_pychrono/control/` and evaluates entirely in-process. Fast, trivially
  parallelizable, but only ever validates the *Python re-implementation* of a
  controller, not the real C++ one that flies the hardware.
- **A dead-end SIL code path**: `simulate_mission.py` already supported
  `UAV_SIL_MODE=true`, swapping in `FlightStackServiceBridge` to drive the
  real C++ flightstack over `/sil/compute_control` - but the bridge only
  exchanges state and motor commands, not gains. If the GA tuner had simply
  set `UAV_SIL_MODE=true`, every candidate would silently re-score whatever
  gains were already baked into the running container at process start,
  making the "optimization" a no-op with zero fitness variance.

**What was added**: a **restart-per-candidate** mechanism. Before each
candidate's simulation, the GA writes that candidate's gains into
`gains_pid.json` and restarts the `flightstack` Docker container so the C++
process picks them up fresh, then drives the run through the existing
`FlightStackServiceBridge` exactly like a normal SIL run. This is inherently
serial (one shared container) and, for now, PID-only - see §6 for the exact
scope limits.

## 2. Package architecture (all of `ga_tuner/`)

```
ga_tuner/
├── README.md                 general usage/quick-start (non-SIL focus)
├── GA_TUNER_SIL.md            this file
├── __init__.py                registries: tuning_classes, evaluator_classes; instantiateTuner()
├── run_tuner.py                entry point: pure-Python evaluation
├── run_tuner_sil.py            entry point: real SIL evaluation (NEW)
├── algorithms/                 GA backends, all implementing BaseGATuner
│   ├── base_ga.py                 abstract loop: init → select → crossover → mutate → survive
│   ├── deap_ga.py                  DEAP backend (blend/SBX/uniform crossover, gaussian/polynomial/uniform mutation)
│   ├── pymoo_ga.py                 pymoo backend (NSGA2/NSGA3/SPEA2/MOEAD/SMS-EMOA/RVEA) - the default
│   └── seeded_sampling.py          injects hand-tuned gain vectors as starting individuals
├── core/                       backend-agnostic abstractions
│   ├── controller_tuning.py        ControllerTuningInterface - the contract every tuner implements
│   ├── fitness_evaluator.py        FitnessEvaluator - caching + optional parallel evaluation
│   ├── optimization_result.py      OptimizationResult - history, convergence/diversity, Pareto front, persistence
│   ├── parameter_bounds.py         ParameterBounds - the gene space (lower/upper bounds, names, groups, random sampling)
│   └── tuning_factory.py           legacy/alternate factory (superseded by integration/ga_utils/registry.py)
├── tuners/                     per-controller encode/decode + bounds (ControllerTuningInterface impls)
│   ├── PID/pid_tuning.py           18 scalar KP/KI/KD gains (tran x/y/z, rot roll/pitch/yaw)
│   ├── base_mrac_tuning.py         shared base for MRAC/HybridMRAC/FunnelMRAC/NonAdaptiveEBCI
│   ├── MRAC/, HybridMRAC/, FunnelMRAC/, NonAdaptiveEBCI/   BaseMRACTuning subclasses
│   └── TwoLayerMRAC/, HybridTwoLayerMRAC/                  duplicate BaseMRACTuning's logic independently (predate it)
├── evaluators/                  run a simulation, score it against metrics
│   ├── base_evaluator.py           UAVSimulationEvaluator - shared simulate/log/survival-penalty plumbing
│   ├── inner_loop_evaluator.py     attitude/angular-rate/rotational-effort metrics
│   ├── outer_loop_evaluator.py     position/velocity/translational-effort metrics (the default, and the only one wired for SIL)
│   ├── combined_evaluator.py       inner + outer together (6 objectives)
│   └── survival_penalty.py         penalizes crashed sims: penalty = base * (1 + time_deficit_ratio), range [base, 2*base]
├── ga_config/                    user-edited configuration
│   ├── ga_config.py                GAConfig: algorithm, population_size, num_generations, crossover/mutation rates, seeding, survival penalty
│   ├── param_config.py             TUNING_SELECTIONS: which parameters to tune per controller
│   ├── param_utils.py              expands "translational"/"all"/group-dict selections into parameter name lists
│   ├── evaluator_config.py         EVALUATOR_TYPE, LOG_DIRECTORY, PARALLEL_CONFIG
│   ├── metrics_config.py           multi_objective flag, per-metric weights, normalization
│   ├── mission_config.py           trajectory, duration, visualization for the GA's own runs
│   └── config.py                   aggregates all of the above into TuningConfig / TUNING_CONFIG
├── integration/                  wires the above into an actual simulation
│   ├── uav_integration.py          create_uav_ga_tuner() - the central factory; UAVModelAdapter (pure-Python)
│   ├── controller_factory.py       instantiateControllerWithGA() - applies a GA vector to a real controller gains object
│   ├── ga_utils/                   get_tuning_instance(), build_parameter_vector(), apply_gain_dict()
│   └── sil/                        SIL restart-per-candidate bridge (NEW - see §3-5)
├── encoding/                     GA-vector ↔ gain-matrix codecs
│   ├── cholesky_utils.py           Cholesky parameterization so mutation/crossover can't produce a non-SPD matrix
│   └── parameter_converter.py      ControllerParameterConverter - auto-discovers SPD/diagonal/scalar params from a gains dict
└── metrics/                      pure metric computation from a simulation log
    ├── attitude_utils.py / inner_loop_metrics.py     RMS attitude & angular-rate tracking error, rotational effort
    ├── translational_utils.py / outer_loop_metrics.py RMS position & velocity tracking error, translational effort
    └── normalizer.py                MetricNormalizer - reference-relative normalization with a cap, for multi-objective fairness across metrics of different scale
```

Data flow for a single candidate, pure-Python path (`run_tuner.py`):
`ga_config/*.py` → `create_uav_ga_tuner()` builds a `PymooGATuner`/`DEAPGATuner`
+ `OuterLoopEvaluator` (etc.) + `UAVModelAdapter` → each generation, the GA
backend calls `evaluator.evaluate_population(candidates)` →
`UAVModelAdapter.run_simulation()` calls `simulateMission()` in-process, which
builds the real controller object via `instantiateControllerWithGA()` (which
reads the GA's vector out of `wrapper_params.external_controller_params`) →
metrics are computed from the resulting log and scored, with
`survival_penalty` short-circuiting crashed runs.

## 3. New files (SIL integration)

All under `integration/sil/`, plus one new top-level entry point and one new
`docker-compose.yml` service:

- **`integration/sil/paths.py`** - `find_repo_root(start)`: locates the repo
  root (the directory containing `docker-compose.yml`), preferring the
  `SIL_REPO_ROOT` env var (set by the `ga_tuner` compose service, see below)
  since walking up from `start` doesn't work when this code runs inside a
  container whose mount subtree differs from where `docker-compose.yml`
  lives on the host.
- **`integration/sil/gain_serializer.py`** - `write_pid_gains_json(gain_dict,
  json_path, tuned_parameter_names=None)`: translates the GA's Python-side
  gain keys (`KP_tran`, `KI_tran`, ... - fully-resolved 3x3 `np.diag`
  matrices) into the C++ flightstack's `gains_pid.json` schema
  (`KP_translational`, ... wrapped as `{"scaling_coef": 1.0, "matrix":
  [[...]]}`). If `tuned_parameter_names` is given, only the matrix groups
  that actually contain a tuned parameter are overwritten - untouched groups
  (e.g. all of rotational, when only translational is being tuned) are left
  exactly as they are on disk instead of being clobbered by the GA's
  Python-side default template.
- **`integration/sil/docker_lifecycle.py`** -
  `restart_flightstack_with_gains(gain_dict, repo_root, controller_type,
  tuned_parameter_names=None, timeout_s=180.0)`: the core restart-per-candidate
  routine.
  1. Writes gains via `gain_serializer.write_pid_gains_json`.
  2. `docker compose up -d flightstack` (idempotent - starts it if not
     running yet).
  3. Captures the container's `State.StartedAt` timestamp, runs `docker
     compose restart flightstack`, and **verifies** `StartedAt` actually
     changed - raising `FlightstackRestartError` if not. A genuine process
     restart (not just a gains re-read) is required because
     `is_offset_odometry_msg_` (`vehicle_state.cpp`) and `timestamp_initial_`
     (`multi_threaded_node.cpp`) are one-shot C++ node state that only resets
     on a fresh process start.
  4. Polls `docker inspect --format {{.State.Health.Status}}` every 2s until
     `"healthy"` or `timeout_s` elapses.
  5. Asserts the running container's `CONTROLLER_TYPE` env var matches the
     expected controller (fails loudly if `docker-compose.yml`'s
     `x-controller-type` anchor is set wrong).
  - Currently only `controller_type == "PID"` is implemented; anything else
    raises `NotImplementedError`.
- **`integration/sil/sil_adapter.py`** - `SILUAVModelAdapter(UAVModelAdapter)`:
  overrides `run_simulation()` to call `restart_flightstack_with_gains(...)`
  with the candidate's gains *before* delegating to the base class, and sets
  `UAV_SIL_MODE=true` in its constructor so the rest of the simulation stack
  (`simulate_mission.py`) takes the `FlightStackServiceBridge` branch.
- **`integration/sil/__init__.py`** - one-line docstring:
  `"""GA-tuner-drives-real-SIL integration (PID, outer-loop, serial,
  restart-per-candidate)."""`
- **`run_tuner_sil.py`** (`ga_tuner/` root) - SIL entry point. Same
  `TUNING_CONFIG` and `create_uav_ga_tuner()`/`summarize_tuner_result()`/
  `save_pareto_front_logs()` calls as `run_tuner.py`, but passes
  `sil_mode=True` and additionally saves a `pareto_front.png` plot. Prints a
  reminder on startup that `docker-compose.yml`'s `x-controller-type` anchor
  must already be `"PID"`.
- **`docker-compose.yml`: `ga_tuner` service** - new service, `profiles:
  ["tools"]` (so it's excluded from a bare `docker compose up`). Builds from
  `docker/simulator.Dockerfile` (needs `rclpy` + PyChrono), mounts
  `/var/run/docker.sock` and `${PWD}:${PWD}` (docker-outside-of-docker, so
  `docker_lifecycle.py`'s `docker compose ...` calls issued *inside* this
  container control the host's `flightstack` container), and sets
  `SIL_REPO_ROOT=${PWD}` so `paths.find_repo_root()` resolves correctly.
  Runs `python .../ga_tuner/run_tuner_sil.py`. Invoke explicitly:
  `docker compose run --rm ga_tuner`.

## 4. Modified files (to support SIL mode)

- **`integration/uav_integration.py`** - `create_uav_ga_tuner(...,
  sil_mode=False)` gained the `sil_mode` parameter:
  - When `True`, validates `controller_type == "PID"`,
    `evaluator_type == "outer"`, and `parallel_config["enabled"] is False`,
    raising otherwise.
  - Instantiates `sil.sil_adapter.SILUAVModelAdapter()` instead of
    `UAVModelAdapter()`.
  - Calls `applyFlightstackTrajectoryOverride()` (pre-existing, from
    [`SIMULATOR_CHANGES.md`](../../SIMULATOR_CHANGES.md)) so the GA's mission
    config mirrors whatever trajectory `config.json` currently selects,
    matching what the flightstack container is actually targeting.
  - `plot_pareto_front(...)` (used only by `run_tuner_sil.py`) was added for
    saving a Pareto-front scatter plot to disk in a headless container.
- **`ga_config/config.py`, `ga_config/evaluator_config.py`,
  `ga_config/ga_config.py`** - default population size / generation count /
  evaluator settings tuned down for the much higher per-candidate cost of a
  real container restart (a plain simulation-only candidate is subsecond to
  a few seconds; a SIL candidate additionally pays a container restart +
  healthcheck round-trip, typically several seconds to tens of seconds).
- **`tuners/PID/pid_tuning.py`** - translational gain bounds narrowed from
  `KP/KI/KD ∈ [0, 150]` to `KP ∈ [3.0, 12.5]`, `KI ∈ [0.0, 4.0]`,
  `KD ∈ [2.0, 12.5]`. This was a direct fix for GA candidates saturating all
  8 motor thrusts and crashing the simulation almost immediately - see the
  survival-penalty discussion earlier in this session. Rotational bounds
  (`KP ∈ [0, 200]`, `KI/KD ∈ [0, 150]`) are unchanged since only
  translational gains are in `TUNING_SELECTIONS` today.

## 5. How to use SIL-mode GA tuning

1. In `docker-compose.yml`, set `x-controller-type: &controller_type "PID"`
   (SIL-GA is PID-only for now).
2. In `ga_config/ga_config.py`, confirm `controller_type = "PID"`.
3. In `ga_config/evaluator_config.py`, confirm `EVALUATOR_TYPE = "outer"` and
   `PARALLEL_CONFIG["enabled"] = False`.
4. From the repo root (not inside a container):
   ```bash
   docker compose run --rm ga_tuner
   ```
   Do **not** also run a manual `docker compose up` at the same time - both
   would contend for the one `flightstack` container, which this run
   restarts itself, once per candidate.
5. Per candidate, the run does: write `gains_pid.json` → `docker compose
   restart flightstack` → wait for healthy → run one PyChrono simulation
   against it via the existing `/sil/compute_control` bridge → score it
   (with survival-penalty short-circuiting a crash). Expect each candidate to
   take noticeably longer than the pure-Python path - budget generations/
   population size accordingly (see `ga_config/ga_config.py`).
6. Results land under `evaluator_config.LOG_DIRECTORY`, same as the
   pure-Python path, plus a `pareto_front.png` in that directory.
7. As with the pure-Python path, this only searches gain space - it does not
   itself update `gains_pid.json` to a final "winning" value after the run
   ends (the file on disk after the run reflects whichever candidate was
   evaluated last). Copy the best solution's values (printed by
   `summarize_tuner_result`) into `gains_pid.json` yourself, or use
   `save_pareto_front_logs()`'s saved per-solution logs to confirm which one
   you want before adopting it.

## 6. Known limitations / gaps

- **PID + outer-loop + serial only.** `create_uav_ga_tuner` raises if
  `sil_mode=True` is combined with any other controller type, evaluator
  type, or `parallel_config["enabled"]=True`. Extending to MRAC-family
  controllers would need `docker_lifecycle.restart_flightstack_with_gains`
  (currently hardcoded to `gains_pid.json`) and `gain_serializer` generalized
  per controller.
- **One shared container = no parallelism.** Real parallelism would need
  multiple independent `flightstack`/`simulator` container pairs, which
  `docker-compose.yml` doesn't support today (both services hardcode
  `ROS_DOMAIN_ID: "42"` with `network_mode: host`, so N simultaneous stacks
  would collide as DDS peers on the same domain).
- **Two tuner classes can't actually be instantiated via the standard
  registry.** `HybridMRACTuning.__init__(self)` and
  `NonAdaptiveEBCITuning.__init__(self)` (unlike every other tuner class)
  don't accept a `tuning_config` argument, but
  `integration/ga_utils/registry.py:get_tuning_instance()` and
  `__init__.py:instantiateTuner()` both always pass one - either call raises
  `TypeError` for these two controllers today. Not SIL-specific (affects the
  pure-Python path too), but worth fixing before either controller is used
  via the GA.
- **`ControllerParameterConverter(baseline_gains=...)` is the wrong kwarg
  name.** `TwoLayerMRACTuning` and `HybridTwoLayerMRACTuning` both construct
  their converter with `baseline_gains=self._baseline_gains`, but
  `ControllerParameterConverter.__init__` only accepts `reference_gains=`.
  Both would raise `TypeError` on first use. Also pre-existing, not
  SIL-specific.
- **`FunnelMRACTuning` extracts `Q_M_funnel_tran`/`Q_M_funnel_rot` but never
  exposes them as tunable bounds** (`BaseMRACTuning._build_parameter_bounds`
  only recognizes `Gamma_*`/`Q_tran`/`Q_rot` keys), so the funnel-specific
  matrices are effectively fixed at their default values regardless of GA
  configuration.
- **No hot-reload without a restart.** Every candidate pays a full container
  restart + healthcheck round-trip; there's no in-process "just change the
  gains" path into a running C++ node. This is a deliberate simplicity
  trade-off (see `docker_lifecycle.py`'s module docstring for why a genuine
  restart is actually required, not just a nice-to-have), but it's the
  dominant per-candidate cost.

## 7. Related docs

- [`README.md`](README.md) - general (non-SIL) GA tuner usage.
- [`../../../../SIL_USAGE_GUIDE.md`](../../../../SIL_USAGE_GUIDE.md) - Section 4
  now points here instead of describing the old limitation.
- [`../../../../SIL.md`](../../../../SIL.md) - documents the `ga_tuner`
  `docker-compose.yml` service itself (infra glue, outside this package).
- [`../../SIMULATOR_CHANGES.md`](../../SIMULATOR_CHANGES.md) - lists the
  `integration/sil/*.py` files as new/modified files within the simulator
  package's upstream diff.