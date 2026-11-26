# Simulator Changes (vs. upstream `andrealaffly/UAV_Sim_PyChrono`)

_Author: Giorgio A. Orlando - 2026-07-24_

This document tracks every file added or modified in this checkout relative to the
upstream PyChrono simulator repository, to support running PyChrono as the physics
engine/state estimator in a Software-In-the-Loop (SIL) loop driven by the C++ flightstack.

## New Files

- **`acsl_pychrono/control/flighstack_client.py`**
  `FlightStackServiceBridge` - replaces the simulator's internal Python controller with a
  lock-step ROS2 service client. Each `run()` call packages the current vehicle state
  (position, velocity, quaternion, angular velocity, sim time) from `ode_input` into a
  `ComputeControl` request, blocks on `/sil/compute_control`, and stores the returned
  per-motor thrusts (Newtons) into `self.motor_thrusts` for the simulation loop to apply.
  Includes:
  - Sub-sampling logic (`_call_stride`): the C++ flightstack integrates with a fixed
    `time_step_rk4_ = 0.01 s`; if the sim runs at a finer timestep, the bridge only calls
    the C++ service every `round(0.01 / sim_dt)` physics steps and holds the last thrust
    values in between, so the RK4 integrator advances in sync with real elapsed time.
  - A block of zeroed dummy attributes (`roll_ref`, `mu_x`, `K_hat_x_tran`, etc.) required
    by the various controller `Logger.collectData()` implementations, since those
    internals are computed inside the C++ controller and never returned over the service.
  - A diagnostic CSV writer (`logs/sil_bridge_diagnostics.csv`) recording each call's raw
    `motor_commands` and timing, for comparing against the sim's own post-saturation
    "applied" thrust logs.

- **`logs/`** - output directory for the diagnostics CSV above (created by the bridge
  at construction time).

- **`params/user_defined_trajectory/landing_1meter.json`**,
  **`params/user_defined_trajectory/takeoff_1meter.json`** - new reference trajectories
  used for SIL takeoff/landing testing.

- A number of `*_copy.json` files under `params/user_defined_trajectory/` (e.g.
  `bean_trajectory0p15 copy.json`, `rollercoaster_trajectory1p1 copy.json`, `stadium
  copy.json`, etc.) and `bean_trajectory.json` - experimentation leftovers from trajectory
  tuning, not referenced by any config; candidates for cleanup.

- **`acsl_pychrono/ga_tuner/integration/sil/`** - restart-per-candidate bridge letting the
  GA tuner (`acsl_pychrono/ga_tuner/`, pre-existing) optimize gains against the *real* C++
  flightstack instead of only the Python controller reimplementation. Full architecture and
  usage in
  [`acsl_pychrono/ga_tuner/GA_TUNER_SIL.md`](acsl_pychrono/ga_tuner/GA_TUNER_SIL.md); summary:
  - `sil_adapter.py` - `SILUAVModelAdapter`, a `UAVModelAdapter` subclass that restarts
    `flightstack` with the candidate's gains before each simulation.
  - `docker_lifecycle.py` - `restart_flightstack_with_gains()`: writes gains, runs
    `docker compose restart flightstack`, verifies the container's `StartedAt` timestamp
    actually changed, waits for the healthcheck, and asserts `CONTROLLER_TYPE` matches.
  - `gain_serializer.py` - `write_pid_gains_json()`: translates the GA's gain-dict schema
    into the C++ side's `gains_pid.json` schema, preserving any untouched gain groups.
  - `paths.py` - `find_repo_root()`: locates `docker-compose.yml`, preferring the
    `SIL_REPO_ROOT` env var for docker-outside-of-docker path resolution.

- **`acsl_pychrono/ga_tuner/run_tuner_sil.py`** - new entry point (mirrors the pre-existing
  `run_tuner.py`) that calls `create_uav_ga_tuner(sil_mode=True)`. Run via the new `ga_tuner`
  docker-compose.yml service (`docker compose run --rm ga_tuner`), not directly.

## Modified Files

- **`acsl_pychrono/__init__.py`**
  `update_cfg_from_cli_args()`: if no `--controller` CLI flag is given, fall back to the
  `CONTROLLER_TYPE` environment variable when set, so the simulator's logged controller
  name matches whatever controller the flightstack container was built with (passed
  through via `docker-compose.yml`).

- **`acsl_pychrono/config/config.py`**
  Added `FLIGHTSTACK_CONFIG_PATH` (`/ros2_ws/src/flightstack/params/config/config.json`,
  bind-mounted read-only from the flightstack container) and
  `applyFlightstackTrajectoryOverride(mission_config)`, which reads the flightstack's
  selected `user_defined_trajectory_file` and mirrors it into the simulator's
  `MissionConfig` (`trajectory_type` / `trajectory_data_path`). This keeps both sides of
  the SIL loop flying the same reference trajectory without duplicating the selection in
  two places.

- **`acsl_pychrono/executor/run_single_simulation.py`**
  When `UAV_SIL_MODE=true`, calls `applyFlightstackTrajectoryOverride()` before building
  `SimulationConfig`, so the flightstack's trajectory choice takes priority over the
  simulator's own default before any CLI overrides are applied.

- **`acsl_pychrono/executor/simulate_mission.py`**
  When `UAV_SIL_MODE=true`, initializes `rclpy`, creates a `flightstack_bridge_node`, and
  substitutes the `controller` variable with a `FlightStackServiceBridge` instance instead
  of the simulator's normal PID/MRAC controller object — the rest of `simulateMission()`
  (state propagation, logging, force application) is unchanged and just calls
  `controller.run()` as before. Shuts `rclpy` down in the `finally` block once the mission
  ends. `is_sil_mode` is initialized to `False` up front so the `finally` cleanup is safe
  even if mission setup fails before the SIL branch runs.

- **`acsl_pychrono/simulation/simulation.py`**
  One unrelated tuning change: `addTwoSteelBallsPayload()` shrinks `ball_radius` from
  `0.0254` to `0.0254/1.15` - a payload-mass experiment, not part of the SIL integration.

- **`acsl_pychrono/simulation/visualization.py`**
  Whitespace-only changes (trailing spaces removed); no behavioral difference.

- **`install/install_pychrono_mac.sh`**
  No content change (diff stat shows 0 lines changed) - line-ending/mode-only diff.

- **`acsl_pychrono/ga_tuner/integration/uav_integration.py`**
  `create_uav_ga_tuner()` gained a `sil_mode: bool = False` parameter. When `True`: validates
  `controller_type == "PID"`, `evaluator_type == "outer"`, and
  `parallel_config["enabled"] is False` (raises otherwise, since SIL evaluation shares one
  Docker container and can't run in parallel); instantiates `SILUAVModelAdapter` instead of
  `UAVModelAdapter`; mirrors the flightstack's currently-selected trajectory into the GA's
  mission config via `applyFlightstackTrajectoryOverride()`. Also added `plot_pareto_front()`,
  used by the new `run_tuner_sil.py` to save a Pareto-front plot from a headless container.

- **`acsl_pychrono/ga_tuner/ga_config/config.py`, `ga_config/evaluator_config.py`,
  `ga_config/ga_config.py`**
  Default population size / generation count / evaluator settings adjusted for the much
  higher per-candidate cost of a SIL restart-per-candidate run (a container restart +
  healthcheck round-trip on top of the simulation itself, vs. subsecond for a pure-Python
  candidate).

- **`acsl_pychrono/ga_tuner/tuners/PID/pid_tuning.py`**
  Translational gain bounds narrowed from `KP/KI/KD ∈ [0, 150]` to `KP ∈ [3.0, 12.5]`,
  `KI ∈ [0.0, 4.0]`, `KD ∈ [2.0, 12.5]`. GA candidates sampled from the old, much wider
  range routinely saturated all 8 motor thrusts and crashed the simulation within the first
  second or two (visible as the `[SURVIVAL PENALTY]` log lines hitting close to the maximum
  2000 penalty run after run). Rotational bounds are unchanged.