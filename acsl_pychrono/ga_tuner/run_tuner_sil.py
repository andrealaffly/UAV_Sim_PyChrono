"""Entry point for running GA tuning against the real SIL flightstack (PID, outer-loop,
serial, restart-per-candidate). See ga_tuner/integration/sil/ and
ACSL_SIL_Flightstack_Guide.md / SIL_USAGE_GUIDE.md for background.

Run via the `ga_tuner` compose service (needs rclpy + PyChrono, which only exist
inside that image, plus docker-outside-of-docker access to restart `flightstack`):

    docker compose run --rm ga_tuner

...from the repo root. Do not also run a manual `docker compose up` at the same
time - both would contend for the one shared flightstack container, which this
script starts and restarts itself, once per candidate, with freshly written gains.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
os.chdir(PROJECT_ROOT)

from acsl_pychrono.ga_tuner.ga_config.config import (
    TUNING_CONFIG,
    TuningConfig,
)
from acsl_pychrono.ga_tuner.integration.uav_integration import (
    create_uav_ga_tuner,
    summarize_tuner_result,
    save_pareto_front_logs,
    plot_pareto_front,
)


def run_from_config(config: TuningConfig) -> None:
    """Run GA tuning against the real SIL flightstack using the provided configuration."""
    tuner = create_uav_ga_tuner(
        ga_config=config.ga,
        evaluator_config=config.evaluator,
        tuned_parameters=list(config.tuned_parameters) if config.tuned_parameters else None,
        mission_overrides=dict(config.mission_overrides) if config.mission_overrides else None,
        sil_mode=True,
    )

    result = tuner.optimize(verbose=config.ga.verbose)
    summarize_tuner_result(tuner, result)
    save_pareto_front_logs(tuner, result, clean_eval_folders=config.evaluator.clean_eval_folders)

    pareto_plot_path = os.path.join(config.evaluator.log_directory, "pareto_front.png")
    plot_pareto_front(tuner, result, save_path=pareto_plot_path, show=False)


def main() -> None:
    print(
        "[SIL-GA] Reminder: docker-compose.yml's x-controller-type anchor must be set to "
        "\"PID\" before running this - the preflight check below will fail loudly if the "
        "running flightstack container disagrees, but set it correctly up front to avoid "
        "an unnecessary container rebuild.\n"
    )
    config = TUNING_CONFIG
    run_from_config(config)


if __name__ == "__main__":
    main()
