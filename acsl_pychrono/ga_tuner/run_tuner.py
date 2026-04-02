"""Entry point for running GA tuning."""

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
)


def run_from_config(config: TuningConfig) -> None:
    """Run GA tuning using the provided configuration."""
    tuner = create_uav_ga_tuner(
        ga_config=config.ga,
        evaluator_config=config.evaluator,
        tuned_parameters=list(config.tuned_parameters) if config.tuned_parameters else None,
        mission_overrides=dict(config.mission_overrides) if config.mission_overrides else None,
    )
    
    result = tuner.optimize(verbose=config.ga.verbose)
    summarize_tuner_result(tuner, result)
    save_pareto_front_logs(tuner, result, clean_eval_folders=config.evaluator.clean_eval_folders)


def main() -> None:
    config = TUNING_CONFIG
    run_from_config(config)


if __name__ == "__main__":
    main()
