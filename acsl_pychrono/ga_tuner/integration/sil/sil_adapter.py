"""UAVModelAdapter variant that drives the real flightstack via SIL restart-per-candidate."""

import os
from pathlib import Path
from typing import Any, Dict

from ..ga_utils import get_tuning_instance
from ..uav_integration import UAVModelAdapter
from ...ga_config.param_config import get_tuned_parameters
from . import docker_lifecycle
from .paths import find_repo_root


class SILUAVModelAdapter(UAVModelAdapter):
    """Before each candidate's simulation, writes its gains into gains_pid.json and
    restarts the flightstack container, then drives the run through the real C++
    controller via FlightStackServiceBridge (UAV_SIL_MODE=true) instead of the
    pure-Python controller reimplementation.
    """

    def __init__(self, *args, repo_root: Path = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.repo_root = repo_root or find_repo_root(Path(__file__))
        os.environ["UAV_SIL_MODE"] = "true"

    def run_simulation(self, config: Dict[str, Any]):
        controller_type = config["controller_type"]
        tuning = get_tuning_instance(controller_type)
        gain_dict = tuning.vector_to_gains(config["parameters"])
        tuned_parameter_names = get_tuned_parameters(controller_type)

        docker_lifecycle.restart_flightstack_with_gains(
            gain_dict, self.repo_root, controller_type, tuned_parameter_names
        )

        return super().run_simulation(config)
