"""Controller factory with GA tuner integration."""

from acsl_pychrono.control import get_controller_classes, available_controllers

from .ga_utils import (
    get_param_key,
    get_tuning_instance,
    build_parameter_vector,
    apply_gain_dict,
    post_process_gains,
    log_applied_gains,
)


def instantiateControllerWithGA(
    controller_type: str,
    ode_input,
    flight_params,
    timestep,
    wrapper_params=None
):
    """
    Factory function to instantiate controllers with optional GA tuner integration.

    Parameter naming convention: uses "{controller_type.lower()}_params" as the key in
    external_controller_params (e.g., PID -> pid_params).
    """
    if controller_type not in available_controllers():
        raise ValueError(f"Unknown controller type: {controller_type}")

    GainsClass, ControllerClass, LoggerClass = get_controller_classes(controller_type)

    gains = GainsClass(flight_params)

    external_params = getattr(wrapper_params, "external_controller_params", None) if wrapper_params else None
    if external_params:
        try:
            _apply_ga_parameters(controller_type, gains, external_params)
        except Exception as exc:
            print(f"Warning: GA tuner parameter mapping failed for {controller_type}: {exc}")

    controller = ControllerClass(gains, ode_input, flight_params, timestep)
    logger = LoggerClass(gains)

    return gains, controller, logger


def _apply_ga_parameters(controller_type: str, gains, external_params):
    """Apply GA parameters for a given controller type."""
    # Ensure tuning support exists
    tuning = get_tuning_instance(controller_type)

    param_key = get_param_key(controller_type)
    params = external_params.get(param_key)
    if params is None:
        print(f"Warning: GA tuner enabled but missing '{param_key}' in external_controller_params")
        return

    vector, error = build_parameter_vector(tuning, params)
    if error:
        print(f"Warning: {controller_type} parameter mapping failed: {error}")
        return

    gain_dict = tuning.vector_to_gains(vector)
    apply_gain_dict(gains, gain_dict)
    post_process_gains(controller_type, gains, gain_dict)
    
    # Get partial tuning info if available
    num_tuned = external_params.get('_num_tuned')
    log_applied_gains(controller_type, gain_dict, num_total_params=len(params), num_tuned_params=num_tuned)

