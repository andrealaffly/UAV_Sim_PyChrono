"""Helpers to build full parameter vectors from external GA params."""

from typing import Mapping, Sequence, Tuple, Union, List
import numpy as np

from ...core.controller_tuning import ControllerTuningInterface


def _extract_named_params_from_mapping(data: Mapping) -> Tuple[List[str], List[float]]:
    """Parse named parameters from mapping; supports names/values or direct mapping."""
    if "names" in data and "values" in data:
        names = list(data["names"])
        values = list(data["values"])
        if len(names) != len(values):
            raise ValueError("parameter names/values length mismatch")
        return names, values

    names = list(data.keys())
    values = [data[name] for name in names]
    return names, values


def build_parameter_vector(
    tuning: ControllerTuningInterface,
    params: Union[Sequence, Mapping, np.ndarray],
) -> Tuple[List[float], str]:
    """
    Convert external params (mapping or full vector) into a full parameter vector.

    Returns:
        (vector, error_message). error_message is "" when successful.
    """
    parameter_names = tuning.get_parameter_names()
    expected_length = len(parameter_names)
    vector = tuning.gains_to_vector(tuning.get_default_gains())

    if isinstance(params, Mapping):
        try:
            names, values = _extract_named_params_from_mapping(params)
        except ValueError as exc:
            return [], f"{exc}"
        for name, value in zip(names, values):
            try:
                idx = parameter_names.index(name)
            except ValueError:
                return [], f"Unknown parameter '{name}'"
            vector[idx] = float(value)
        return vector, ""

    if not isinstance(params, (Sequence, np.ndarray)) or isinstance(params, (str, bytes)):
        return [], "Parameters must be a full vector or a mapping of parameter names"

    if len(params) != expected_length:
        return [], f"Expected {expected_length} parameters, received {len(params)}"

    return list(params), ""
