"""Serialize the GA's PID gain dict into the C++ flightstack's gains_pid.json schema.

GA-side keys (PIDTuning.vector_to_gains, pid_tuning.py) use a "_tran"/"_rot"
suffix and hold the fully-resolved 3x3 gain matrices directly (np.diag(...)).
The C++ side (pid.cpp, extractMatrixFromJSON) reads "_translational"/
"_rotational" keys, each wrapped as {"scaling_coef": ..., "matrix": [[...]]}
with the two multiplied together. We write scaling_coef=1.0 and the fully
resolved matrix directly, since we control both sides of this write and don't
need to reverse-engineer any scaling split.
"""

import json
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import numpy as np

_SUFFIX_MAP = {
    "KP_tran": "KP_translational",
    "KI_tran": "KI_translational",
    "KD_tran": "KD_translational",
    "KP_rot": "KP_rotational",
    "KI_rot": "KI_rotational",
    "KD_rot": "KD_rotational",
}


def write_pid_gains_json(
    gain_dict: Dict[str, Any],
    json_path: Path,
    tuned_parameter_names: Optional[Sequence[str]] = None,
) -> None:
    """Rewrite the tuned PID gain keys in json_path, preserving any other content.

    gain_dict always contains all 6 matrices (tuning.vector_to_gains reconstructs a full
    parameter vector, filling untuned entries from PIDTuning.get_default_gains() — a
    Python-side hardcoded template that does NOT necessarily match this JSON file's
    current values). If tuned_parameter_names is given, only the matrix groups that
    actually contain at least one tuned parameter (e.g. "KP_tran_x") get overwritten;
    any group with zero tuned parameters (e.g. all of rotational, when only translational
    is being tuned) is left exactly as-is on disk, instead of being silently replaced by
    that unrelated template.
    """
    print(f"Writing tuned PID gains to {json_path} (tuned_parameter_names={tuned_parameter_names})")
    missing = set(_SUFFIX_MAP) - set(gain_dict)
    if missing:
        raise ValueError(f"gain_dict is missing expected PID keys: {sorted(missing)}")

    with open(json_path, "r") as f:
        data = json.load(f)

    for ga_key, json_key in _SUFFIX_MAP.items():
        if tuned_parameter_names is not None:
            group_is_tuned = any(name.startswith(ga_key + "_") for name in tuned_parameter_names)
            if not group_is_tuned:
                continue

        matrix = np.asarray(gain_dict[ga_key], dtype=float)
        if matrix.shape != (3, 3):
            raise ValueError(f"{ga_key}: expected a 3x3 matrix, got shape {matrix.shape}")
        data[json_key] = {
            "scaling_coef": 1.0,
            "matrix": matrix.tolist(),
        }

    with open(json_path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
