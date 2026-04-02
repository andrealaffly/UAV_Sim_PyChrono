import numpy as np
import scipy.io as sio
from pathlib import Path
from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedSeq
from main import *

def normalize_floats(obj):
    if isinstance(obj, float):
        return round(obj)
    elif isinstance(obj, list):
        return [normalize_floats(x) for x in obj]
    elif isinstance(obj, dict):
        return {k: normalize_floats(v) for k, v in obj.items()}
    return obj

def flow_matrix(mat):
    rows = CommentedSeq()
    rows.fa.set_flow_style()      # [[...], [...]]
    for row in mat:
        r = CommentedSeq(row)
        r.fa.set_flow_style()     # [a, b, c]
        rows.append(r)
    return rows

base_path = Path('./simulation_logs/new/pareto_solutions')
FILE_PATHS = []
for file_path in base_path.glob("*.mat"):
    FILE_PATHS.append(file_path)

#26
for idx in range(20, 49 + 1):
    print("===============================================")
    print(f"Loading PID gains from file index: {idx}")
    print("===============================================")
    file_path = FILE_PATHS[idx]
    # Load the .mat file
    mat_contents = sio.loadmat(file_path, squeeze_me=True, struct_as_record=False)

    gains_struct = mat_contents["gains"]

    TUNED_PID_GAINS = ["KP_tran", "KD_tran", "KI_tran", "KP_rot", "KD_rot", "KI_rot"]

    ga_gains = {}

    for name in TUNED_PID_GAINS:
        value = getattr(gains_struct, name)

        # Convert MATLAB matrix to Python list
        if isinstance(value, np.ndarray):
            ga_gains[name] = value.tolist()
        else:
            ga_gains[name] = float(value)

        print(f"{name}: {ga_gains[name]}")
        
        

    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.width = 40#28
    yaml.indent(mapping=2, sequence=10, offset=2)

    yaml_file = "./acsl_pychrono/uav/QUAD/Controller_Gains/PID_Tunned.yaml"

    # Load existing YAML
    with open(yaml_file, "r") as f:
        config = yaml.load(f)

    # Update only the tuned gains
    for gain_name, gain_value in ga_gains.items():
        if gain_name not in config:
            print(f"[WARN] {gain_name} not found in YAML, skipping")
            continue

        # config[gain_name]["matrix"] = gain_value
        config[gain_name]["matrix"] = flow_matrix(
            normalize_floats(gain_value)
            # gain_value
        )


    # Write back (comments preserved)
    with open(yaml_file, "w") as f:
        yaml.dump(config, f)
        
    run_experiment(
        uav="QUAD",
        controller="PID",
        visualize="Yes",
        # simulation_duration=3.5,
        add_payload=False,
        # payload_type=payload_type,
        # sequential_drop=sequential_drop,
        # sequential_drop_start=sequential_drop_start,
        include_environment=False,
        apply_motor_failure=False,
        # motor_failure_time=motor_failure_time,
        trajectory_type="piecewise_polynomial_trajectory",
        # trajectory_file="rollercoaster_trajectory1p2.json"
        trajectory_file="bean_trajectory0p2.json"
    )

