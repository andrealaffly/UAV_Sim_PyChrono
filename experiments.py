from main import *
import time

# UAVS = ["Q", "SQ", "X8", "X8", "X8"]
# CONTROLLERS = ["PID", "PID", "TwoLayerMRAC", "HybridTwoLayerMRAC", "NonAdaptiveEBCI"]
# ADD_PAYLOAD = [True, False, True, True, True]
# PAYLOAD_TYPE = ["sling_ball_payload", "two_steel_balls", "sling_ball_payload", "ten_steel_balls_in_two_lines", "many_steel_balls_in_random_position"]
# SEQUENTIAL_DROP = [False, False, False, True, True]
# SEQUENTIAL_DROP_START = [0, 0, 0, 0.5, 1]
# INCLUDE_ENVIRONMENT = [False, True, False, False, False]
# APPLY_MOTOR_FAILURE = [False, False, True, False, False]
# MOTOR_FAILURE_TIME = [1, 0, 1, 0, 0]
# TRAJECTORY_TYPE = ["piecewise_polynomial_trajectory", "piecewise_polynomial_trajectory", "piecewise_polynomial_trajectory", "piecewise_polynomial_trajectory", "circular_trajectory"]
# TRAJECTORY_FILE = ["bean_trajectory0p2.json", "bean_trajectory0p2.json", "rollercoaster_trajectory1p2.json", "rollercoaster_trajectory1p2.json", "rollercoaster_trajectory1p2.json"]


# for uav, controller, add_payload, payload_type, sequential_drop, sequential_drop_start, include_environment, apply_motor_failure, motor_failure_time, trajectory_type, trajectory_file in zip(UAVS, CONTROLLERS, ADD_PAYLOAD, PAYLOAD_TYPE, SEQUENTIAL_DROP, SEQUENTIAL_DROP_START, INCLUDE_ENVIRONMENT, APPLY_MOTOR_FAILURE, MOTOR_FAILURE_TIME, TRAJECTORY_TYPE, TRAJECTORY_FILE):
#     run_experiment(
#         uav=uav,
#         controller=controller,
#         visualize="Yes",
#         simulation_duration=3.5,
#         add_payload=add_payload,
#         payload_type=payload_type,
#         sequential_drop=sequential_drop,
#         sequential_drop_start=sequential_drop_start,
#         include_environment=include_environment,
#         apply_motor_failure=apply_motor_failure,
#         motor_failure_time=motor_failure_time,
#         trajectory_type=trajectory_type,
#         trajectory_file=trajectory_file
#     )
#     time.sleep(.1)
#     print("----------------------------------------")

# CONTROLLERS = [
#     "TwoLayerMRAC",
#     "HybridTwoLayerMRAC",
#     "FunnelTwoLayerMRAC"
# ]

CONTROLLERS = [
    # "PID", # Tuned
    # "MRAC", # Tuned
    # "HybridMRAC", # Tuned
    # "NonAdaptiveEBCI", # Tuned
    "TwoLayerMRAC", # Tuned
    "FunnelMRAC", # Works on Rollercoaster only - Not tuned
    "HybridTwoLayerMRAC", # Tuned
    "FunnelTwoLayerMRAC" # Doesn't work - Not tuned
]

for controller in CONTROLLERS:
    run_experiment(
        uav="X8",
        controller=controller,
        visualize=True,
        # simulation_duration=3.5,
        add_payload="True",
        payload_type="many_steel_balls_in_random_position",
        # payload_type="two_steel_balls", 
        # payload_type="ten_steel_balls_in_two_lines",
        # payload_type="sling_ball_payload",
        trajectory_type="piecewise_polynomial_trajectory",
        # trajectory_file="bean_trajectory0p2.json",
        trajectory_file="rollercoaster_trajectory1p2.json"
    )
    print("----------------------------------------")
