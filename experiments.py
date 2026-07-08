from main import *
import time

# Experiment -- Test all UAVs =======================================
UAVS = [
    "SQ",
    "Q",
    "QUAD",
    "X8",
    # "X81",
    # "Q1",
    # "SQ1",
]

for uav in UAVS:
    run_experiment(
        uav=uav,
        controller="MRAC",
        visualize=True,
        add_payload="False",
        trajectory_type="piecewise_polynomial_trajectory",
        trajectory_file="bean_trajectory0p2.json",
        # trajectory_file="rollercoaster_trajectory1p2.json"
    )
    print("----------------------------------------")
# ===================================================================

# Experiment -- Test all controllers ================================
# CONTROLLERS = [
#     "PID",
#     "MRAC",
#     "HybridMRAC",
#     "NonAdaptiveEBCI",
#     "TwoLayerMRAC",
#     "FunnelMRAC",
#     "HybridTwoLayerMRAC",
#     "FunnelTwoLayerMRAC",
# ]

# for controller in CONTROLLERS:
#     run_experiment(
#         uav="QUAD",
#         controller=controller,
#         visualize=True,
#         # simulation_duration=3.5,
#         add_payload="False",
#         payload_type="many_steel_balls_in_random_position",
#         # payload_type="two_steel_balls", 
#         # payload_type="ten_steel_balls_in_two_lines",
#         # payload_type="sling_ball_payload",
#         trajectory_file="bean_trajectory0p2.json",
#         # trajectory_file="rollercoaster_trajectory1p2.json"
#     )
#     print("----------------------------------------")
# ===================================================================

# Experiment -- Test all trajectories ===============================
# TAJECTORIES = [
#     "bean_trajectory0p2.json", # Tuned
#     "rollercoaster_trajectory1p2", # Works - Not tuned
# ]

# for trajectory in TAJECTORIES:
#     run_experiment(
#         uav="QUAD",
#         controller="MRAC",
#         visualize=False,
#         add_payload="False",
#         trajectory_type="piecewise_polynomial_trajectory",
#         trajectory_file=trajectory,
#     )
#     print("----------------------------------------")
# ===================================================================
    
# Experiment -- Test textures =======================================
# TEXTURES = [
#     "grass0.jpg", #"1.jpg",
#     "grass1.jpg", #"2.jpg",
#     "grass2.jpg", #"6.jpg",
#     "grass3.jpg", #"8.jpg",
#     "grass4.jpg", #"9.jpg",
#     "grass5.jpg", #"10.jpg",
#     "grass6.jpg", #"11.jpg",
#     "grass7.jpg", #"11.jpg",
#     "grass8.jpg", #"11.jpg",
# ]

# for texture in TEXTURES:
#     run_experiment(
#         uav="QUAD",
#         controller="PID",
#         visualize=True,
#         add_payload="False",
#         floor_texture_path=texture,
#         trajectory_type="piecewise_polynomial_trajectory",
#         trajectory_file="rollercoaster_trajectory1p2.json",
#     )
#     print("----------------------------------------")
# ===================================================================

# Experiment -- Test all controllers with different texture =========
# CONTROLLERS = [
#     "PID", # Tuned
#     "MRAC", # Works - Not tuned
#     "HybridMRAC", # Works - Not tuned
#     "NonAdaptiveEBCI", #  Works - Not tuned
#     "TwoLayerMRAC", # Works on Rollercoaster only - Not tuned
#     "FunnelMRAC", # Works on Rollercoaster only - Not tuned
#     "HybridTwoLayerMRAC", # Works on bean_trajectory only - Not tuned
#     "FunnelTwoLayerMRAC" # Doesn't work - Not tuned
# ]

# TEXTURES = [
#     "6.jpg",
#     "9.jpg",
#     "10.jpg",
#     "11.jpg",
#     "grass8.jpg",
#     "grass8.jpg",
#     "grass8.jpg",
#     "grass8.jpg",
# ]

# for texture, controller in zip(TEXTURES, CONTROLLERS):
#     run_experiment(
#         uav="QUAD",
#         controller=controller,
#         visualize=True,
#         # simulation_duration=3.5,
#         add_payload="False",
#         payload_type="many_steel_balls_in_random_position",
#         floor_texture_path=texture,
#         # payload_type="two_steel_balls", 
#         # payload_type="ten_steel_balls_in_two_lines",
#         # payload_type="sling_ball_payload",
#         trajectory_type="piecewise_polynomial_trajectory",
#         # trajectory_file="bean_trajectory0p2.json",
#         trajectory_file="rollercoaster_trajectory1p2.json"
#     )
#     print("----------------------------------------")
# ===================================================================


# Experiment -- Test miscellaneous ==================================
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
#     print("----------------------------------------")
# ===================================================================