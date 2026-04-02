from main import *
import time

UAVS = ["QUAD1", "PURPLE_QUAD", "X8", "X8_DEFAULT", "X8"]
CONTROLLERS = ["PID", "PID", "TwoLayerMRAC", "HybridTwoLayerMRAC", "NonAdaptiveEBCI"]
ADD_PAYLOAD = [True, False, True, True, True]
PAYLOAD_TYPE = ["sling_ball_payload", "two_steel_balls", "sling_ball_payload", "ten_steel_balls_in_two_lines", "many_steel_balls_in_random_position"]
SEQUENTIAL_DROP = [False, False, False, True, True]
SEQUENTIAL_DROP_START = [0, 0, 0, 0.5, 1]
INCLUDE_ENVIRONMENT = [False, True, False, False, False]
APPLY_MOTOR_FAILURE = [False, False, True, False, False]
MOTOR_FAILURE_TIME = [1, 0, 1, 0, 0]
TRAJECTORY_TYPE = ["piecewise_polynomial_trajectory", "piecewise_polynomial_trajectory", "piecewise_polynomial_trajectory", "piecewise_polynomial_trajectory", "circular_trajectory"]
TRAJECTORY_FILE = ["bean_trajectory0p2.json", "bean_trajectory0p2.json", "rollercoaster_trajectory1p2.json", "rollercoaster_trajectory1p2.json", "rollercoaster_trajectory1p2.json"]


for uav, controller, add_payload, payload_type, sequential_drop, sequential_drop_start, include_environment, apply_motor_failure, motor_failure_time, trajectory_type, trajectory_file in zip(UAVS, CONTROLLERS, ADD_PAYLOAD, PAYLOAD_TYPE, SEQUENTIAL_DROP, SEQUENTIAL_DROP_START, INCLUDE_ENVIRONMENT, APPLY_MOTOR_FAILURE, MOTOR_FAILURE_TIME, TRAJECTORY_TYPE, TRAJECTORY_FILE):
    run_experiment(
        uav=uav,
        controller=controller,
        visualize="Yes",
        simulation_duration=3.5,
        add_payload=add_payload,
        payload_type=payload_type,
        sequential_drop=sequential_drop,
        sequential_drop_start=sequential_drop_start,
        include_environment=include_environment,
        apply_motor_failure=apply_motor_failure,
        motor_failure_time=motor_failure_time,
        trajectory_type=trajectory_type,
        trajectory_file=trajectory_file
    )
    time.sleep(.1)
    print("----------------------------------------")
