# acsl_pychrono/__init__.py
__version__ = "1.0.0"
__author__ = "Mattia Gramuglia"
__email__ = "a.lafflitto@vt.edu"
__license__ = "BSD-3-Clause license"
__copyright__ = "Copyright (c) 2025 Mattia Gramuglia, Andrea L'Afflitto. All rights reserved."
__url__ = "https://github.com/andrealaffly/UAV_Sim_PyChrono"
__description__ = "This repository presents a high-fidelity simulation environment to test controllers for " \
                  "autonomous multi-rotor UAVs (an X8 UAV). Multiple linear and nonlinear control systems are" \
                  " provided. A wrapper allows performing a user-defined number of tests automatically."

import argparse
from .config import config as Cfg

def str2bool(v):
  if isinstance(v, bool):
    return v
  if v.lower() in ("yes", "true", "t", "1", "on"):
    return True
  if v.lower() in ("no", "false", "f", "0", "off"):
    return False
  raise argparse.ArgumentTypeError("Boolean value expected.")

def get_cli_args():
  """Parse CLI arguments and return them."""
  parser = argparse.ArgumentParser(
    description="Extra functionality to change config parameters from terminal.",
    formatter_class=argparse.RawTextHelpFormatter
  )

  # UAV and Controller options
  parser.add_argument("--uav", help="Instantiate UAV from name.")
  parser.add_argument(
    "--controller",
    choices=[
      "PID", 
      "MRAC",
      "TwoLayerMRAC",
      "FunnelMRAC",
      "HybridMRAC",
      "HybridTwoLayerMRAC",
      "NonAdaptiveEBCI",
      "FunnelTwoLayerMRAC",
      ],
    help="Instantiate controller from available type."
  )

  # Simulation options
  parser.add_argument("--simulation_duration", type=float, help="Total simulation duration in seconds.")
  parser.add_argument(
    "--visualize",
    type=str,
    help="Enable/disable real-time rendering (true/false) of the simulation with Irrlicht."
  )

  # Camera options
  parser.add_argument(
    "--camera_mode",
    choices=[
      "fixed",
      "default",
      "side",
      "front",
      "follow",
      "fpv",
      "orbit",
      "follow_smooth",
      "topdown",
    ],
    help="Select dynamic camera mode."
  )

  # Payload options
  parser.add_argument(
    "--add_payload",
    type=str,
    help='Add payload to the UAV.'
  )
  parser.add_argument(
    "--payload_type",
    choices=[
      "two_steel_balls",
      "ten_steel_balls_in_two_lines",
      "many_steel_balls_in_random_position",
      "sling_ball_payload"
    ],
    help='Specify payload type.'
  )
  parser.add_argument(
    "--drop_two_steel_balls",
    type=str,
    help="Enable dropping two steel balls payload."
  )
  parser.add_argument("--two_steel_balls_drop_time", type=float, help="Time (s) at which to drop the two steel balls.")
  parser.add_argument(
    "--sequential_drop",
    type=str,
    help="Enable sequentially dropping multiple payload balls."
  )
  parser.add_argument("--sequential_drop_start", type=float, help="Start time (s) for sequential ball drops.")
  parser.add_argument("--sequential_drop_interval", type=float, help="Interval (s) between each ball drop.")

  # Trajectory options
  parser.add_argument(
    "--trajectory_type",
    choices=[
      "circular_trajectory",
      "hover_trajectory",
      "square_trajectory",
      "rounded_rectangle_trajectory",
      "piecewise_polynomial_trajectory"
    ],
    help="Specify the user-defined trajectory type."
  )
  parser.add_argument(
    "--trajectory_file",
    help="Path (relative to 'params/user_defined_trajectory') of trajectory data file to execute."
  )
  parser.add_argument(
    "--hover_after_trajectory",
    type=float,
    help="Time in seconds to hover after executing the trajectory before landing."
  )

  # Motor failure
  parser.add_argument(
    "--apply_motor_failure",
    type=str,
    help="Trigger a motor failure event."
  )
  parser.add_argument("--motor_failure_time", type=float, help="Time (s) when motor failure occurs.")

  # External forces
  parser.add_argument(
    "--apply_wind_force",
    type=str,
    help="Apply aerodynamic wind force to the UAV."
  )
  parser.add_argument(
    "--wind_force_vector",
    type=float,
    nargs=3,
    metavar=("Fx", "Fy", "Fz"),
    help="Wind force vector components [N] in global coordinate system (e.g., --wind_force_vector 0.5 0.0 0.0)."
  )

  # Environment options
  parser.add_argument(
    "--include_environment",
    type=str,
    help="Include external environment in the simulation."
  )
  parser.add_argument(
    "--environment_path",
    choices=["environmentA/environmentA.py", "environment3/environment3.py"],
    help="Path to the environment script, relative to 'assets/environments'."
    )
  
  # # Wrapper Flag options 
  # parser.add_argument(
  #   "--wrapper_mode",
  #   type=str,
  #   help="Runs simulation in batches for multiple parallel simulation."
  # )

  return parser.parse_args()

def update_cfg_from_cli_args(sim_cfg: Cfg.SimulationConfig, cli_args):
  """
  Update the simulation configuration based on command-line arguments.
  Here you can add more parameters to be updated from CLI args.
  """
  
  # UAV and controller
  if cli_args.uav:
    sim_cfg.vehicle_config.uav_name = cli_args.uav
    
  if cli_args.controller:
    sim_cfg.mission_config.controller_type = cli_args.controller
    
  # Payload options
  if cli_args.add_payload:
    sim_cfg.mission_config.add_payload_flag = str2bool(cli_args.add_payload)
    
  if cli_args.payload_type:
    sim_cfg.mission_config.payload_type = cli_args.payload_type

  # Payload dropping
  if cli_args.drop_two_steel_balls:
    sim_cfg.mission_config.drop_two_steel_balls = str2bool(cli_args.drop_two_steel_balls)

  if cli_args.two_steel_balls_drop_time is not None:
    sim_cfg.mission_config.two_steel_balls_drop_time = cli_args.two_steel_balls_drop_time

  if cli_args.sequential_drop:
    sim_cfg.mission_config.sequentially_drop_multiple_balls = str2bool(cli_args.sequential_drop)

  if cli_args.sequential_drop_start is not None:
    sim_cfg.mission_config.sequentially_drop_start_time = cli_args.sequential_drop_start

  if cli_args.sequential_drop_interval is not None:
    sim_cfg.mission_config.sequentially_drop_interval = cli_args.sequential_drop_interval
    
  # Simulation settings
  if cli_args.simulation_duration is not None:
    sim_cfg.mission_config.simulation_duration_seconds = cli_args.simulation_duration

  if cli_args.visualize:
    sim_cfg.mission_config.visualization_flag = str2bool(cli_args.visualize)

  # Camera settings
  if cli_args.camera_mode:
    sim_cfg.mission_config.camera_mode = cli_args.camera_mode

  # Trajectory settings
  if cli_args.trajectory_type:
    sim_cfg.mission_config.trajectory_type = cli_args.trajectory_type

  if cli_args.trajectory_file:
    sim_cfg.mission_config.trajectory_data_path = cli_args.trajectory_file

  if cli_args.hover_after_trajectory is not None:
    sim_cfg.mission_config.hover_after_trajectory_time_seconds = cli_args.hover_after_trajectory

  # Motor failure
  if cli_args.apply_motor_failure:
    sim_cfg.mission_config.apply_motor_failure = str2bool(cli_args.apply_motor_failure)

  if cli_args.motor_failure_time is not None:
    sim_cfg.mission_config.motor_failure_time = cli_args.motor_failure_time

  # External forces
  if cli_args.apply_wind_force:
    sim_cfg.mission_config.apply_wind_force = str2bool(cli_args.apply_wind_force)

  if cli_args.wind_force_vector is not None:
    sim_cfg.mission_config.wind_force_vector = tuple(cli_args.wind_force_vector)
    
  # Environment inclusion
  if cli_args.include_environment:
    sim_cfg.environment_config.include = str2bool(cli_args.include_environment)

  if cli_args.environment_path:
    sim_cfg.environment_config.model_relative_path = cli_args.environment_path
    
  # # Wrapper mode
  # if cli_args.wrapper_mode:
  #   sim_cfg.mission_config.wrapper_flag = str2bool(cli_args.wrapper_mode)
  