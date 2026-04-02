import warnings
from dataclasses import dataclass, field
from typing import Any


@dataclass
class MissionConfig:
  # Total simulation duration in seconds
  simulation_duration_seconds: float = 23.0
  # Run the simulator in Wrapper mode (more simulations automatically run sequentially)
  wrapper_flag: bool = False
  # If True, perform real-time rendering of the simulation with Irrlicht
  visualization_flag: bool = True
  # Dynamic camera options:
  # "fixed"
  # "default",
  # "side",
  # "front",
  # "follow",
  # "fpv"
  # "orbit"
  # "follow_smooth"
  # "topdown"
  camera_mode: str = "fixed"
  # Simulation timestep used by Chrono
  timestep: float = 0.005 #0.005

  # Controller types:
  # "PID",
  # "MRAC",
  # "TwoLayerMRAC",
  # "FunnelMRAC",
  # "HybridMRAC",
  # "HybridTwoLayerMRAC",
  # "NonAdaptiveEBCI",
  # "FunnelTwoLayerMRAC",
  controller_type: str = "FunnelTwoLayerMRAC"

  # User-defined trajectory types:
  # "circular_trajectory",
  # "hover_trajectory",
  # "square_trajectory",
  # "rounded_rectangle_trajectory",
  # "piecewise_polynomial_trajectory"
  trajectory_type: str = "piecewise_polynomial_trajectory"

  # If the trajectory_type is "piecewise_polynomial_trajectory", then choose the trajectory file to run
  # Path relative to 'current_working_directory/params/user_defined_trajectory'
  # "bean_trajectory0p2.json"
  # "rollercoaster_trajectory1p2.json"
  # "stadium.json"
  trajectory_data_path: str = "rollercoaster_trajectory1p2.json"

  # Time for which, after executing the "trajectory_data_path",
  # the vehicle is hovering before starting the landing phase
  hover_after_trajectory_time_seconds: float = 5.0

  # Flag to add or remove the payload from the simulation
  add_payload_flag: bool = True
  # Payload types: 
  # "two_steel_balls"
  # "ten_steel_balls_in_two_lines"
  # "many_steel_balls_in_random_position"
  # "sling_ball_payload"
  payload_type: str = "two_steel_balls"

  # Payload Dropping (only for "two_steel_balls" payload type)
  drop_two_steel_balls: bool = False
  two_steel_balls_drop_time: float = 1.0 # 3.7
  
  # Dropping multiple balls one after the other (only for payload types other than "two_steel_balls")
  sequentially_drop_multiple_balls: bool = False
  sequentially_drop_start_time: float = 1.0 # 3.0
  sequentially_drop_interval: float = 0.10
  
  # Motor Failure parameters
  apply_motor_failure: bool = False
  motor_failure_time: float = 1.0 # 4.5
  
  # Apply External Forces
  # Applying Aero-Dynamic force to the drone
  apply_wind_force: bool = False
  wind_force_vector: tuple[float, float, float] = (0.5, 0.0, 0.0) # [N] in global pychrono coordinate system

  # Unique wrapper batch folder passed to the function used for running many parallel wrapper simulations 
  wrapper_batch_dir: str = "" # LEAVE BLANK!!!

  # Number of parallel simulations (one per CPU) to be run in "wrapper" mode
  wrapper_max_parallel: int = 18

  def __post_init__(self):
    if self.wrapper_flag and self.visualization_flag:
      warnings.warn("Visualization is disabled because wrapper mode is enabled.")
      self.visualization_flag = False

@dataclass
class VehicleConfig:
  # UAV types:
  # "Q"
  # "SQ"
  # "X8"
  uav_name: str = "X8"

@dataclass
class EnvironmentConfig:
  # Include external environment in the simulation
  include: bool = False
  # Path relative to 'current_working_directory/assets/environments'
  # "environment3/environment3.py" 
  # "environmentA/environmentA.py" 
  model_relative_path: str = "environmentA/environmentA.py"

@dataclass
class WrapperParams: # Add here the params to be sweeped by the wrapper with their default values
  my_ball_density: float = 7850
  # Optional fields used by GA tuner integrations
  pid_params: list[float] | None = None
  mrac_params: list[float] | None = None
  external_controller_params: dict[str, Any] | None = None  # GA tuner parameters

@dataclass
class SimulationConfig:
  mission_config: MissionConfig = field(default_factory=MissionConfig)
  vehicle_config: VehicleConfig = field(default_factory=VehicleConfig)
  environment_config: EnvironmentConfig = field(default_factory=EnvironmentConfig)
  wrapper_params: WrapperParams = field(default_factory=WrapperParams)
  
