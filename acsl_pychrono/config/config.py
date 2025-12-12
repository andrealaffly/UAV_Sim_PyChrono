import warnings
from dataclasses import dataclass, field

@dataclass
class MissionConfig:
  # Total simulation duration in seconds
  simulation_duration_seconds: float = 29.5 # 21.5
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
  camera_mode: str = "fpv"
  # Simulation timestep used by Chrono
  timestep: float = 0.005 #0.005

  # Controller types:
  # "PID",
  # "MRAC",
  # "TwoLayerMRAC",
  # "FunnelMRAC",
  # "HybridMRAC",
  # "HybridTwoLayerMRAC",
  # "NonAdaptiveEBCI"
  controller_type: str = "MRAC"

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
  trajectory_data_path: str = "rollercoaster_trajectory1p2.json"

  # Time for which, after executing the "trajectory_data_path",
  # the vehicle is hovering before starting the landing phase
  hover_after_trajectory_time_seconds: float = 5.0

  # Flag to add or remove the payload from the simulation
  add_payload_flag: bool = False
  # Payload types: 
  # "two_steel_balls"
  # "ten_steel_balls_in_two_lines"
  # "many_steel_balls_in_random_position"
  # "sling_ball_payload"
  payload_type: str = "two_steel_balls"

  # Unique wrapper batch folder passed to the function used for running many parallel wrapper simulations 
  wrapper_batch_dir: str = "" # LEAVE BLANK!!!

  # Number of parallel simulations (one per CPU) to be run in "wrapper" mode
  wrapper_max_parallel: int = 20

  def __post_init__(self):
    if self.wrapper_flag and self.visualization_flag:
      warnings.warn("Visualization is disabled because wrapper mode is enabled.")
      self.visualization_flag = False

@dataclass
class VehicleConfig:
  # Path relative to 'current_working_directory/assets/vehicles'
  model_relative_path: str = "x8copter/x8copter.py" 

@dataclass
class EnvironmentConfig:
  # Include external environment in the simulation
  include: bool = False
  # Path relative to 'current_working_directory/assets/environments'
  model_relative_path: str = "environmentA/environmentA.py" 

@dataclass
class WrapperParams: # Add here the params to be sweeped by the wrapper with their default values
  my_ball_density: float = 7850

@dataclass
class SimulationConfig:
  mission_config: MissionConfig = field(default_factory=MissionConfig)
  vehicle_config: VehicleConfig = field(default_factory=VehicleConfig)
  environment_config: EnvironmentConfig = field(default_factory=EnvironmentConfig)
  wrapper_params: WrapperParams = field(default_factory=WrapperParams)
  
