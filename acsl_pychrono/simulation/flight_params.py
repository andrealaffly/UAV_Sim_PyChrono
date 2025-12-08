import yaml
import numpy as np
from pathlib import Path
from acsl_pychrono.uav import UAV_BASE

# Flight parameters class to hold UAV and misc methods
# This class acts as a container for passing UAV to the simulation
class FlightParams:
  def __init__(self, uav: UAV_BASE):

    self.uav = uav
    
  # Function to get Controller's numerical Parameters from YAML config file
  @staticmethod
  def get_controller_config(gains_config_filename: str, uav_name: str):
    """Load the Controller gains from its YAML file."""
    config_path = Path.cwd() / "acsl_pychrono/uav" / uav_name / "Controller_Gains" / gains_config_filename
    
    # Load and return the configuration dictionary
    try:
      with open(config_path, "r") as f:
        print(f"[INFO] Loading Gains from file: {config_path}")
        return yaml.safe_load(f)
    except FileNotFoundError:
      raise FileNotFoundError(f"Controller gains config file not found at {config_path}")

  @staticmethod
  def get_scaled_matrix_from_config(config_dict: dict, var_name: str):
    """Utility function to get a scaled matrix from a config section."""
    
    param_dict = config_dict[var_name]
    scaling_factor = param_dict["scaling_factor"]
    matrix = np.array(param_dict["matrix"])
    # print(f"matrix type: {type(matrix)}" )
    
    result = np.matrix(scaling_factor * matrix)
    # print(f"{var_name}, type: {type(result)}:\n{result} \n************")
    return result
  
  @staticmethod
  def get_scalar_from_config(config_dict: dict, var_name: str):
    """Utility function to get a scalar from a config section."""
    
    result = config_dict[var_name]
    # print(f"{var_name}, type: {type(result)}:\n{result} \n************")
    return result
