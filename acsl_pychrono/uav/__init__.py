# acsl_pychrono/uav/__init__.py
from pathlib import Path
import importlib
import pkgutil
import yaml
from .uav_base import UAV_BASE

# Dynamically discover UAV modules in the current package instead of hardcoding them
_package_path = Path(__file__).parent # Path to the current package directory
_discovered_uavs = {                  # Map of UAV type to module name
  m.name: "UAV"                                 
  for m in pkgutil.iter_modules([str(_package_path)])   
  if m.ispkg and not m.name.startswith("__")
}

def instantiateUAV(uav_name: str, controller_name: str):
  """Dynamically instantiate a UAV and its controller parameters based on the UAV type."""

  if uav_name not in _discovered_uavs:
    raise ValueError(f"Unknown UAV type: {uav_name}")

  # Import the UAV module dynamically
  module_name = _discovered_uavs[uav_name]
  module = importlib.import_module(f".{uav_name}", package=__name__)

  # Get the UAV class and its controller parameters class
  UAV_Class = getattr(module, module_name)
  
  # Instantiate UAV and its controller parameters
  uav = UAV_Class(controller_name)
  return uav

# Function to get UAV configuration dictionary
def get_uav_config(uav_name: str):
  """Load the UAV configuration from its YAML file."""

  module_name = _discovered_uavs.get(uav_name) # Get module name
  if not module_name:
    raise ValueError(f"Unknown UAV type: {module_name}")

  # Construct path to the config YAML file
  config_path = _package_path / uav_name / f"{uav_name}_config.yaml"
  if not config_path.exists():
    raise FileNotFoundError(f"Config not found for {uav_name} at {config_path}")

 # Load and return the configuration dictionary
  with open(config_path, "r") as f:
    return yaml.safe_load(f)

# acsl_pychrono/uav/__init__.py
from pathlib import Path
import importlib
import pkgutil
import yaml
from .uav_base import UAV_BASE

# Dynamically discover UAV modules in the current package instead of hardcoding them
_package_path = Path(__file__).parent # Path to the current package directory
_discovered_uavs = {                  # Map of UAV type to module name
  m.name: "UAV"                                 
  for m in pkgutil.iter_modules([str(_package_path)])   
  if m.ispkg and not m.name.startswith("__")
}

def instantiateUAV(uav_name: str, controller_name: str):
  """Dynamically instantiate a UAV and its controller parameters based on the UAV type."""

  if uav_name not in _discovered_uavs:
    raise ValueError(f"Unknown UAV type: {uav_name}")

  # Import the UAV module dynamically
  module_name = _discovered_uavs[uav_name]
  module = importlib.import_module(f".{uav_name}", package=__name__)

  # Get the UAV class and its controller parameters class
  UAV_Class = getattr(module, module_name)
  
  # Instantiate UAV and its controller parameters
  uav = UAV_Class(controller_name)
  return uav

# Function to get UAV configuration dictionary
def get_uav_config(uav_name: str):
  """Load the UAV configuration from its YAML file."""

  module_name = _discovered_uavs.get(uav_name) # Get module name
  if not module_name:
    raise ValueError(f"Unknown UAV type: {module_name}")

  # Construct path to the config YAML file
  config_path = _package_path / uav_name / f"{uav_name}_config.yaml"
  if not config_path.exists():
    raise FileNotFoundError(f"Config not found for {uav_name} at {config_path}")

 # Load and return the configuration dictionary
  with open(config_path, "r") as f:
    return yaml.safe_load(f)
