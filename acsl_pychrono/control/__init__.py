# acsl_pychrono/control/__init__.py
from pathlib import Path
import importlib
import pkgutil

# Sub-packages that hold shared helpers rather than a controller
_HELPER_PACKAGES = {"rkhs"}

# Discover available controller modules dynamically
_package_path = Path(__file__).parent
_discovered_controllers = {
  m.name: m.name
  for m in pkgutil.iter_modules([str(_package_path)])
  if m.ispkg and not m.name.startswith("__") and m.name not in _HELPER_PACKAGES
}

def get_controller_classes(controller_type: str):
  """
  Dynamically load controller classes:
  returns (GainsClass, ControllerClass, LoggerClass)
  """
  if controller_type not in _discovered_controllers:
    raise ValueError(f"Unknown controller type: {controller_type}")

  module_name = _discovered_controllers[controller_type]
  module = importlib.import_module(f".{module_name}", package=__name__)

  try:
    GainsClass = getattr(module, f"{controller_type}Gains")
    ControllerClass = getattr(module, f"{controller_type}")
    LoggerClass = getattr(module, f"{controller_type}Logger")
  except AttributeError as e:
    raise ImportError(
      f"[ERROR] {controller_type} module missing required class: {e}"
    )

  return GainsClass, ControllerClass, LoggerClass


def available_controllers():
  """Return list of available controller types"""
  return sorted(_discovered_controllers.keys())

def instantiateController(controller_type: str, ode_input, flight_params, timestep):
  GainsClass, ControllerClass, LoggerClass = get_controller_classes(controller_type)

  gains = GainsClass(flight_params)
  controller = ControllerClass(gains, ode_input, flight_params, timestep)
  logger = LoggerClass(gains)

  return gains, controller, logger