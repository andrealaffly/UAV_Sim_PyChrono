import traceback
from acsl_pychrono.simulation.simulation import Simulation
import acsl_pychrono.user_defined_trajectory as Traj
import acsl_pychrono.control as Ctrl
import acsl_pychrono.uav as UAV_Module
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.simulation.ode_input import OdeInput
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.ga_tuner.integration.controller_factory import instantiateControllerWithGA

def simulateMission(sim: Simulation, git_info: dict | None = None):
  
  # Instantiation of the UAV object
  uav = UAV_Module.instantiateUAV(
    sim.vehicle_config.uav_name,
    sim.mission_config.controller_type
  )
  
  # Instantiation of classes
  flight_params = FlightParams(uav)
  ode_input = OdeInput()
  sim.setGravitationalAcceleration(flight_params)

  # Instantiate user-defined trajectory
  user_defined_trajectory: Traj.BaseUserDefinedTrajectory = Traj.instantiateTrajectory(
    sim.mission_config,
    flight_params,
    sim.mfloor,
    sim.mfloor_Yposition
  )

  wrapper_params = getattr(getattr(sim, "simulation_config", None), "wrapper_params", None)
  external_params = getattr(wrapper_params, "external_controller_params", None) if wrapper_params else None

  if external_params:
    (gains, controller, logger) = instantiateControllerWithGA(
      sim.mission_config.controller_type,
      ode_input,
      flight_params,
      sim.mission_config.timestep,
      wrapper_params=wrapper_params
    )
  else:
    # Instantiation of controller, gains, and logger
    (gains, controller, logger) = Ctrl.instantiateController(
      sim.mission_config.controller_type,
      ode_input,
      flight_params,
      sim.mission_config.timestep
    )

  sim.assignInstances(
    flight_params,
    ode_input,
    user_defined_trajectory,
    gains,
    controller,
    logger
  )

  try:
    sim.runSimulationLoop()
  except Exception as e:
    print(f"\n[ERROR] Simulation crashed: {e}")
    traceback.print_exc()
  finally:
    print("\n[INFO] Saving logs before exit...")
    log_dict = logger.toDictionary()
    Logging.saveMatlabWorkspaceLog(
      log_dict,
      gains,
      sim.simulation_config,
      git_info
    )

  return log_dict