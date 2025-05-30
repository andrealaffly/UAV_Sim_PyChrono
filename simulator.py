def main():
    pass

if __name__ == '__main__':
    main()

from acsl_pychrono.simulation.simulation import Simulation
import acsl_pychrono.user_defined_trajectory as Traj
import acsl_pychrono.control as Ctrl
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.ode_input import OdeInput
from acsl_pychrono.flight_params import FlightParams

def simulateMission():
   
    sim = Simulation()

    # Instantiation of classes
    flight_params = FlightParams()
    ode_input = OdeInput()
    sim.setGravitationalAcceleration(flight_params)

    # Instantiate user-defined trajectory
    user_defined_trajectory: Traj.BaseUserDefinedTrajectory = Traj.instantiateTrajectory(
        sim.simulation_config,
        flight_params,
        sim.mfloor,
        sim.mfloor_Yposition
    )

    # Instantiation of controller, gains, and logger
    (gains, controller, logger) = Ctrl.instantiateController(
        sim.simulation_config.controller_type,
        ode_input,
        flight_params,
        sim.simulation_config.timestep
    )

    sim.assignInstances(
        flight_params,
        ode_input,
        user_defined_trajectory,
        gains,
        controller,
        logger
    )

    sim.runSimulationLoop()

    # Convert logged data to a dictionary
    log_dict = logger.toDictionary()
    # Export data to MATLAB workspace
    Logging.saveMatlabWorkspaceLog(log_dict, gains, sim.simulation_config.controller_type)