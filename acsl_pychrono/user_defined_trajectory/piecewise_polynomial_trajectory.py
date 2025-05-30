import math
import numpy as np
import json
import os
from numpy.polynomial import polynomial
import pychrono as chrono
from acsl_pychrono.config.config import SimulationConfig
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.user_defined_trajectory.trajectory_auxillary import TrajectoryAuxillary
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import BaseUserDefinedTrajectory

class PiecewisePolynomialTrajectory(BaseUserDefinedTrajectory):
  def __init__(
      self, 
      flight_params: FlightParams,
      mfloor,
      mfloor_Yposition,
      simulation_config: SimulationConfig
      ) -> None:
    
    self.controller_start_time = flight_params.controller_start_time
    self.trajectory_data_path = simulation_config.trajectory_data_path

    self.translational_position_in_I_user = np.zeros((3,1))
    self.translational_velocity_in_I_user = np.zeros((3,1))
    self.translational_acceleration_in_I_user = np.zeros((3,1))
    self.t_adjusted = 0
    self.segment = 0
    self.velocity_norm2D = 0
    self.psi_ref = 0
    self.psi_ref_dot = 0
    self.psi_ref_ddot = 0
    self.psi_ref_previous = 0

    self.setParameters()
    self.addVisualization(mfloor, mfloor_Yposition)

  def setParameters(self):
    # Define path to the JSON file
    # Prepend working directory and "/params/user_defined_trajectory"
    base_path = os.path.join(os.getcwd(), "params", "user_defined_trajectory")
    full_path = os.path.join(base_path, self.trajectory_data_path)

    # Load the JSON file
    with open(full_path, "r") as file:
      data = json.load(file)

    # Load waypoints, waypoint times, and polynomial coefficients
    self.waypoints = np.array(data["waypoints"])                          
    self.waypointTimes = np.array(data["waypoint_times"])                
    self.pp_coefficients = np.array(data["piecewise_polynomial_coefficients"])
    self.pp_coefficients = np.flip(self.pp_coefficients, axis=1)  # reverse coefficients order per row

    (self.position_coef_x,
     self.position_coef_y,
     self.position_coef_z
    ) = TrajectoryAuxillary.PolyCoefAssigning(self.pp_coefficients)
    
    self.velocity_coef_x = TrajectoryAuxillary.PolyderMatrix(self.position_coef_x)
    self.velocity_coef_y = TrajectoryAuxillary.PolyderMatrix(self.position_coef_y)
    self.velocity_coef_z = TrajectoryAuxillary.PolyderMatrix(self.position_coef_z)

    self.acceleration_coef_x = TrajectoryAuxillary.PolyderMatrix(self.velocity_coef_x)
    self.acceleration_coef_y = TrajectoryAuxillary.PolyderMatrix(self.velocity_coef_y)
    self.acceleration_coef_z = TrajectoryAuxillary.PolyderMatrix(self.velocity_coef_z)

    self.jerk_coef_x = TrajectoryAuxillary.PolyderMatrix(self.acceleration_coef_x)
    self.jerk_coef_y = TrajectoryAuxillary.PolyderMatrix(self.acceleration_coef_y)
    self.jerk_coef_z = TrajectoryAuxillary.PolyderMatrix(self.acceleration_coef_z)

  def computeUserDefinedTrajectory(self, t):
    """
    Piecewise polynomial trajectory.
    t: Current simulation time
    """
    # Account for the delay introduced by controller_start_time
    t = t - self.controller_start_time

    (self.t_adjusted,
     self.segment
    ) = TrajectoryAuxillary.PolyTimeAdjusted(self.waypointTimes, t)

    self.translational_position_in_I_user[0] = polynomial.polyval(
      self.t_adjusted,
      self.position_coef_x[self.segment,:]
    )
    self.translational_position_in_I_user[1] = polynomial.polyval(
      self.t_adjusted,
      self.position_coef_y[self.segment,:]
    )
    self.translational_position_in_I_user[2] = polynomial.polyval(
      self.t_adjusted,
      self.position_coef_z[self.segment,:]
    )

    self.translational_velocity_in_I_user[0] = polynomial.polyval(
      self.t_adjusted,
      self.velocity_coef_x[self.segment,:]
    )
    self.translational_velocity_in_I_user[1] = polynomial.polyval(
      self.t_adjusted,
      self.velocity_coef_y[self.segment,:]
    )
    self.translational_velocity_in_I_user[2] = polynomial.polyval(
      self.t_adjusted,
      self.velocity_coef_z[self.segment,:]
    )

    self.translational_acceleration_in_I_user[0] = polynomial.polyval(
      self.t_adjusted,
      self.acceleration_coef_x[self.segment,:]
    )
    self.translational_acceleration_in_I_user[1] = polynomial.polyval(
      self.t_adjusted,
      self.acceleration_coef_y[self.segment,:]
    )
    self.translational_acceleration_in_I_user[2] = polynomial.polyval(
      self.t_adjusted,
      self.acceleration_coef_z[self.segment,:]
    )
    
    return (self.translational_position_in_I_user,
            self.translational_velocity_in_I_user,
            self.translational_acceleration_in_I_user)  
  
  def computeUserDefinedYaw(self):
    "User-defined reference yaw angle"

    self.velocity_norm2D = TrajectoryAuxillary.Norm2D(
      self.velocity_coef_x[self.segment,:],
      self.velocity_coef_y[self.segment,:],
      self.t_adjusted
    )
    
    if self.t_adjusted == 0:
      self.psi_ref = 0
      self.psi_ref_dot = 0
      self.psi_ref_ddot = 0
      
    elif (self.t_adjusted > 0 and self.velocity_norm2D < 1e-5):
      self.psi_ref = self.psi_ref_previous
      self.psi_ref_dot = 0
      self.psi_ref_ddot = 0
      
    else:
      self.psi_ref = TrajectoryAuxillary.YawComputation(
        self.velocity_coef_x[self.segment,:],
        self.velocity_coef_y[self.segment,:],
        self.t_adjusted
      )
      self.psi_ref_dot = TrajectoryAuxillary.YawDotComputation(
        self.velocity_coef_x[self.segment,:],
        self.velocity_coef_y[self.segment,:],
        self.acceleration_coef_x[self.segment,:],
        self.acceleration_coef_y[self.segment,:],
        self.t_adjusted
      )
      self.psi_ref_ddot = TrajectoryAuxillary.YawDotDotComputation(
        self.velocity_coef_x[self.segment,:],
        self.velocity_coef_y[self.segment,:],
        self.acceleration_coef_x[self.segment,:],
        self.acceleration_coef_y[self.segment,:],
        self.jerk_coef_x[self.segment,:],
        self.jerk_coef_y[self.segment,:],
        self.t_adjusted
      )
      
    self.psi_ref_previous = self.psi_ref

    return (self.psi_ref, self.psi_ref_dot, self.psi_ref_ddot)
  
  def computePositionVector(self, samplingTime = 0.01):
    "Drawing ChLineSegement for visualization of the trajectory in simulation"
    
    self.samplingTime = samplingTime
    sampling_time_vector = TrajectoryAuxillary.SamplingTimeVector(self.waypointTimes, self.samplingTime)

    pos_x = np.zeros(sampling_time_vector.size)
    pos_y = np.zeros(sampling_time_vector.size)
    pos_z = np.zeros(sampling_time_vector.size)
    
    for i in range(sampling_time_vector.size):
      (position, _, _)= self.computeUserDefinedTrajectory(sampling_time_vector[i])
      pos_x[i] = position[0]
      pos_y[i] = position[1]
      pos_z[i] = position[2]
    
    return (pos_x, pos_y, pos_z)
  
  def addVisualization(self, mfloor, mfloor_Yposition):
    "Add circular trajectory visualization to the Chrono body (mfloor)"
     # Create a ChLinePath geometry, and insert sub-paths
    mpath = chrono.ChLinePath()
    (pos_x, pos_y, pos_z) = self.computePositionVector(0.01)
    
    for i in range(pos_x.size - 1):
      mpath.AddSubLine(
        chrono.ChLineSegment(
          chrono.ChVectorD(pos_x[i],
                           -pos_z[i] + mfloor_Yposition,
                           pos_y[i]
          ),
          chrono.ChVectorD(pos_x[i+1],
                           -pos_z[i+1] + mfloor_Yposition,
                           pos_y[i+1]
          )
        )
      )
    
    # Create a ChLineShape, a visualization asset for lines.
    # The ChLinePath is a special type of ChLine and it can be visualized.
    mpathasset = chrono.ChLineShape()
    mpathasset.SetLineGeometry(mpath)
    mpathasset.SetColor(chrono.ChColor(0,0,0))
    mfloor.AddVisualShape(mpathasset)
    