import math
import numpy as np
import json
import os
from typing import Tuple
from numpy.polynomial import polynomial
import pychrono as chrono
from acsl_pychrono.config.config import MissionConfig
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.user_defined_trajectory.trajectory_auxillary import TrajectoryAuxillary
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import BaseUserDefinedTrajectory
from acsl_pychrono.user_defined_trajectory.landing import LandingPolynomials

class PiecewisePolynomialTrajectory(BaseUserDefinedTrajectory):
  def __init__(
      self, 
      flight_params: FlightParams,
      mfloor,
      mfloor_Yposition,
      mission_config: MissionConfig
      ) -> None:
    
    self.controller_start_time = flight_params.uav.controller_start_time
    self.trajectory_data_path = mission_config.trajectory_data_path
    self.hover_after_trajectory_time_seconds = mission_config.hover_after_trajectory_time_seconds

    self.translational_position_in_I_user = np.zeros((3, 1))
    self.translational_velocity_in_I_user = np.zeros((3, 1))
    self.translational_acceleration_in_I_user = np.zeros((3, 1))
    self.translational_position_in_I_user_previous = np.zeros((3, 1))
    self.t_adjusted = 0
    self.segment = 0
    self.velocity_norm2D = 0
    self.psi_ref = 0
    self.psi_ref_dot = 0
    self.psi_ref_ddot = 0
    self.psi_ref_previous = 0

    self.landing_polynomials = LandingPolynomials()

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

    self.landing_start_time_seconds = (
      self.controller_start_time + self.waypointTimes[-1] + self.hover_after_trajectory_time_seconds
    )
    self.landing_end_times_seconds = (
      self.landing_start_time_seconds + self.landing_polynomials.get("1meterIn4seconds", "meta", "duration_seconds")
    )

  def computeUserDefinedTrajectory(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Piecewise polynomial trajectory.
    t: Current simulation time
    """
    # Account for the delay introduced by controller_start_time
    t -= self.controller_start_time

    self.time_current_minus_takeoff = t - self.controller_start_time

    (self.t_adjusted, self.segment) = TrajectoryAuxillary.PolyTimeAdjusted(self.waypointTimes, self.time_current_minus_takeoff)

    landing_start = self.landing_start_time_seconds
    landing_end = self.landing_end_times_seconds
    last_waypoint_time = self.waypointTimes[-1]

    # If the current time has passed the last waypoint time but it is not greater than "landing_start_time_seconds",
    # set the trajectory to perform HOVER in the last position
    if self.time_current_minus_takeoff >= last_waypoint_time and self.time_current_minus_takeoff < landing_start:
      self.translational_position_in_I_user = self.translational_position_in_I_user_previous.copy()
      self.translational_velocity_in_I_user = np.zeros((3, 1))
      self.translational_acceleration_in_I_user = np.zeros((3, 1))

    # If the current time has passed the "landing_start_time_seconds" but not the "landing_end_times_seconds",
    # perform the LANDING meneuvre
    elif landing_start <= self.time_current_minus_takeoff < landing_end:
      t_landing = self.time_current_minus_takeoff - landing_start
      self.translational_position_in_I_user = np.array([
        [self.translational_position_in_I_user_previous[0].item()],
        [self.translational_position_in_I_user_previous[1].item()],
        [polynomial.polyval(t_landing, self.landing_polynomials.get("1meterIn4seconds", "position", "z"))]
      ])
      self.translational_velocity_in_I_user = np.array([
        [0.0],
        [0.0],
        [polynomial.polyval(t_landing, self.landing_polynomials.get("1meterIn4seconds", "velocity", "z"))]
      ])
      self.translational_acceleration_in_I_user = np.array([
        [0.0],
        [0.0],
        [polynomial.polyval(t_landing, self.landing_polynomials.get("1meterIn4seconds", "acceleration", "z"))]
      ])

    # If the current time has passed the "landing_end_times_seconds",
    # stay at the last good location
    elif self.time_current_minus_takeoff >= landing_end:
      self.translational_position_in_I_user = self.translational_position_in_I_user_previous.copy()
      self.translational_velocity_in_I_user = np.zeros((3, 1))
      self.translational_acceleration_in_I_user = np.zeros((3, 1))

    # Follow the piecewise polynomial trajectory
    else:
      coefs = [
        [self.position_coef_x, self.position_coef_y, self.position_coef_z],
        [self.velocity_coef_x, self.velocity_coef_y, self.velocity_coef_z],
        [self.acceleration_coef_x, self.acceleration_coef_y, self.acceleration_coef_z],
      ]

      outputs = [
        self.translational_position_in_I_user,
        self.translational_velocity_in_I_user,
        self.translational_acceleration_in_I_user,
      ]

      for k in range(3):
        for i in range(3):
          outputs[k][i] = polynomial.polyval(self.t_adjusted, coefs[k][i][self.segment, :])

    # Final return for all code paths
    self.translational_position_in_I_user_previous = self.translational_position_in_I_user.copy()
    return (
      self.translational_position_in_I_user,
      self.translational_velocity_in_I_user,
      self.translational_acceleration_in_I_user
    )

  def computeUserDefinedYaw(self):
    "User-defined reference yaw angle"

    last_waypoint_time = self.waypointTimes[-1]

    self.velocity_norm2D = TrajectoryAuxillary.Norm2D(
      self.velocity_coef_x[self.segment,:],
      self.velocity_coef_y[self.segment,:],
      self.t_adjusted
    )
          
    if (
      (self.t_adjusted >= 0 and self.velocity_norm2D < 1e-5) or
      (self.time_current_minus_takeoff >= last_waypoint_time)
    ):
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
      pos_x[i] = position[0].item()
      pos_y[i] = position[1].item()
      pos_z[i] = position[2].item()
    
    return (pos_x, pos_y, pos_z)
  
  def addVisualization(self, mfloor, mfloor_Yposition):
    "Add circular trajectory visualization to the Chrono body (mfloor)"
     # Create a ChLinePath geometry, and insert sub-paths
    mpath = chrono.ChLinePath()
    (pos_x, pos_y, pos_z) = self.computePositionVector(0.01)
    
    for i in range(pos_x.size - 1):
      mpath.AddSubLine(
        chrono.ChLineSegment(
          chrono.ChVector3d(pos_x[i],
                           -pos_z[i] + mfloor_Yposition,
                           pos_y[i]
          ),
          chrono.ChVector3d(pos_x[i+1],
                           -pos_z[i+1] + mfloor_Yposition,
                           pos_y[i+1]
          )
        )
      )
    
    # Create a ChVisualShapeLine, a visualization asset for lines.
    # The ChLinePath is a special type of ChLine and it can be visualized.
    mpathasset = chrono.ChVisualShapeLine()
    mpathasset.SetLineGeometry(mpath)
    mpathasset.SetColor(chrono.ChColor(0,0,0))
    mfloor.AddVisualShape(mpathasset)
    