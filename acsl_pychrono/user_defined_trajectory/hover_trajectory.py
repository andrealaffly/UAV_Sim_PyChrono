import math
import numpy as np
import pychrono as chrono
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import BaseUserDefinedTrajectory

class HoverTrajectory(BaseUserDefinedTrajectory):
  def __init__(self, flight_params: FlightParams, mfloor, mfloor_Yposition) -> None:
    self.altitude_trajectory = -1
    self.controller_start_time = flight_params.controller_start_time

    self.addVisualization(mfloor, mfloor_Yposition)

  def computeUserDefinedTrajectory(self, t):
    "Hover trajectory at a constant altitude"

    # Account for the delay introduced by controller_start_time
    self.t_adjusted = t - self.controller_start_time
    
    translational_position_in_I_user = np.zeros((3,1))
    translational_velocity_in_I_user = np.zeros((3,1))
    translational_acceleration_in_I_user = np.zeros((3,1))
    
    translational_position_in_I_user[0] = 0 
    translational_position_in_I_user[1] = 0
    translational_position_in_I_user[2] = self.altitude_trajectory
    
    translational_velocity_in_I_user[0] = 0
    translational_velocity_in_I_user[1] = 0
    translational_velocity_in_I_user[2] = 0
    
    translational_acceleration_in_I_user[0] = 0
    translational_acceleration_in_I_user[1] = 0
    translational_acceleration_in_I_user[2] = 0
    
    return (translational_position_in_I_user,
            translational_velocity_in_I_user,
            translational_acceleration_in_I_user)  
  
  def computeUserDefinedYaw(self):
    "User-defined reference yaw angle"

    psi_ref = 0
    psi_ref_dot = 0
    psi_ref_ddot = 0
    
    return (psi_ref, psi_ref_dot, psi_ref_ddot)
  
  def addVisualization(self, mfloor, mfloor_Yposition):
    "Add hover point visualization to the Chrono body (mfloor)"

    # Define the position of the hover point in absolute coordinates
    hover_position = chrono.ChVectorD(0, abs(self.altitude_trajectory) + mfloor_Yposition, 0)

    # Create a sphere shape with a small radius
    sphere_shape = chrono.ChSphereShape()
    sphere_shape.GetSphereGeometry().rad = 0.01 

    # Set color for the sphere (optional, for visualization)
    sphere_shape.SetColor(chrono.ChColor(0, 0, 0)) 

    # Attach the sphere shape to the floor at the hover point position
    mfloor.AddVisualShape(sphere_shape, chrono.ChFrameD(hover_position))


