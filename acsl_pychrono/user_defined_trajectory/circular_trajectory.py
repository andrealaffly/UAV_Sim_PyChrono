import math
import numpy as np
import pychrono as chrono
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import BaseUserDefinedTrajectory

class CircularTrajectory(BaseUserDefinedTrajectory):
  def __init__(self, flight_params: FlightParams, mfloor, mfloor_Yposition) -> None:
    self.radius_trajectory = 3
    self.angular_velocity_trajectory = 0.2
    self.altitude_trajectory = -1
    self.controller_start_time = flight_params.controller_start_time

    self.addVisualization(mfloor, mfloor_Yposition)

  def computeUserDefinedTrajectory(self, t):
    "Circular trajectory at a constant altitude"

    # Account for the delay introduced by controller_start_time
    self.t_adjusted = t - self.controller_start_time
    
    translational_position_in_I_user = np.zeros((3,1))
    translational_velocity_in_I_user = np.zeros((3,1))
    translational_acceleration_in_I_user = np.zeros((3,1))
    
    # Position
    translational_position_in_I_user[0] = (
      self.radius_trajectory*math.cos(-self.angular_velocity_trajectory * self.t_adjusted) - self.radius_trajectory
    )
    translational_position_in_I_user[1] = (
      self.radius_trajectory*math.sin(-self.angular_velocity_trajectory * self.t_adjusted)
    )
    translational_position_in_I_user[2] = self.altitude_trajectory
    
    # Velocity
    translational_velocity_in_I_user[0] = (
      -self.angular_velocity_trajectory * self.radius_trajectory * 
      math.sin(self.angular_velocity_trajectory * self.t_adjusted)
    )
    translational_velocity_in_I_user[1] = (
      -self.angular_velocity_trajectory * self.radius_trajectory * 
      math.cos(self.angular_velocity_trajectory * self.t_adjusted)
    )
    translational_velocity_in_I_user[2] = 0
    
    # Acceleration
    translational_acceleration_in_I_user[0] = (
      -self.angular_velocity_trajectory**2 * self.radius_trajectory * 
      math.cos(self.angular_velocity_trajectory * self.t_adjusted)
    )
    translational_acceleration_in_I_user[1] = (
      self.angular_velocity_trajectory**2 * self.radius_trajectory * 
      math.sin(self.angular_velocity_trajectory * self.t_adjusted)
    )
    translational_acceleration_in_I_user[2] = 0
    
    return (translational_position_in_I_user,
            translational_velocity_in_I_user,
            translational_acceleration_in_I_user)  
  
  def computeUserDefinedYaw(self):
    "User-defined reference yaw angle"

    if (self.t_adjusted < 0.5):
      psi_ref = -self.angular_velocity_trajectory * self.t_adjusted  - math.pi/8
    elif (self.t_adjusted >= 0.5 and self.t_adjusted < 1):
      psi_ref = -self.angular_velocity_trajectory * self.t_adjusted  - math.pi/4
    elif (self.t_adjusted >= 1 and self.t_adjusted < 1.5):
      psi_ref = -self.angular_velocity_trajectory * self.t_adjusted  - 3*math.pi/8
    else:    
      psi_ref = -self.angular_velocity_trajectory * self.t_adjusted  - math.pi/2
        
    psi_ref_dot = -self.angular_velocity_trajectory
    psi_ref_ddot = 0
    
    return (psi_ref, psi_ref_dot, psi_ref_ddot)
  
  def addVisualization(self, mfloor, mfloor_Yposition):
    "Add circular trajectory visualization to the Chrono body (mfloor)"
    # Create a ChLinePath geometry, and insert sub-paths 
    mpath = chrono.ChLinePath()
    marc1 = chrono.ChLineArc(
      chrono.ChCoordsysD(
        chrono.ChVectorD(-self.radius_trajectory, abs(self.altitude_trajectory) + mfloor_Yposition, 0),
        chrono.ChQuaternionD(0.70710678118,0.70710678118,0,0)
      ),
      self.radius_trajectory,
      chrono.CH_C_2PI,
      0,
      False
    )
    mpath.AddSubLine(marc1)
    mpath.Set_closed(True)

    # Create a ChLineShape, a visualization asset for lines.
    # The ChLinePath is a special type of ChLine and it can be visualized.
    mpathasset = chrono.ChLineShape()
    mpathasset.SetLineGeometry(mpath)
    mpathasset.SetColor(chrono.ChColor(0,0,0))
    mfloor.AddVisualShape(mpathasset)