import math
import numpy as np
import pychrono as chrono
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import BaseUserDefinedTrajectory

class SquareTrajectory(BaseUserDefinedTrajectory):
  def __init__(self, flight_params: FlightParams, mfloor, mfloor_Yposition) -> None:
    self.square_side_size = 5
    self.linear_velocity_trajectory = 3.5 # 1.0
    self.altitude_trajectory = -1
    self.controller_start_time = flight_params.controller_start_time
    self.time_side = ( # time required to complete one side of the square
    self.square_side_size / self.linear_velocity_trajectory
    ) 

    self.addVisualization(mfloor, mfloor_Yposition)

  def computeUserDefinedTrajectory(self, t):
    "Square trajectory at a constant altitude"

    # Account for the delay introduced by controller_start_time
    self.t_adjusted = t - self.controller_start_time
    
    translational_position_in_I_user = np.zeros((3,1))
    translational_velocity_in_I_user = np.zeros((3,1))
    translational_acceleration_in_I_user = np.zeros((3,1))
    
    if self.t_adjusted < self.time_side:
      # First segment of the square
      translational_position_in_I_user[0] = self.linear_velocity_trajectory * self.t_adjusted
      translational_position_in_I_user[1] = 0
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = self.linear_velocity_trajectory
      translational_velocity_in_I_user[1] = 0
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = 0
      translational_acceleration_in_I_user[1] = 0
      translational_acceleration_in_I_user[2] = 0
    
    elif (self.t_adjusted >= self.time_side and self.t_adjusted < 2 * self.time_side):
      # Second segment of the square
      translational_position_in_I_user[0] = self.square_side_size
      translational_position_in_I_user[1] = -self.square_side_size + self.linear_velocity_trajectory * self.t_adjusted
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = 0
      translational_velocity_in_I_user[1] = self.linear_velocity_trajectory
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = 0
      translational_acceleration_in_I_user[1] = 0
      translational_acceleration_in_I_user[2] = 0
    
    return (translational_position_in_I_user,
            translational_velocity_in_I_user,
            translational_acceleration_in_I_user)  
  
  def computeUserDefinedYaw(self):
    "User-defined reference yaw angle"
        
    if self.t_adjusted < self.time_side:
      # First segment of the square
      psi_ref = 0
      psi_ref_dot = 0
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.time_side and self.t_adjusted <  2 * self.time_side):
      # Second segment of the square
      psi_ref = math.pi/2
      # psi_ref = min(math.pi/2, (self.t_adjusted-time_side) * 0.1)
      psi_ref_dot = 0
      psi_ref_ddot = 0
    
    return (psi_ref, psi_ref_dot, psi_ref_ddot)
  
  def addVisualization(self, mfloor, mfloor_Yposition):
    "Add square trajectory visualization to the Chrono body (mfloor)"
    # Create a ChLinePath geometry, and insert sub-paths
    mpath = chrono.ChLinePath()

    seg1 = chrono.ChLineSegment(
      chrono.ChVectorD(0,abs(self.altitude_trajectory) + mfloor_Yposition,0),
      chrono.ChVectorD(self.square_side_size,abs(self.altitude_trajectory) + mfloor_Yposition,0)
    )
    seg2 = chrono.ChLineSegment(
      chrono.ChVectorD(self.square_side_size, abs(self.altitude_trajectory) + mfloor_Yposition, 0),
      chrono.ChVectorD(self.square_side_size, abs(self.altitude_trajectory) + mfloor_Yposition, self.square_side_size)
    )
    seg3 = chrono.ChLineSegment(
      chrono.ChVectorD(self.square_side_size, abs(self.altitude_trajectory) + mfloor_Yposition, self.square_side_size),
      chrono.ChVectorD(0, abs(self.altitude_trajectory) + mfloor_Yposition, self.square_side_size)
    )
    seg4 = chrono.ChLineSegment(
      chrono.ChVectorD(0, abs(self.altitude_trajectory) + mfloor_Yposition, self.square_side_size),
      chrono.ChVectorD(0,abs(self.altitude_trajectory) + mfloor_Yposition, 0)
    )

    mpath.AddSubLine(seg1)
    mpath.AddSubLine(seg2)
    mpath.AddSubLine(seg3)
    mpath.AddSubLine(seg4)
    mpath.Set_closed(True)

    # Create a ChLineShape, a visualization asset for lines.
    # The ChLinePath is a special type of ChLine and it can be visualized.
    mpathasset = chrono.ChLineShape()
    mpathasset.SetLineGeometry(mpath)
    mpathasset.SetColor(chrono.ChColor(0,0,0))
    mfloor.AddVisualShape(mpathasset)