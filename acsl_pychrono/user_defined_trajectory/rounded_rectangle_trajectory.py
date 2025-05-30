import math
import numpy as np
import pychrono as chrono
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import BaseUserDefinedTrajectory

class RoundedRectangleTrajectory(BaseUserDefinedTrajectory):
  def __init__(self, flight_params: FlightParams, mfloor, mfloor_Yposition) -> None:
    """
    Example of past values:
    - 22.5 seconds "Stadium": 5, 0, 2, 1, -1
    - "Stadium": 4, 0, 0.3, 1.2, -1.1
    - "Stadium": 5, 0, 2, 1, -1
    """
    self.length_horizontal = 4
    self.length_vertical = 0
    self.rounding_radius = 1
    self.linear_velocity_trajectory = 1.5
    self.altitude_trajectory = -1
    self.controller_start_time = flight_params.controller_start_time
    self.omega_corner = ( # Compute the constant angular velocity on smoothed corners
      self.linear_velocity_trajectory / self.rounding_radius
    )
    self.t_1 = self.length_horizontal / self.linear_velocity_trajectory;
    self.t_2 = self.t_1 + (self.rounding_radius * math.pi/2) / self.linear_velocity_trajectory;
    self.t_3 = self.t_2 + self.length_vertical / self.linear_velocity_trajectory;
    self.t_4 = self.t_3 + (self.rounding_radius * math.pi/2) / self.linear_velocity_trajectory;
    self.t_5 = self.t_4 + self.length_horizontal / self.linear_velocity_trajectory;
    self.t_6 = self.t_5 + (self.rounding_radius * math.pi/2) / self.linear_velocity_trajectory;
    self.t_7 = self.t_6 + self.length_vertical / self.linear_velocity_trajectory;
    self.t_8 = self.t_7 + (self.rounding_radius * math.pi/2) / self.linear_velocity_trajectory;
  
    self.addVisualization(mfloor, mfloor_Yposition)

  def computeUserDefinedTrajectory(self, t):
    """
    Rounded rectangle trajectory at a constant altitude

    Segment definitions (in XY plane):
    - Segment 1: From (0, 0)                                                                         to (length_horizontal, 0)
    - Segment 2: From (length_horizontal, 0)                                                         to (length_horizontal + rounding_radius, rounding_radius)
    - Segment 3: From (length_horizontal + rounding_radius, rounding_radius)                         to (length_horizontal + rounding_radius, length_vertical + rounding_radius)
    - Segment 4: From (length_horizontal + rounding_radius, length_vertical + rounding_radius)       to (length_horizontal, length_vertical + 2*rounding_radius)
    - Segment 5: From (length_horizontal, length_vertical + 2*rounding_radius)                       to (0, length_vertical + 2*rounding_radius)
    - Segment 6: From (0, length_vertical + 2*rounding_radius)                                       to (-rounding_radius, length_vertical + rounding_radius)
    - Segment 7: From (-rounding_radius, length_vertical + rounding_radius)                          to (-rounding_radius, rounding_radius)
    - Segment 8: From (-rounding_radius, rounding_radius)                                            to (0, 0)

    Timing per segment (assuming constant linear velocity):
    - Segment 1: From t_0 = 0                                                          to t_1 = length_horizontal / linear_velocity_trajectory
    - Segment 2: From t_1 = length_horizontal / linear_velocity_trajectory             to t_2 = t_1 + (π/4 * rounding_radius) / linear_velocity_trajectory
    - Segment 3: From t_2                                                               to t_3 = t_2 + length_vertical / linear_velocity_trajectory
    - Segment 4: From t_3                                                               to t_4 = t_3 + (π/4 * rounding_radius) / linear_velocity_trajectory
    - Segment 5: From t_4                                                               to t_5 = t_4 + length_horizontal / linear_velocity_trajectory
    - Segment 6: From t_5                                                               to t_6 = t_5 + (π/4 * rounding_radius) / linear_velocity_trajectory
    - Segment 7: From t_6                                                               to t_7 = t_6 + length_vertical / linear_velocity_trajectory
    - Segment 8: From t_7                                                               to T_max = t_7 + (π/4 * rounding_radius) / linear_velocity_trajectory
    """

    # Account for the delay introduced by controller_start_time
    self.t_adjusted = t - self.controller_start_time
    
    translational_position_in_I_user = np.zeros((3,1))
    translational_velocity_in_I_user = np.zeros((3,1))
    translational_acceleration_in_I_user = np.zeros((3,1))

    if (self.t_adjusted >= 0 and self.t_adjusted < self.t_1):
      # 1 Top horizontal segment
      translational_position_in_I_user[0] = self.linear_velocity_trajectory * self.t_adjusted
      translational_position_in_I_user[1] = 0
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = self.linear_velocity_trajectory
      translational_velocity_in_I_user[1] = 0
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = 0
      translational_acceleration_in_I_user[1] = 0
      translational_acceleration_in_I_user[2] = 0

    elif (self.t_adjusted >= self.t_1 and self.t_adjusted < self.t_2):
      # 2 Top-right rounding radius
      translational_position_in_I_user[0] = (
        self.length_horizontal + self.rounding_radius * 
        math.cos(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_1))
      )
      translational_position_in_I_user[1] = (
        self.rounding_radius + self.rounding_radius * 
        math.sin(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_1))
      )
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = (
        self.rounding_radius * self.omega_corner * math.cos(self.omega_corner * (self.t_adjusted - self.t_1))
      )
      translational_velocity_in_I_user[1] = (
        self.rounding_radius * self.omega_corner * math.sin(self.omega_corner * (self.t_adjusted - self.t_1))
      )
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = (
        -self.rounding_radius * self.omega_corner**2 * math.sin(self.omega_corner * (self.t_adjusted - self.t_1))
      )
      translational_acceleration_in_I_user[1] = (
        self.rounding_radius * self.omega_corner**2 * math.cos(self.omega_corner * (self.t_adjusted - self.t_1))
      )
      translational_acceleration_in_I_user[2] = 0
        
    elif (self.t_adjusted >= self.t_2 and self.t_adjusted < self.t_3):
      # 3 Right vertical segment
      translational_position_in_I_user[0] = self.length_horizontal + self.rounding_radius
      translational_position_in_I_user[1] = (
        self.rounding_radius + self.linear_velocity_trajectory*(self.t_adjusted - self.t_2)
      )
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = 0
      translational_velocity_in_I_user[1] = self.linear_velocity_trajectory
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = 0
      translational_acceleration_in_I_user[1] = 0
      translational_acceleration_in_I_user[2] = 0
        
    elif (self.t_adjusted >= self.t_3 and self.t_adjusted < self.t_4):
      # 4 Bottom-right rounding radius
      translational_position_in_I_user[0] = (
        self.length_horizontal - self.rounding_radius * 
        math.sin(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_3))
      )
      translational_position_in_I_user[1] = (
        self.rounding_radius + self.length_vertical +
        self.rounding_radius * math.cos(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_3))
      )
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = (
        -self.rounding_radius * self.omega_corner * math.sin(self.omega_corner * (self.t_adjusted - self.t_3))
      )
      translational_velocity_in_I_user[1] = (
        self.rounding_radius * self.omega_corner * math.cos(self.omega_corner * (self.t_adjusted - self.t_3))
      )
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = (
        -self.rounding_radius * self.omega_corner**2 * math.cos(self.omega_corner * (self.t_adjusted - self.t_3))
      )
      translational_acceleration_in_I_user[1] = (
        -self.rounding_radius * self.omega_corner**2 * math.sin(self.omega_corner * (self.t_adjusted - self.t_3))
      )
      translational_acceleration_in_I_user[2] = 0
        
    elif (self.t_adjusted >= self.t_4 and self.t_adjusted < self.t_5):
      # 5 Bottom horizontal segment
      translational_position_in_I_user[0] = (
        self.length_horizontal - self.linear_velocity_trajectory * (self.t_adjusted - self.t_4)
      )
      translational_position_in_I_user[1] = self.length_vertical + 2*self.rounding_radius
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = -self.linear_velocity_trajectory
      translational_velocity_in_I_user[1] = 0
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = 0
      translational_acceleration_in_I_user[1] = 0
      translational_acceleration_in_I_user[2] = 0
        
    elif (self.t_adjusted >= self.t_5 and self.t_adjusted < self.t_6):       
      # 6 Bottom-left rounding radius
      translational_position_in_I_user[0] = (
        0 - self.rounding_radius * math.cos(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_5))
      )
      translational_position_in_I_user[1] = (
        self.length_vertical + self.rounding_radius 
        - self.rounding_radius * math.sin(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_5))
      )
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = (
        -self.rounding_radius * self.omega_corner * math.cos(self.omega_corner * (self.t_adjusted - self.t_5))
      )
      translational_velocity_in_I_user[1] = (
        -self.rounding_radius * self.omega_corner * math.sin(self.omega_corner * (self.t_adjusted - self.t_5))
      )
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = (
        self.rounding_radius * self.omega_corner**2 * math.sin(self.omega_corner * (self.t_adjusted - self.t_5))
      )
      translational_acceleration_in_I_user[1] = (
        -self.rounding_radius * self.omega_corner**2 * math.cos(self.omega_corner * (self.t_adjusted - self.t_5))
      )
      translational_acceleration_in_I_user[2] = 0
        
    elif (self.t_adjusted >= self.t_6 and self.t_adjusted < self.t_7):
      # 7 Left vertical segment
      translational_position_in_I_user[0] = -self.rounding_radius
      translational_position_in_I_user[1] = (
        self.rounding_radius + self.length_vertical - self.linear_velocity_trajectory * (self.t_adjusted - self.t_6)
      )
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = 0
      translational_velocity_in_I_user[1] = -self.linear_velocity_trajectory
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = 0
      translational_acceleration_in_I_user[1] = 0
      translational_acceleration_in_I_user[2] = 0
        
    elif (self.t_adjusted >= self.t_7 and self.t_adjusted < self.t_8):
      # 8 Bottom-left rounding radius
      translational_position_in_I_user[0] = (
        0 + self.rounding_radius * math.sin(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_7))
      )
      translational_position_in_I_user[1] = (
        self.rounding_radius - self.rounding_radius * 
        math.cos(-math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_7))
      )
      translational_position_in_I_user[2] = self.altitude_trajectory
      
      translational_velocity_in_I_user[0] = (
        self.rounding_radius * self.omega_corner * math.sin(self.omega_corner * (self.t_adjusted - self.t_7))
      )
      translational_velocity_in_I_user[1] = (
        -self.rounding_radius * self.omega_corner * math.cos(self.omega_corner * (self.t_adjusted - self.t_7))
      )
      translational_velocity_in_I_user[2] = 0
      
      translational_acceleration_in_I_user[0] = (
        self.rounding_radius * self.omega_corner**2 * math.cos(self.omega_corner * (self.t_adjusted - self.t_7))
      )
      translational_acceleration_in_I_user[1] = (
        self.rounding_radius * self.omega_corner**2 * math.sin(self.omega_corner * (self.t_adjusted - self.t_7))
      )
      translational_acceleration_in_I_user[2] = 0
        
    else:
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
    "User-defined reference yaw angle"  # TO BE MODIFIEEEEEEEDDDDDD!

    if (self.t_adjusted >= 0 and self.t_adjusted < self.t_1):
      # 1 Top horizontal segment
      psi_ref = 0
      psi_ref_dot = 0
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.t_1 and self.t_adjusted < self.t_2):
      # 2 Top-right rounding radius
      psi_ref = self.omega_corner * (self.t_adjusted - self.t_1)
      psi_ref_dot = self.omega_corner
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.t_2 and self.t_adjusted < self.t_3):
      # 3 Right vertical segment
      psi_ref = math.pi/2
      psi_ref_dot = 0
      psi_ref_ddot = 0
    
    elif (self.t_adjusted >= self.t_3 and self.t_adjusted < self.t_4):
      # 4 Bottom-right rounding radius
      psi_ref = math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_3)
      psi_ref_dot = self.omega_corner
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.t_4 and self.t_adjusted < self.t_5):
      # 5 Bottom horizontal segment
      psi_ref = math.pi
      psi_ref_dot = 0
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.t_5 and self.t_adjusted < self.t_6):
      # 6 Bottom-left rounding radius
      psi_ref = math.pi + self.omega_corner * (self.t_adjusted - self.t_5)
      psi_ref_dot = self.omega_corner
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.t_6 and self.t_adjusted < self.t_7):
      # 7 Left vertical segment
      psi_ref = 3*math.pi/2
      psi_ref_dot = 0
      psi_ref_ddot = 0
        
    elif (self.t_adjusted >= self.t_7 and self.t_adjusted < self.t_8):
      # 8 Bottom-left rounding radius
      psi_ref = 3*math.pi/2 + self.omega_corner * (self.t_adjusted - self.t_7)
      psi_ref_dot = self.omega_corner
      psi_ref_ddot = 0
        
    else:
      psi_ref = 0
      psi_ref_dot = 0
      psi_ref_ddot = 0
    
    return (psi_ref, psi_ref_dot, psi_ref_ddot)
  
  def addVisualization(self, mfloor, mfloor_Yposition):
    "Add circular trajectory visualization to the Chrono body (mfloor)"
    # Create a ChLinePath geometry, and insert sub-paths # ROUNDED RECTANGLE TRAJECTORY
    mpath = chrono.ChLinePath()

    seg1 = chrono.ChLineSegment(
      chrono.ChVectorD(0, abs(self.altitude_trajectory) + mfloor_Yposition, 0),
      chrono.ChVectorD(self.length_horizontal, abs(self.altitude_trajectory) + mfloor_Yposition, 0)
    )
    arc2 = chrono.ChLineArc(
      chrono.ChCoordsysD(
        chrono.ChVectorD(self.length_horizontal, abs(self.altitude_trajectory) + mfloor_Yposition, self.rounding_radius),
        chrono.ChQuaternionD(0.70710678118, 0.70710678118, 0, 0)
      ),
      self.rounding_radius,
      -math.pi/2,
      0,
      True
    )
    seg3 = chrono.ChLineSegment(
      chrono.ChVectorD(self.length_horizontal + self.rounding_radius,
                       abs(self.altitude_trajectory) + mfloor_Yposition,
                       self.rounding_radius
      ),
      chrono.ChVectorD(self.length_horizontal + self.rounding_radius,
                       abs(self.altitude_trajectory) + mfloor_Yposition,
                       self.rounding_radius + self.length_vertical
      )
    )
    arc4 = chrono.ChLineArc(
      chrono.ChCoordsysD(
        chrono.ChVectorD(self.length_horizontal,
                         abs(self.altitude_trajectory) + mfloor_Yposition,
                         self.rounding_radius + self.length_vertical
        ),
        chrono.ChQuaternionD(0.70710678118,0.70710678118,0,0)
      ),
      self.rounding_radius,
      0,
      math.pi/2,
      True
    )
    seg5 = chrono.ChLineSegment(
      chrono.ChVectorD(self.length_horizontal,
                       abs(self.altitude_trajectory) + mfloor_Yposition,
                       2*self.rounding_radius + self.length_vertical
      ),
      chrono.ChVectorD(0, 
                       abs(self.altitude_trajectory) + mfloor_Yposition,
                       2*self.rounding_radius + self.length_vertical
      )
    )
    arc6 = chrono.ChLineArc(
      chrono.ChCoordsysD(
        chrono.ChVectorD(0,
                         abs(self.altitude_trajectory) + mfloor_Yposition,
                         self.rounding_radius + self.length_vertical
        ),
        chrono.ChQuaternionD(0.70710678118,0.70710678118,0,0)
      ),
      self.rounding_radius,
      math.pi/2,
      math.pi,
      True
    )
    seg7 = chrono.ChLineSegment(
      chrono.ChVectorD(-self.rounding_radius,
                       abs(self.altitude_trajectory) + mfloor_Yposition,
                       self.rounding_radius + self.length_vertical
      ),
      chrono.ChVectorD(-self.rounding_radius,
                       abs(self.altitude_trajectory) + mfloor_Yposition,
                       self.rounding_radius
      )
    )
    arc8 = chrono.ChLineArc(
      chrono.ChCoordsysD(
        chrono.ChVectorD(0, abs(self.altitude_trajectory) + mfloor_Yposition, self.rounding_radius),
        chrono.ChQuaternionD(0.70710678118,0.70710678118,0,0)
      ),
      self.rounding_radius,
      -math.pi,
      -math.pi/2,
      True
    )
    
    mpath.AddSubLine(seg1)
    mpath.AddSubLine(arc2)
    mpath.AddSubLine(seg3)
    mpath.AddSubLine(arc4)
    mpath.AddSubLine(seg5)
    mpath.AddSubLine(arc6)
    mpath.AddSubLine(seg7)
    mpath.AddSubLine(arc8)
    mpath.Set_closed(True)

    # Create a ChLineShape, a visualization asset for lines.
    # The ChLinePath is a special type of ChLine and it can be visualized.
    mpathasset = chrono.ChLineShape()
    mpathasset.SetLineGeometry(mpath)
    mpathasset.SetColor(chrono.ChColor(0,0,0))
    mfloor.AddVisualShape(mpathasset)
    