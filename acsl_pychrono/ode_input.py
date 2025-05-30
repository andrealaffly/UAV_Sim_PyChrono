import math
import numpy as np

from acsl_pychrono.simulation.pixhawk_state import VehicleState
from acsl_pychrono.user_defined_trajectory.base_user_defined_trajectory import UserDefinedTrajectoryState

class OdeInput:
  def __init__(self):
    self.time_now = 0.0 # Current time
    # Vehicle state
    self.translational_position_in_I = np.zeros((3, 1))
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    self.translational_velocity_in_I = np.zeros((3, 1))
    self.angular_velocity = np.zeros((3, 1))

    # User-defined trajectory
    self.translational_position_in_I_user = np.zeros((3, 1))
    self.translational_velocity_in_I_user = np.zeros((3, 1))
    self.translational_acceleration_in_I_user = np.zeros((3, 1))
    self.yaw_ref = 0.0
    self.yaw_ref_dot = 0.0
    self.yaw_ref_ddot = 0.0

  def update(self, 
    time_now: float,
    vehicle_state: VehicleState,
    user_defined_trajectory_state: UserDefinedTrajectoryState
    ) -> None:

    # Current time
    self.time_now = time_now
    
    # Vehicle state
    self.translational_position_in_I = vehicle_state.position_global
    self.roll = vehicle_state.roll
    self.pitch = vehicle_state.pitch
    self.yaw = vehicle_state.yaw
    self.translational_velocity_in_I = vehicle_state.velocity_global
    self.angular_velocity = vehicle_state.angular_velocity_local

    # User-defined trajectory
    self.translational_position_in_I_user = user_defined_trajectory_state.position
    self.translational_velocity_in_I_user = user_defined_trajectory_state.velocity
    self.translational_acceleration_in_I_user = user_defined_trajectory_state.acceleration
    self.yaw_ref = user_defined_trajectory_state.yaw
    self.yaw_ref_dot = user_defined_trajectory_state.yaw_dot
    self.yaw_ref_ddot = user_defined_trajectory_state.yaw_dot_dot
