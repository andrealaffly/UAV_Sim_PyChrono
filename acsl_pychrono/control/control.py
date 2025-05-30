import math
import numpy as np
from numpy.typing import NDArray
from abc import ABC, abstractmethod

from acsl_pychrono.functions import rk4singlestep
from acsl_pychrono.ode_input import OdeInput
from acsl_pychrono.flight_params import FlightParams

class Control:
  def __init__(self, odein: OdeInput) -> None:
    self.odein = odein
    self.y = None # Will be initialized in subclasses
    self.timestep = None # Will be initialized in subclasses

  @abstractmethod
  def computeControlAlgorithm(self, ode_input: OdeInput) -> None:
    """
    Each subclass must implement this method to define the control algorithm.
    """
    pass
    
  @abstractmethod
  def ode(self, t, y) -> NDArray[np.float64]:
    """
    Each subclass must implement this method to define the system of equations to be integrated.
    """
    pass

  def integrateODEOneStepRK4(self):
    """
    Perform a single RK4 integration step using the current state.
    Updates self.y in place.
    """
    self.y = rk4singlestep(self.ode, self.timestep, self.odein.time_now, self.y)

  def run(self, ode_input: OdeInput) -> None:
    """
    Perform one full control loop step:
    1. Update the control algorithm.
    2. Integrate the system forward in time.
    """
    self.computeControlAlgorithm(ode_input)
    self.integrateODEOneStepRK4()

  @staticmethod
  def computeU1RollPitchRef(mu_x, mu_y, mu_z, mass_total_estimated, G_acc, yaw_ref):

    u1 = math.sqrt(mu_x ** 2 + mu_y ** 2 + (mass_total_estimated * G_acc - mu_z) ** 2)
    
    calculation_var_A = -(1/u1) * (mu_x * math.sin(yaw_ref) - mu_y * math.cos(yaw_ref))
    roll_ref = math.atan2(calculation_var_A, math.sqrt(1 - calculation_var_A ** 2))
    
    pitch_ref = math.atan2(-(mu_x * math.cos(yaw_ref) + mu_y * math.sin(yaw_ref)),
                            (mass_total_estimated * G_acc - mu_z))
    
    return u1, roll_ref, pitch_ref
  
  @staticmethod
  def computeTranslationalPositionError(position, desired_position):
    translational_position_error = position - desired_position
    return translational_position_error
  
  @staticmethod
  def computeAngularReferenceSignals(
      fp: FlightParams,
      odein: OdeInput,
      roll_ref,
      pitch_ref,
      state_phi_ref_diff,
      state_theta_ref_diff
    ):

    internal_state_differentiator_phi_ref_diff = fp.A_phi_ref * state_phi_ref_diff + fp.B_phi_ref*roll_ref
    internal_state_differentiator_theta_ref_diff = fp.A_theta_ref * state_theta_ref_diff + fp.B_theta_ref*pitch_ref
    
    roll_ref_dot = np.asarray(fp.C_phi_ref*state_phi_ref_diff).item()
    pitch_ref_dot = np.asarray(fp.C_theta_ref*state_theta_ref_diff).item()
    
    roll_ref_ddot = np.asarray(fp.C_phi_ref*internal_state_differentiator_phi_ref_diff).item()
    pitch_ref_ddot = np.asarray(fp.C_theta_ref*internal_state_differentiator_theta_ref_diff).item()
    
    angular_position_ref_dot = np.array([roll_ref_dot, pitch_ref_dot, odein.yaw_ref_dot]).reshape(3,1)
    angular_position_ref_ddot = np.array([roll_ref_ddot, pitch_ref_ddot, odein.yaw_ref_ddot]).reshape(3,1)

    return (
      internal_state_differentiator_phi_ref_diff,
      internal_state_differentiator_theta_ref_diff,
      angular_position_ref_dot,
      angular_position_ref_ddot
    )
  
  @staticmethod
  def computeJacobianInverse(roll: float, pitch: float) -> np.matrix:
    """
    Compute the inverse of the Jacobian matrix (ZYX Euler angles) given roll and pitch.
    
    Args:
        roll (float): Roll angle in radians.
        pitch (float): Pitch angle in radians.

    Returns:
        np.matrix: 3x3 inverse Jacobian matrix.
    """
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-6:
        raise ValueError("Pitch angle too close to ±90°, Jacobian is singular.")

    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    sin_pitch = math.sin(pitch)

    J_inv = np.matrix([
        [1, (sin_roll * sin_pitch) / cos_pitch, (cos_roll * sin_pitch) / cos_pitch],
        [0,                           cos_roll,                          -sin_roll],
        [0,               sin_roll / cos_pitch,               cos_roll / cos_pitch]
    ])

    return J_inv

  @staticmethod
  def computeAngularError(roll, pitch, yaw, roll_ref, pitch_ref, yaw_ref) -> np.ndarray:
    """
    Compute the angular error between current and reference Euler angles.

    Args:
      roll (float): Current roll angle (rad)
      pitch (float): Current pitch angle (rad)
      yaw (float): Current yaw angle (rad)
      roll_ref (float): Desired roll angle (rad)
      pitch_ref (float): Desired pitch angle (rad)
      yaw_ref (float): Desired yaw angle (rad)

    Returns:
      np.ndarray: 3x1 angular error vector
    """
    yaw_error = ((yaw - yaw_ref + math.pi) % (2 * math.pi)) - math.pi

    angular_error = np.array([
      roll - roll_ref,
      pitch - pitch_ref,
      yaw_error
    ]).reshape(3, 1)

    return angular_error
  
  @staticmethod
  def computeAngularErrorAndDerivative(
    odein: OdeInput,
    roll_ref,
    pitch_ref,
    angular_position_ref_dot
    ):

    angular_error = Control.computeAngularError(odein.roll, odein.pitch, odein.yaw,
                                                roll_ref, pitch_ref, odein.yaw_ref)

    Jacobian_matrix_inverse = Control.computeJacobianInverse(odein.roll, odein.pitch)

    angular_position_dot = Jacobian_matrix_inverse * odein.angular_velocity
    angular_error_dot = angular_position_dot - angular_position_ref_dot

    return angular_error, angular_position_dot, angular_error_dot
  
  @staticmethod
  def computeMotorThrusts(fp: FlightParams, u1, u2, u3, u4):
    """
    Compute motor thrusts 'motor_thrusts' from the control inputs (u1, u2, u3, u4) using the allocation matrix inverse.
    Returns:
      T (8x1 np.array): thrusts for 8 motors
    """
    U = np.array([u1, u2, u3, u4])
    motor_thrusts = np.matmul(fp.U_mat_inv, U).reshape(8,1) # array of thrust of each motor (T1, T2, T3, T4, T5, T6, T7, T8)
    return motor_thrusts
  
  @staticmethod
  def computeRotationMatrices(roll, pitch, yaw):
    """
    Compute rotation matrices from local to global and global to local frames.
    """
    R3 = np.matrix([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw),  math.cos(yaw), 0],
                    [            0,              0, 1]])
    
    R2 = np.matrix([[ math.cos(pitch), 0, math.sin(pitch)],
                    [               0, 1,               0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])
    
    R1 = np.matrix([[1,              0,               0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll),  math.cos(roll)]])
    
    R_from_loc_to_glob = R3 * R2 * R1
    R_from_glob_to_loc = R_from_loc_to_glob.transpose()

    return R_from_loc_to_glob, R_from_glob_to_loc
  
  @staticmethod
  def computeJacobianDot(roll: float, pitch: float, roll_dot: float, pitch_dot: float) -> np.matrix:
    """
    Compute the time derivative of the Jacobian matrix (ZYX Euler angles)
    based on current roll and pitch angles and their time derivatives.

    Args:
        roll (float): Roll angle in radians.
        pitch (float): Pitch angle in radians.
        roll_dot (float): Time derivative of roll angle [rad/s].
        pitch_dot (float): Time derivative of pitch angle [rad/s].

    Returns:
        np.matrix: 3x3 time derivative of the Jacobian matrix.
    """
    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)

    J_dot = np.matrix(np.zeros((3, 3)))
    J_dot[0, 2] = -cos_pitch * pitch_dot
    J_dot[1, 1] = -sin_roll * roll_dot
    J_dot[1, 2] = cos_roll * cos_pitch * roll_dot - sin_roll * sin_pitch * pitch_dot
    J_dot[2, 1] = -cos_roll * roll_dot
    J_dot[2, 2] = -cos_pitch * sin_roll * roll_dot - cos_roll * sin_pitch * pitch_dot

    return J_dot
  
  @staticmethod
  def computeJacobian(roll: float, pitch: float) -> np.matrix:
    """
    Compute the Jacobian matrix for ZYX Euler angles given roll and pitch.

    Args:
        roll (float): Roll angle in radians.
        pitch (float): Pitch angle in radians.

    Returns:
        np.matrix: 3x3 Jacobian matrix.
    """
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)
    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)

    J = np.matrix([
        [1,          0,             -sin_pitch],
        [0,   cos_roll, sin_roll * cos_pitch],
        [0,  -sin_roll, cos_roll * cos_pitch]
    ])

    return J

