import math
import numpy as np
from numpy import linalg as LA

from acsl_pychrono.ode_input import OdeInput
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.control.control import Control

class BaseMRAC():  
  def computeTrajectoryTrackingErrors(self, odein: OdeInput):
    """
    Computes translational and rotational tracking errors and extracts the reference position.
    Assumes self.odein and self.x_ref_tran / self.omega_ref are already set.
    """
    # State vector for translation: position + velocity
    self.x_tran = np.append(
      odein.translational_position_in_I,
      odein.translational_velocity_in_I,
      axis=0
    )
    
    # Compute trajectory tracking errors
    self.e_tran = self.x_tran - self.x_ref_tran
    self.e_rot = odein.angular_velocity - self.omega_ref
    
    # Extract reference position
    self.translational_position_in_I_ref = self.x_ref_tran[0:3]

  def computeReferenceCommandInputOuterLoop(self):
    r_tran = self.gains.mass_total_estimated * (
      - self.gains.KI_tran * self.integral_position_tracking_ref
      + self.odein.translational_acceleration_in_I_user
      + self.gains.KP_tran * self.odein.translational_position_in_I_user
      + self.gains.KD_tran * self.odein.translational_velocity_in_I_user
    )

    return r_tran
  
  def computeReferenceModelOuterLoop(self):
    x_ref_tran_dot = self.gains.A_ref_tran * self.x_ref_tran + self.gains.B_ref_tran * self.r_tran

    return x_ref_tran_dot
  
  def computeMuPDbaselineOuterLoop(self):
    mu_PD_baseline_tran = -self.gains.mass_total_estimated * (
      self.gains.KP_tran_PD_baseline * (self.odein.translational_position_in_I - self.translational_position_in_I_ref)
      + self.gains.KD_tran_PD_baseline * (self.odein.translational_velocity_in_I - self.x_ref_tran[3:6])
      - self.x_ref_tran_dot[3:6]
    )

    return mu_PD_baseline_tran
  
  def computeRegressorVectorAndThetaBarOuterLoop(self):
    # Compute rotation matrices
    (R_from_loc_to_glob,
     R_from_glob_to_loc
    ) = Control.computeRotationMatrices(self.odein.roll, self.odein.pitch, self.odein.yaw)
    
    translational_velocity_in_J = R_from_glob_to_loc * self.odein.translational_velocity_in_I
    translational_velocity_in_J_norm = LA.norm(R_from_glob_to_loc * self.odein.translational_velocity_in_I)
    self.Phi_adaptive_tran = -0.5 * translational_velocity_in_J * translational_velocity_in_J_norm 

    Phi_adaptive_tran_augmented = np.matrix(np.block([[self.mu_PD_baseline_tran],
                                                      [self.Phi_adaptive_tran]]))
    Theta_tran_adaptive_bar_augmented = np.matrix(np.block([[np.identity(3)],
                                                            [self.gains.Theta_tran_adaptive_bar]]))
    
    return Phi_adaptive_tran_augmented, Theta_tran_adaptive_bar_augmented
  
  def computeMuBaselineBarOuterLoop(self):
    mu_baseline_tran = (
      self.gains.K_x_tran_bar.T * self.x_tran
      + self.gains.K_r_tran_bar.T * self.r_tran
      - self.Theta_tran_adaptive_bar_augmented.T * self.Phi_adaptive_tran_augmented
    )

    return mu_baseline_tran
  
  def computeMuAdaptiveOuterLoop(self):
    mu_adaptive_tran = (
      self.K_hat_x_tran.T * self.x_tran
      + self.K_hat_r_tran.T * self.r_tran
      - self.Theta_hat_tran.T * self.Phi_adaptive_tran_augmented
    )

    return mu_adaptive_tran
  
  def computeMuRawOuterLoop(self):
    mu_tran_raw = (
      self.mu_PD_baseline_tran
      + self.mu_baseline_tran
      + self.mu_adaptive_tran
    )

    return mu_tran_raw
  
  def computeOmegaCmdAndOmegaCmdDotInnerLoop(self):
    Jacobian_matrix = Control.computeJacobian(self.odein.roll, self.odein.pitch)

    Jacobian_matrix_dot = Control.computeJacobianDot(
      self.odein.roll,
      self.odein.pitch,
      self.angular_position_dot[0],
      self.angular_position_dot[1]
    )

    omega_cmd = Jacobian_matrix * (
      - self.gains.KP_rot * self.angular_error 
      - self.gains.KI_rot * self.integral_angular_error 
      + self.angular_position_ref_dot
    )
    
    omega_cmd_dot = (
      Jacobian_matrix_dot * (
        - self.gains.KP_rot * self.angular_error 
        - self.gains.KI_rot * self.integral_angular_error 
        + self.angular_position_ref_dot)
      + Jacobian_matrix * (
        - self.gains.KP_rot * self.angular_error_dot 
        - self.gains.KI_rot * self.angular_error 
        + self.angular_position_ref_ddot)
    )

    return omega_cmd, omega_cmd_dot
  
  def computeReferenceModelInnerLoop(self):
    omega_ref_dot = (
      - self.gains.K_P_omega_ref * (self.omega_ref - self.omega_cmd) 
      - self.gains.K_I_omega_ref * self.integral_e_omega_ref_cmd
      + self.omega_cmd_dot
    )

    return omega_ref_dot
  
  def computeReferenceCommandInputInnerLoop(self):
    r_rot = (
      self.gains.K_P_omega_ref * self.omega_cmd
      - self.gains.K_I_omega_ref * self.integral_e_omega_ref_cmd
      + self.omega_cmd_dot
    )

    return r_rot
  
  def computeMomentPIbaselineInnerLoop(self):
    Moment_baseline_PI = -self.fp.I_matrix_estimated * (
      self.gains.KP_rot_PI_baseline * self.e_rot
      + self.gains.KI_rot_PI_baseline * self.integral_e_rot 
      - self.omega_ref_dot
    )

    return Moment_baseline_PI
  
  def computeRegressorVectorInnerLoop(self):
    Phi_adaptive_rot = np.array([[self.odein.angular_velocity[1].item() * self.odein.angular_velocity[2].item()],
                                 [self.odein.angular_velocity[0].item() * self.odein.angular_velocity[2].item()],
                                 [self.odein.angular_velocity[0].item() * self.odein.angular_velocity[1].item()]])
    
    Phi_adaptive_rot_augmented = np.matrix(np.block([[self.Moment_baseline_PI],
                                                     [Phi_adaptive_rot]]))
    
    return Phi_adaptive_rot, Phi_adaptive_rot_augmented
  
  def compute_eTransposePB_InnerLoop(self):
    eTranspose_P_B_rot = self.e_rot.T * self.gains.P_rot * self.gains.B_rot

    return eTranspose_P_B_rot
  
  def computeMomentBaselineInnerLoop(self):
    Moment_baseline = np.cross(
      self.odein.angular_velocity.ravel(),
      (self.fp.I_matrix_estimated * self.odein.angular_velocity).ravel()
    ).reshape(3,1)

    return Moment_baseline
  
  def computeMomentAdaptiveInnerLoop(self):
    Moment_adaptive = (
      self.K_hat_x_rot.T * self.odein.angular_velocity
      + self.K_hat_r_rot.T * self.r_rot
      - self.Theta_hat_rot.T * self.Phi_adaptive_rot_augmented
    )

    return Moment_adaptive
  
  def computeU2_U3_U4(self):
    Moment = self.Moment_baseline_PI + self.Moment_baseline + self.Moment_adaptive
    
    u2 = Moment[0].item()
    u3 = Moment[1].item()
    u4 = Moment[2].item()

    return u2, u3, u4, Moment
