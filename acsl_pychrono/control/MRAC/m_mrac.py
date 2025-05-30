import math
import numpy as np  

class M_MRAC:
  @staticmethod
  def computeAdaptiveLaw(Gamma_gain, pi_vector, eTranspose_P_B):
    K_hat_state_dot = Gamma_gain * pi_vector * eTranspose_P_B

    return K_hat_state_dot
  
  def reshapeAdaptiveGainsToMatrices(self):
    """
    Reshapes all gain parameters to their correct (row, col) shape and converts them to np.matrix.
    This is intended to be called once after loading or updating gains stored as flat arrays.
    """
    self.K_hat_x_tran = np.matrix(self.K_hat_x_tran.reshape(6,3))
    self.K_hat_r_tran = np.matrix(self.K_hat_r_tran.reshape(3,3))
    self.Theta_hat_tran = np.matrix(self.Theta_hat_tran.reshape(6,3))
    self.K_hat_x_rot = np.matrix(self.K_hat_x_rot.reshape(3,3))
    self.K_hat_r_rot = np.matrix(self.K_hat_r_rot.reshape(3,3))
    self.Theta_hat_rot = np.matrix(self.Theta_hat_rot.reshape(6,3))

  def compute_eTransposePB_OuterLoop(self):
    eTranspose_P_B_tran = self.e_tran.T * self.gains.P_tran * self.gains.B_tran

    return eTranspose_P_B_tran

  def computeAllAdaptiveLawsOuterLoop(self, eTranspose_P_B_tran):
    self.K_hat_x_tran_dot = self.computeAdaptiveLaw(
      -self.gains.Gamma_x_tran,
      self.x_tran,
      eTranspose_P_B_tran
    )
    self.K_hat_r_tran_dot = self.computeAdaptiveLaw(
      -self.gains.Gamma_r_tran,
      self.r_tran,
      eTranspose_P_B_tran
    )
    self.Theta_hat_tran_dot = self.computeAdaptiveLaw(
      self.gains.Gamma_Theta_tran,
      self.Phi_adaptive_tran_augmented,
      eTranspose_P_B_tran
    )

  def computeAllAdaptiveLawsInnerLoop(self, eTranspose_P_B_rot):
    self.K_hat_x_rot_dot = self.computeAdaptiveLaw(
      -self.gains.Gamma_x_rot,
      self.odein.angular_velocity,
      eTranspose_P_B_rot
    )
    self.K_hat_r_rot_dot = self.computeAdaptiveLaw(
      -self.gains.Gamma_r_rot,
      self.r_rot,
      eTranspose_P_B_rot
    )
    self.Theta_hat_rot_dot = self.computeAdaptiveLaw(
      self.gains.Gamma_Theta_rot,
      self.Phi_adaptive_rot_augmented,
      eTranspose_P_B_rot
    )