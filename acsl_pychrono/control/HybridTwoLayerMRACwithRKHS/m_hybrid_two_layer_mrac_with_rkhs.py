import math
import numpy as np  
from acsl_pychrono.control.MRAC.m_mrac import M_MRAC

class M_HybridTwoLayerMRACwithRKHS:
  """
  
  """

  @staticmethod
  def computeControlLaw(
    K_hat_x: np.ndarray,
    x: np.ndarray,
    K_hat_r: np.ndarray,
    r: np.ndarray,
    Theta_hat: np.ndarray,
    Phi: np.ndarray,
    K_hat_g: np.ndarray,
    e: np.ndarray
    ) -> np.ndarray:
    control_input = (
      K_hat_x.T * x
      + K_hat_r.T * r
      - Theta_hat.T * Phi
      + K_hat_g.T * e
    )

    return control_input
  
  @staticmethod
  def computeAllRobustAdaptiveLaws(
    Gamma_x: np.ndarray,
    x: np.ndarray,
    Gamma_r: np.ndarray,
    r: np.ndarray,
    Gamma_Theta: np.ndarray,
    Phi_regressor_vector: np.ndarray,
    Gamma_g: np.ndarray,
    e: np.ndarray,
    eTranspose_P_B: np.ndarray,
    dead_zone_value: float,
    sigma_x: float,
    sigma_r: float,
    sigma_Theta: float,
    sigma_g: float,
    eTranspose_P_B_norm: float,
    K_hat_x: np.ndarray,
    K_hat_r: np.ndarray,
    Theta_hat: np.ndarray,
    K_hat_g: np.ndarray,
    use_deadzone: bool,
    use_emodification: bool
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    K_hat_x_dot = M_MRAC.computeRobustAdaptiveLaw(
      -Gamma_x,
      dead_zone_value,
      x,
      eTranspose_P_B,
      sigma_x,
      eTranspose_P_B_norm,
      K_hat_x,
      use_deadzone,
      use_emodification
    )

    K_hat_r_dot = M_MRAC.computeRobustAdaptiveLaw(
      -Gamma_r,
      dead_zone_value,
      r,
      eTranspose_P_B,
      sigma_r,
      eTranspose_P_B_norm,
      K_hat_r,
      use_deadzone,
      use_emodification
    )

    Theta_hat_dot = M_MRAC.computeRobustAdaptiveLaw(
      Gamma_Theta,
      dead_zone_value,
      Phi_regressor_vector,
      eTranspose_P_B,
      sigma_Theta,
      eTranspose_P_B_norm,
      Theta_hat,
      use_deadzone,
      use_emodification
    )

    K_hat_g_dot = M_MRAC.computeRobustAdaptiveLaw(
      -Gamma_g,
      dead_zone_value,
      e,
      eTranspose_P_B,
      sigma_g,
      eTranspose_P_B_norm,
      K_hat_g,
      use_deadzone,
      use_emodification
    )

    return (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot, K_hat_g_dot)
