from abc import ABC
import numpy as np

class BaseVehicleParamsGains(ABC):
  I_matrix_estimated: np.ndarray
  mass_total_estimated: float
  air_density_estimated: float
  surface_area_estimated: float
  drag_coefficient_matrix_estimated: np.ndarray

class BaseGeneralDimensionsGains(ABC):
  number_of_states: int
  size_DATA: int

class BaseBaselineGains(ABC):
  KP_tran: np.ndarray
  KD_tran: np.ndarray
  KI_tran: np.ndarray
  KP_tran_PD_baseline: np.ndarray
  KD_tran_PD_baseline: np.ndarray
  KP_rot: np.ndarray
  KI_rot: np.ndarray
  KP_rot_PI_baseline: np.ndarray
  KI_rot_PI_baseline: np.ndarray
  K_P_omega_ref: np.ndarray
  K_I_omega_ref: np.ndarray

class BaseMRACTranslationalGains(ABC):
  A_tran: np.ndarray
  B_tran: np.ndarray
  A_ref_tran: np.ndarray
  B_ref_tran: np.ndarray
  Gamma_x_tran: np.ndarray
  Gamma_r_tran: np.ndarray
  Gamma_Theta_tran: np.ndarray
  Q_tran: np.ndarray
  P_tran: np.ndarray

class BaseMRACRotationalGains(ABC):
  A_rot: np.ndarray
  B_rot: np.ndarray
  A_ref_rot: np.ndarray
  B_ref_rot: np.ndarray
  Q_rot: np.ndarray
  P_rot: np.ndarray
  Gamma_x_rot: np.ndarray
  Gamma_r_rot: np.ndarray
  Gamma_Theta_rot: np.ndarray

class BaseTwoLayerGains(ABC):
  A_transient_tran: np.ndarray
  Gamma_g_tran: np.ndarray
  sigma_g_tran: float
  A_transient_rot: np.ndarray
  Gamma_g_rot: np.ndarray
  sigma_g_rot: float

class BaseSafetyMechanismGains(ABC):
  use_safety_mechanism: bool
  sphereEpsilon: float
  maximumThrust: float
  EllipticConeEpsilon: float
  maximumRollAngle: float
  maximumPitchAngle: float
  planeEpsilon: float
  alphaPlane: float

class BaseDeadZoneModificationGains(ABC):
  use_dead_zone_modification: bool
  dead_zone_delta_tran: float
  dead_zone_e0_tran: float
  dead_zone_delta_rot: float
  dead_zone_e0_rot: float

class BaseEModificationGains(ABC):
  use_e_modification: bool
  sigma_x_tran: float
  sigma_r_tran: float
  sigma_Theta_tran: float
  sigma_x_rot: float
  sigma_r_rot: float
  sigma_Theta_rot: float

class BaseProjectionOperatorGains(ABC):
  use_projection_operator: bool

  x_e_x_tran: np.ndarray
  S_diagonal_x_tran: np.ndarray
  alpha_x_tran: float

  x_e_r_tran: np.ndarray
  S_diagonal_r_tran: np.ndarray
  alpha_r_tran: float

  x_e_Theta_tran: np.ndarray
  S_diagonal_Theta_tran: np.ndarray
  alpha_Theta_tran: float

  x_e_g_tran: np.ndarray
  S_diagonal_g_tran: np.ndarray
  alpha_g_tran: float

  x_e_x_rot: np.ndarray
  S_diagonal_x_rot: np.ndarray
  alpha_x_rot: float

  x_e_r_rot: np.ndarray
  S_diagonal_r_rot: np.ndarray
  alpha_r_rot: float

  x_e_Theta_rot: np.ndarray
  S_diagonal_Theta_rot: np.ndarray
  alpha_Theta_rot: float

  x_e_g_rot: np.ndarray
  S_diagonal_g_rot: np.ndarray
  alpha_g_rot: float

  S_x_tran: np.ndarray
  S_r_tran: np.ndarray
  S_Theta_tran: np.ndarray
  S_g_tran: np.ndarray
  S_x_rot: np.ndarray
  S_r_rot: np.ndarray
  S_Theta_rot: np.ndarray
  S_g_rot: np.ndarray

  epsilon_x_tran: float
  epsilon_r_tran: float
  epsilon_Theta_tran: float
  epsilon_g_tran: float
  epsilon_x_rot: float
  epsilon_r_rot: float
  epsilon_Theta_rot: float
  epsilon_g_rot: float

class BaseErrorBoundingControlInputGains(ABC):
  use_error_bounding_control_input: bool
  xi_bar_d_tran: float
  lambda_bar_tran: float
  delta_ebci_tran: float
  xi_bar_d_rot: float
  lambda_bar_rot: float
  delta_ebci_rot: float

class BaseHybridGains(ABC):
  use_hybrid: bool
  alpha_hybrid_series_tran: float
  tolerance_time_reset_series_hybrid_tran: float
  alpha_hybrid_series_rot: float
  tolerance_time_reset_series_hybrid_rot: float
  lambda_bar_rot: float
  delta_ebci_rot: float

class BaseFunnelGains(ABC):
  Q_M_funnel_tran: np.ndarray
  M_funnel_tran: np.ndarray
  xi_bar_d_funnel_tran: float
  lambda_max_M_funnel_tran: float
  lambda_min_Q_M_funnel_tran: float
  e_min_funnel_tran: float
  eta_max_funnel_tran: float
  delta_1_funnel_tran: float
  delta_2_funnel_tran: float
  delta_3_funnel_tran: float
  lambda_max_P_tran: float
  lambda_min_Q_tran: float
  initial_cond_diameter_funnel_tran: float
  initial_cond_eta_funnel_tran: float
  u_max: float
  u_min: float
  Delta_u_min: float
  nu_funnel_tran: float
  use_funnel_romoco_old_tran: bool
  use_eigenvalue_lambda_sat_funnel_tran: bool

  Q_M_funnel_rot: np.ndarray
  M_funnel_rot: np.ndarray
  xi_bar_d_funnel_rot: float
  lambda_max_M_funnel_rot: float
  lambda_min_Q_M_funnel_rot: float
  e_min_funnel_rot: float
  eta_max_funnel_rot: float
  delta_1_funnel_rot: float
  delta_2_funnel_rot: float
  delta_3_funnel_rot: float
  lambda_max_P_rot: float
  lambda_min_Q_rot: float
  initial_cond_diameter_funnel_rot: float
  initial_cond_eta_funnel_rot: float
  Moment_max: float
  Moment_min: float
  Delta_Moment_min: float
  nu_funnel_rot: float
  use_eigenvalue_lambda_sat_funnel_rot: bool

  def __post_init__(self):
    if (
      (self.delta_3_funnel_tran <= self.delta_2_funnel_tran) or
      (self.delta_3_funnel_rot <= self.delta_2_funnel_rot)
    ):
      raise ValueError("delta_3 must be greater than delta_2")
    
    if (
      (self.eta_max_funnel_tran <= self.delta_1_funnel_tran + self.delta_3_funnel_tran) or
      (self.eta_max_funnel_rot <= self.delta_1_funnel_rot + self.delta_3_funnel_rot)
    ):
      raise ValueError("eta_max must be greater than delta_1 + delta_3")
    
    if (
      (self.delta_3_funnel_tran <= self.e_min_funnel_tran) or
      (self.delta_3_funnel_rot <= self.e_min_funnel_rot)
    ):
      raise ValueError("delta_3 must be greater than e_min")
    
    if (
      (self.nu_funnel_tran < 0) or (self.nu_funnel_rot < 0) or
      (self.nu_funnel_tran >= self.lambda_min_Q_tran) or (self.nu_funnel_rot >= self.lambda_min_Q_rot)
    ):
      raise ValueError(
        f"Invalid nu value:\n"
        f"nu_funnel_tran = {self.nu_funnel_tran}, lambda_min_Q_tran = {self.lambda_min_Q_tran};\n"
        f"nu_funnel_rot = {self.nu_funnel_rot}, lambda_min_Q_rot = {self.lambda_min_Q_rot}.\n"
        f"Each nu must be in [0, lambda_min(Q))."
      )


# ----------------------------------------------------------------------------------------------------------------------

class BaseMRACGains(
  BaseVehicleParamsGains,
  BaseGeneralDimensionsGains,
  BaseBaselineGains,
  BaseMRACTranslationalGains,
  BaseMRACRotationalGains,
  BaseTwoLayerGains,
  BaseSafetyMechanismGains,
  BaseDeadZoneModificationGains,
  BaseEModificationGains,
  BaseProjectionOperatorGains,
  BaseErrorBoundingControlInputGains,
  BaseHybridGains,
  BaseFunnelGains
):
  pass


