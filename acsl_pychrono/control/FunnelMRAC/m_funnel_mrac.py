import warnings
import math
import numpy as np  
from numpy import linalg as LA
# from scipy.sparse.linalg import eigsh
from scipy.sparse.linalg import eigsh, ArpackError

class M_FunnelMRAC:
  @staticmethod
  def computeXiAndSigmasFunnel(
    control_input,
    control_input_max,
    control_input_min,
    Delta_control_input_min,
    eta,
    lambda_sat
    ) -> tuple[float, float, float]:
    """
    Computes xi, sigma_ideal, sigma_nom

    Returns:
      (xi, sigma_ideal, sigma_nom)
    """
    xi = M_FunnelMRAC.computeXiFunnel(control_input, control_input_max, control_input_min, Delta_control_input_min)
    sigma_ideal = eta * xi
    sigma_nom = min(sigma_ideal, lambda_sat)

    return (xi, sigma_ideal, sigma_nom)
  
  @staticmethod
  def computeXiFunnel(
    control_input,
    control_input_max,
    control_input_min,
    Delta_control_input_min,
    ) -> float:
    """
    Computes xi 

    Returns:
      - xi
    """
    xi = (
      (control_input_max - LA.norm(control_input)) / 
      max(LA.norm(control_input) - control_input_min, Delta_control_input_min)
    )

    return xi
  
  @staticmethod
  def computeEtaDotFunnel(
    e,
    e_dot,
    e_norm,
    eta,
    H_function,
    sigma_nom,
    M,
    e_min,
    eta_max,
    delta_1,
    delta_2,
    delta_3,
    print_flag: bool = False
    ) -> float:
    """
    Computation of eta_dot for funnel logic.

    Returns:
    - eta_dot: the time derivative of η
    """

    eta_lower_bound = math.sqrt(delta_1)
    eta_upper_bound = math.sqrt(eta_max - delta_3)
    # Different cases eta_dot
    if (
      e_norm > e_min and
      H_function <= delta_2 and
      eta > eta_lower_bound and eta < eta_upper_bound
    ):
      arg = - (e.T * M * e_dot) / eta
      eta_dot = min(arg, sigma_nom)
      if print_flag:
        print("CASE 1")
      
    elif (
      e_norm > e_min and
      H_function > delta_2 and
      eta > eta_lower_bound and eta < eta_upper_bound
    ):
      eta_dot = sigma_nom
      if print_flag:
        print("CASE 2")

    elif (
      e_norm > e_min and
      eta <= eta_lower_bound
    ):
      eta_dot = max(0.0, sigma_nom)
      if print_flag:
        print("CASE 3")

    elif (
      e_norm > e_min and
      eta >= eta_upper_bound
    ):
      eta_dot = min(0.0, sigma_nom)
      if print_flag:
        print("CASE 4")

    else: # elif (e_norm <= e_min):
      eta_dot = 0.0
      if print_flag:
        print("CASE 5")

    return eta_dot
  
  @staticmethod
  def computeVeFunnel(
    e,
    P,
    H_function
    ) -> float:
    """
    Compute Ve_function = (e^T * P * e) / H_function
    """
    if H_function <= 0:
      # raise ValueError("H_function must be positive to compute Ve_function.")
      # warnings.warn("H_function must be positive to compute Ve_function.")
      print("H_function must be positive to compute Ve_function.")
      print("H_function: ", H_function)

    Ve_function = (e.T * P * e) / H_function
    return Ve_function
  
  @staticmethod
  def computeHfunctionFunnel(
    eta_max,
    eta_funnel,
    e,
    M
    ) -> tuple[float, float, float]:
    """
    Compute H_function and cache the value of (e.T * M * e).

    Returns:
      (H_function, eT_M_e, funnel_diameter)
    """

    eT_M_e = e.T * M * e
    funnel_diameter = eta_max - eta_funnel**2
    H_function = funnel_diameter - eT_M_e
    return H_function, eT_M_e, funnel_diameter
  
  @staticmethod
  def computeLambdaSatFunnelFromMatrixEigenvalue(
    Q,
    Ve_function,
    Q_M,
    xi_bar,
    e_norm,
    P,
    M,
    H_function,
    nu,
    lambda_max_P,
    eta_funnel
    ) -> float:
    """
    Compute the lambda saturation value using the less conservative method:

    \\lambda_{\\rm sat}(t, e) &= \\max \\left( 0, \\frac{H(t,e) [\\lambda_{\\rm min}(Q + V_e(t,e)Q_{\\rm M}
    - \\frac{2\\overline{\\xi}_{\\rm d}}{\\|e\\|} (P + V_e(t,e)M)) - \\nu]}{2 \\lambda_{\\rm max}(P) \\eta(t)} \\right)
    """
    Ve_function = Ve_function.item()
    H_function = H_function.item()
    matrix_saturation = (
      Q + Ve_function * Q_M - ((2 * xi_bar) / e_norm) * (P + Ve_function * M)
    )
    if np.isnan(matrix_saturation).any() or np.isinf(matrix_saturation).any():
      return 0.0

    eigval_min, _ = eigsh(matrix_saturation, k=1, which='SA') # SA = smallest algebraic
    lambda_min_matrix_saturation = eigval_min[0]

    lambda_sat = max(
      0.0,
      H_function * (lambda_min_matrix_saturation - nu) / (2 * lambda_max_P * eta_funnel)
    )
    return lambda_sat
  
  @staticmethod
  def computeLambdaSatFunnelConservative(
    H_function,
    lambda_min_Q,
    Ve_function,
    lambda_min_Q_M,
    nu,
    xi_bar,
    e_norm,
    lambda_max_P,
    lambda_max_M,
    eta_funnel
    ) -> float:
    """
    Compute the lambda saturation value using the more conservative method:

    \\lambda_{\\rm sat}(t, e) &= \\frac{H(t,e)\\Big(\\lambda_{\\rm min}(Q) + V_e(t,e)\\lambda_{\\rm min}(Q_{\\rm M})
    -\\nu - \\frac{2\\overline{\\xi}_{\\rm d}}{\\|e\\|} 
    \\big( \\lambda_{\\rm max}(P) + V_e(t,e)\\lambda_{\\rm max}(M) \\big) \\Big)}{2 \\lambda_{\\rm max}(P) \\eta(t)}
    """
    numerator = (
      H_function * (
        (lambda_min_Q + Ve_function * lambda_min_Q_M) - nu 
        - ((2 * xi_bar) / e_norm) * (lambda_max_P + Ve_function * lambda_max_M)
      )
    )

    denominator = 2 * lambda_max_P * eta_funnel

    lambda_sat = numerator / denominator

    return lambda_sat
  
  @staticmethod
  def compute_eTransposePBFunnel(
    e,
    Ve_function,
    H_function,
    P,
    M,
    B
    ) -> tuple[np.ndarray, float]:
    eTranspose_P_B_funnel = (e.T * (P + M * Ve_function.item()) * B) / H_function
    eTranspose_P_B_funnel_norm = float(np.linalg.norm(eTranspose_P_B_funnel))
    return eTranspose_P_B_funnel, eTranspose_P_B_funnel_norm
  
  @staticmethod
  def computeXiAndEtaDotFunnelOldRoMoCoMethod(
    control_input,
    control_input_max,
    control_input_min,
    Delta_control_input_min,
    e: np.ndarray,
    Q: np.ndarray,
    Ve_function: float,
    eta_funnel: float,
    nu_funnel: float,
    H_function: float
  ) -> tuple[float, float]:
    """
    Compute xi with old RoMoCo on/off funnel method

    Returns:
      xi
    """
    xi_temp = M_FunnelMRAC.computeXiFunnel(control_input, control_input_max, control_input_min, Delta_control_input_min)

    eT_Q_e = float(e.T @ Q @ e)
    threshold = 2.0 * Ve_function * (eta_funnel ** 2) * xi_temp + nu_funnel

    condition_active = ((eT_Q_e >= threshold) and (H_function > 0.0))

    if condition_active:
      xi = xi_temp
    else:
      xi = 0.0

    eta_dot_funnel = eta_funnel * xi

    return (xi, eta_dot_funnel)
