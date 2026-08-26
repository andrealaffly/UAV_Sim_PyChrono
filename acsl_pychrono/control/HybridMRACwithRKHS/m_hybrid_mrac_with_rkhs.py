import math
import numpy as np  

class M_HybridMRACwithRKHS:
  """
  
  """

  @staticmethod
  def computePreviousTrajectoryTrackingErrors(e_tran: np.ndarray, e_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    e_tran_previous = e_tran
    e_rot_previous = e_rot
    return (e_tran_previous, e_rot_previous)
  
  @staticmethod
  def evaluateSeriesElement(s: float, alpha: float) -> float:
    """
    Parameters
    ----------
    s : s-th term in the series used to compute reference resetting events

    Returns
    -------
    element : output of the series
    """
    element = 1/(s**alpha)
    return element
  
  @staticmethod
  def findSseries(s_previous: float, weighted_e_squared: float, alpha: float) -> int:
    """"
    This function allows to compute s such that \\sum s is convergent;
    see the comments after (40) in the paper.
    
    weighted_e_squared = e^T(t_j) P e(t_j)
    """   
    s = max(math.ceil(weighted_e_squared**(-1/alpha)), s_previous + 1)
    return s
  
  @staticmethod
  def computeQuadraticForm(vector: np.ndarray, matrix: np.ndarray) -> float:
    return (np.matmul(vector.T, np.matmul(matrix, vector))).item()
  
  @staticmethod
  def updateSummationHybridP(ePe: float, ePe_previous: float, summation: float) -> float:
    if ePe > ePe_previous:
      delta_ePe = ePe - ePe_previous
      summation += delta_ePe
    return summation
  
  @staticmethod
  def resetSeriesIfNeeded(
    s_hybrid: int,
    time_now: float,
    time_of_last_trajectory_reset: float,
    tolerance_time_reset_series: float
    ) -> int:
    time_since_last_trajectory_reset = time_now - time_of_last_trajectory_reset
    if s_hybrid > 1e9 or (s_hybrid > 0 and time_since_last_trajectory_reset > tolerance_time_reset_series):
      # If enough time passed since the last resetting event, then reset the first element of the series
      s_hybrid = 0
    return s_hybrid
  
  @staticmethod
  def checkAndTriggerTrajectoryReset(
    integral_eQe: float,
    summation_hybrid_P: float,
    time_now: float,
    time_of_last_trajectory_reset: float,
    ePe: float,
    alpha: float,
    s_hybrid: int,
    e: np.ndarray,
    e_previous: np.ndarray,
    state_reference_model: np.ndarray,
    Q: np.ndarray,
    yout_storage: np.ndarray,
    yout_indices_reference_model: slice,
    yout_index_integral_eQe: int
    ) -> tuple[int, float]:
    """
    Perform trajectory reset logic if condition is met
    """
    if integral_eQe >= summation_hybrid_P:
      # Update the time of the event of the reference trajectory reset
      time_of_last_trajectory_reset = time_now
      # Update 's' variable of the series
      s_hybrid = M_HybridMRACwithRKHS.findSseries(s_hybrid, ePe, alpha)
      # Update reference trajectory
      delta_series = ePe - M_HybridMRACwithRKHS.evaluateSeriesElement(s_hybrid, alpha)
      jump_factor = 1 - math.sqrt(delta_series / ePe)
      jump_reference_trajectory = jump_factor * e
      yout_storage[yout_indices_reference_model] = state_reference_model + jump_reference_trajectory
      # Update integral term related to (e^T * Q * e)
      delta_integral_eQe = (e.T * Q * e - e_previous.T * Q * e_previous)
      yout_storage[yout_index_integral_eQe] = integral_eQe + delta_integral_eQe

    return (s_hybrid, time_of_last_trajectory_reset)
  
  @staticmethod
  def runHybridStep(
    e: np.ndarray,
    e_previous: np.ndarray,
    P: np.ndarray,
    Q: np.ndarray,
    summation_hybrid_P: float,
    s_hybrid: int,
    time_of_last_trajectory_reset: float,
    integral_eQe: float,
    state_reference_model: np.ndarray,
    alpha_hybrid_series: float,
    time_now: float,
    tolerance_time_reset_series_hybrid: float,
    yout_storage: np.ndarray,
    yout_indices_reference_model: slice,
    yout_index_integral_eQe: int,
    first_controller_loop: bool
    )-> tuple[bool, float, int, float]:

    ePe = M_HybridMRACwithRKHS.computeQuadraticForm(e, P)
    ePe_previous = M_HybridMRACwithRKHS.computeQuadraticForm(e_previous, P)

    summation_hybrid_P = M_HybridMRACwithRKHS.updateSummationHybridP(
      ePe, ePe_previous, summation_hybrid_P
    )

    if not first_controller_loop:
      s_hybrid = M_HybridMRACwithRKHS.resetSeriesIfNeeded(
        s_hybrid,
        time_now,
        time_of_last_trajectory_reset,
        tolerance_time_reset_series_hybrid
      )

      (s_hybrid, 
       time_of_last_trajectory_reset
      ) = M_HybridMRACwithRKHS.checkAndTriggerTrajectoryReset(
        integral_eQe,
        summation_hybrid_P,
        time_now,
        time_of_last_trajectory_reset,
        ePe,
        alpha_hybrid_series,
        s_hybrid,
        e,
        e_previous,
        state_reference_model,
        Q,
        yout_storage,
        yout_indices_reference_model,
        yout_index_integral_eQe
      )
    
    first_controller_loop = False

    return (first_controller_loop, summation_hybrid_P, s_hybrid, time_of_last_trajectory_reset)
