import math
import numpy as np
from numpy import linalg as LA
from scipy import linalg
from acsl_pychrono.flight_params import FlightParams

class MRACGains:
  def __init__(self, flight_params: FlightParams):
    # General vehicle properties
    self.I_matrix_estimated = flight_params.I_matrix_estimated
    self.mass_total_estimated = flight_params.mass_total_estimated
    self.air_density_estimated = flight_params.air_density_estimated
    self.surface_area_estimated = flight_params.surface_area_estimated
    self.drag_coefficient_matrix_estimated = flight_params.drag_coefficient_matrix_estimated

    # Number of states to be integrated by RK4
    self.number_of_states = 106
    # Length of the array vector that will be exported 
    self.size_DATA = 92

    # ----------------------------------------------------------------
    #                     Baseline Parameters
    # ----------------------------------------------------------------

    # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
    self.KP_tran = np.matrix(1 * np.diag([5,5,6]))
    self.KD_tran = np.matrix(1 * np.diag([8,8,3]))
    self.KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

    # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
    self.KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
    self.KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

    # **Rotational** baseline parameters
    # self.KP_rot = np.matrix(1e2 * np.diag([1,1,0.5]))
    # self.KP_rot = np.matrix(5e1 * np.diag([0.1,0.1,0.5]))
    self.KP_rot = np.matrix(5e1 * np.diag([0.05,0.05,0.5]))
    # self.KI_rot = np.matrix(1e2 * np.diag([1,1,1]))
    self.KI_rot = np.matrix(1e2 * np.diag([1,1,1]))

    # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)       
    # self.KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,1]))
    # self.KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
    # self.KI_rot_PI_baseline = np.matrix(1e-1 * np.diag([1,1,0.5]))
    self.KP_rot_PI_baseline = np.matrix(200 * np.diag([1,1,0.3]))
    self.KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
    self.KI_rot_PI_baseline = np.matrix(1e2 * np.diag([10,10,0.5]))

    # self.K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))
    self.K_P_omega_ref = np.matrix(1.5e0 * np.diag([50,50,10]))
    self.K_I_omega_ref = np.matrix(1e2 * np.diag([1,1,1]))

    # ----------------------------------------------------------------
    #                   Translational Parameters MRAC
    # ----------------------------------------------------------------

    # Plant parameters **Translational** dynamics
    self.A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                        [np.zeros((3, 3)), np.zeros((3, 3))]])

    self.B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                      [np.identity(3)]]))

    self.A_tran_bar = self.A_tran # Estimated A_tran matrix
    self.Lambda_bar = (1/self.mass_total_estimated) * np.identity(3) # Estimate of \Lambda
    self.Theta_tran_adaptive_bar = self.air_density_estimated * self.surface_area_estimated * self.drag_coefficient_matrix_estimated

    # **Translational** reference model parameters and estimates
    self.A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                            [        -self.KP_tran,        -self.KD_tran]])

    self.B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                      [(1/self.mass_total_estimated)*np.identity(3)]]))

    # **Translational** adaptive parameters
    self.Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,10,1,1,10])) # Adaptive rates
    self.Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
    self.Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

    # **Translational** parameters Lyapunov equation
    self.Q_tran = np.matrix(4e-3 * np.diag([1,1,20,1,1,20]))
    self.P_tran = np.matrix(linalg.solve_continuous_lyapunov(self.A_ref_tran.T, -self.Q_tran))
    
    # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
    # Used for baseline controller and to center constraining ellipsoid when using projection operator
    self.K_x_tran_bar = (LA.pinv(self.B_tran*self.Lambda_bar)*(self.A_ref_tran - self.A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
    self.K_r_tran_bar = (LA.pinv(self.B_tran*self.Lambda_bar)*self.B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

    # ----------------------------------------------------------------
    #                   Rotational Parameters MRAC
    # ----------------------------------------------------------------

    # Plant parameters **Rotational** dynamics
    self.A_rot = np.matrix(np.zeros((3,3)))
    self.B_rot = np.matrix(np.eye(3))

    # **Rotational** reference model parameters
    self.A_ref_rot = -self.K_P_omega_ref
    self.B_ref_rot = np.matrix(np.eye(3))

    # **Rotational** parameters Lyapunov equation
    # self.Q_rot = np.matrix(7e-3 * np.diag([1,1,2]))
    self.Q_rot = np.matrix(7e-3 * np.diag([2,2,2]))
    self.P_rot = np.matrix(linalg.solve_continuous_lyapunov(self.A_ref_rot.T, -self.Q_rot))

    # **Rotational** adaptive parameters
    # self.Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,10])) # Adaptive rates
    # self.Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
    # self.Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
    self.Gamma_x_rot = np.matrix(1e4 * np.diag([1,1,1])) # Adaptive rates
    self.Gamma_r_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates
    self.Gamma_Theta_rot = np.matrix(1e2 * np.diag([1,1,1,1,1,1])) # Adaptive rates
    
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------
    self.use_safety_mechanism = False
    
    # Mu - sphere intersection
    self.sphereEpsilon = 1e-2
    self.maximumThrust = 85 # [N] 85
    
    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = 1e-2
    self.maximumRollAngle = math.radians(60) # [rad] 25 - 32
    self.maximumPitchAngle = math.radians(60) # [rad] 25 - 32
    
    # Mu - plane intersection
    self.planeEpsilon = 1e-2
    self.alphaPlane = 0.95 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.