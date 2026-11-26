import math
import numpy as np
from numpy import linalg as LA
from scipy import linalg
import scipy
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.control.projection_operator import ProjectionOperator
from acsl_pychrono.control.base_mrac_gains import BaseMRACGains

class TwoLayerMRACGains(BaseMRACGains):
  def __init__(self, flight_params: FlightParams):
    # General vehicle properties
    self.I_matrix_estimated = flight_params.uav.I_matrix_estimated
    self.mass_total_estimated = flight_params.uav.mass_total_estimated
    self.air_density_estimated = flight_params.uav.air_density_estimated
    self.surface_area_estimated = flight_params.uav.surface_area_estimated
    self.drag_coefficient_matrix_estimated = flight_params.uav.drag_coefficient_matrix_estimated

    # Controller's numerical Parameters config filename
    gains_config_filename = flight_params.uav.controller_config_filename
    gains_config_file = flight_params.get_controller_config(gains_config_filename, flight_params.uav.name)
    
    # Number of states to be integrated by RK4
    self.number_of_states = 133

    # ----------------------------------------------------------------
    #                     Baseline Parameters
    # ----------------------------------------------------------------

    # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
    self.KP_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_tran")
    self.KD_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "KD_tran")
    self.KI_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "KI_tran")

    # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
    self.KP_tran_PD_baseline = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_tran_PD_baseline")
    self.KD_tran_PD_baseline = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_tran_PD_baseline")

    # **Rotational** baseline parameters
    self.KP_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_rot")
    self.KI_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "KI_rot")

    # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)       
    self.KP_rot_PI_baseline = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_rot_PI_baseline")
    self.KI_rot_PI_baseline = flight_params.get_scaled_matrix_from_config(gains_config_file, "KI_rot_PI_baseline")

    self.K_P_omega_ref = flight_params.get_scaled_matrix_from_config(gains_config_file, "K_P_omega_ref")
    self.K_I_omega_ref = flight_params.get_scaled_matrix_from_config(gains_config_file, "K_I_omega_ref")

    # ----------------------------------------------------------------
    #                   Translational Parameters MRAC
    # ----------------------------------------------------------------

    # Plant parameters **Translational** dynamics
    self.A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                        [np.zeros((3, 3)), np.zeros((3, 3))]])

    self.B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                      [np.identity(3)]]))

    # **Translational** reference model parameters and estimates
    self.A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                            [        -self.KP_tran,        -self.KD_tran]])

    self.B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                      [(1/self.mass_total_estimated)*np.identity(3)]]))

    # **Translational** adaptive parameters
    self.Gamma_x_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_x_tran")
    self.Gamma_r_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_r_tran")
    self.Gamma_Theta_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_Theta_tran")

    # **Translational** parameters Lyapunov equation
    self.Q_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "Q_tran")
    
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
    self.Q_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "Q_rot")
    
    # **Rotational** adaptive parameters
    self.Gamma_x_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_x_rot")
    self.Gamma_r_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_r_rot")
    self.Gamma_Theta_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_Theta_rot")

    # ----------------------------------------------------------------
    #                   Two-Layer MRAC Parameters
    # ----------------------------------------------------------------

    # **Translational** second layer parameters
    poles_ref_tran = LA.eig(self.A_ref_tran)[0]
    poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
    K_transient_tran = scipy.signal.place_poles(self.A_tran, self.B_ref_tran, poles_transient_tran)
    K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
    self.A_transient_tran = self.A_tran - self.B_ref_tran*K_transient_tran 

    self.P_tran = np.matrix(linalg.solve_continuous_lyapunov(self.A_transient_tran.T, -self.Q_tran))
    self.Gamma_g_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_g_tran") # Adaptive rates

    # **Rotational** second layer parameters
    poles_ref_rot = LA.eig(self.A_ref_rot)[0]
    poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
    K_transient_rot = scipy.signal.place_poles(self.A_rot, self.B_ref_rot, poles_transient_rot)
    K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
    self.A_transient_rot = self.A_rot - self.B_ref_rot*K_transient_rot
    
    self.P_rot = np.matrix(linalg.solve_continuous_lyapunov(self.A_transient_rot.T, -self.Q_rot))
    self.Gamma_g_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "Gamma_g_rot") # Adaptive rates
    
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------
    self.use_safety_mechanism = flight_params.get_scalar_from_config(gains_config_file, "use_safety_mechanism")
    
    # Mu - sphere intersection
    self.sphereEpsilon = flight_params.get_scalar_from_config(gains_config_file, "sphereEpsilon")
    self.maximumThrust = flight_params.get_scalar_from_config(gains_config_file, "maximumThrust") # [N] 85
    
    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = flight_params.get_scalar_from_config(gains_config_file, "EllipticConeEpsilon")
    self.maximumRollAngle = math.radians(flight_params.get_scalar_from_config(gains_config_file, "maximumRollAngle_deg")) # [rad] 25 - 32
    self.maximumPitchAngle = math.radians(flight_params.get_scalar_from_config(gains_config_file, "maximumPitchAngle_deg")) # [rad] 25 - 32
    
    # Mu - plane intersection
    self.planeEpsilon = flight_params.get_scalar_from_config(gains_config_file, "planeEpsilon")
    self.alphaPlane = flight_params.get_scalar_from_config(gains_config_file, "alphaPlane") # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
    
    # ----------------------------------------------------------------
    #                  Dead-Zone modification Parameters
    # ----------------------------------------------------------------
    self.use_dead_zone_modification = flight_params.get_scalar_from_config(gains_config_file, "use_dead_zone_modification")

    self.dead_zone_delta_tran = flight_params.get_scalar_from_config(gains_config_file, "dead_zone_delta_tran")
    self.dead_zone_e0_tran = flight_params.get_scalar_from_config(gains_config_file, "dead_zone_e0_tran")

    self.dead_zone_delta_rot = flight_params.get_scalar_from_config(gains_config_file, "dead_zone_delta_rot")
    self.dead_zone_e0_rot = flight_params.get_scalar_from_config(gains_config_file, "dead_zone_e0_rot")

    # ----------------------------------------------------------------
    #                  e-modification Parameters
    # ----------------------------------------------------------------
    self.use_e_modification = flight_params.get_scalar_from_config(gains_config_file, "use_e_modification")

    self.sigma_x_tran = flight_params.get_scalar_from_config(gains_config_file, "sigma_x_tran")
    self.sigma_r_tran = flight_params.get_scalar_from_config(gains_config_file, "sigma_r_tran")
    self.sigma_Theta_tran = flight_params.get_scalar_from_config(gains_config_file, "sigma_Theta_tran")
    self.sigma_g_tran = flight_params.get_scalar_from_config(gains_config_file, "sigma_g_tran")

    self.sigma_x_rot = flight_params.get_scalar_from_config(gains_config_file, "sigma_x_rot")
    self.sigma_r_rot = flight_params.get_scalar_from_config(gains_config_file, "sigma_r_rot")
    self.sigma_Theta_rot = flight_params.get_scalar_from_config(gains_config_file, "sigma_Theta_rot")
    self.sigma_g_rot = flight_params.get_scalar_from_config(gains_config_file, "sigma_g_rot")

    # ----------------------------------------------------------------
    #                  Projection Operator Parameters
    # ----------------------------------------------------------------
    self.use_projection_operator = flight_params.get_scalar_from_config(gains_config_file, "use_projection_operator")

    # K_x_hat translational
    self.x_e_x_tran = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_x_tran_transpose"))
    self.S_diagonal_x_tran = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_x_tran_transpose")))
    self.alpha_x_tran = flight_params.get_scalar_from_config(gains_config_file, "alpha_x_tran")

    # K_r_hat translational
    self.x_e_r_tran = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_r_tran_transpose"))
    self.S_diagonal_r_tran = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_r_tran_transpose")))
    self.alpha_r_tran = flight_params.get_scalar_from_config(gains_config_file, "alpha_r_tran")

    # Theta_hat translational
    self.x_e_Theta_tran = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_Theta_tran_transpose"))
    self.S_diagonal_Theta_tran = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_Theta_tran_transpose")))
    self.alpha_Theta_tran = flight_params.get_scalar_from_config(gains_config_file, "alpha_Theta_tran")

    # K_g_hat translational
    self.x_e_g_tran = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_g_tran_transpose"))
    self.S_diagonal_g_tran = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_g_tran_transpose")))
    self.alpha_g_tran = flight_params.get_scalar_from_config(gains_config_file, "alpha_g_tran")

    # K_x_hat rotational
    self.x_e_x_rot = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_x_rot_transpose"))
    self.S_diagonal_x_rot = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_x_rot_transpose")))
    self.alpha_x_rot = flight_params.get_scalar_from_config(gains_config_file, "alpha_x_rot")

    # K_r_hat rotational
    self.x_e_r_rot = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_r_rot_transpose"))
    self.S_diagonal_r_rot = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_r_rot_transpose")))
    self.alpha_r_rot = flight_params.get_scalar_from_config(gains_config_file, "alpha_r_rot")

    # Theta_hat rotational
    self.x_e_Theta_rot = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_Theta_rot_transpose"))
    self.S_diagonal_Theta_rot = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_Theta_rot_transpose")))
    self.alpha_Theta_rot = flight_params.get_scalar_from_config(gains_config_file, "alpha_Theta_rot")

    # K_g_hat rotational
    self.x_e_g_rot = np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "x_e_g_rot_transpose"))
    self.S_diagonal_g_rot = np.array(np.transpose(flight_params.get_scaled_matrix_from_config(gains_config_file, "S_diagonal_g_rot_transpose")))
    self.alpha_g_rot = flight_params.get_scalar_from_config(gains_config_file, "alpha_g_rot")

    # Generate S matrices from diagonal
    self.S_x_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_x_tran.flatten())
    self.S_r_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_r_tran.flatten())
    self.S_Theta_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_Theta_tran.flatten())
    self.S_g_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_g_tran.flatten())
    self.S_x_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_x_rot.flatten())
    self.S_r_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_r_rot.flatten())
    self.S_Theta_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_Theta_rot.flatten())
    self.S_g_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_g_rot.flatten())

    # Compute epsilon values from alpha
    self.epsilon_x_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_x_tran)
    self.epsilon_r_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_r_tran)
    self.epsilon_Theta_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_Theta_tran)
    self.epsilon_g_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_g_tran)
    self.epsilon_x_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_x_rot)
    self.epsilon_r_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_r_rot)
    self.epsilon_Theta_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_Theta_rot)
    self.epsilon_g_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_g_rot)

    # ----------------------------------------------------------------
    #     Non-Adaptive Error Bounding Control Input Parameters
    # ----------------------------------------------------------------
    self.use_error_bounding_control_input = flight_params.get_scalar_from_config(gains_config_file, "use_error_bounding_control_input")

    self.xi_bar_d_tran = flight_params.get_scalar_from_config(gains_config_file, "xi_bar_d_tran")
    self.lambda_bar_tran = flight_params.get_scalar_from_config(gains_config_file, "lambda_bar_tran")
    self.delta_ebci_tran = flight_params.get_scalar_from_config(gains_config_file, "delta_ebci_tran")
    
    self.xi_bar_d_rot = flight_params.get_scalar_from_config(gains_config_file, "xi_bar_d_rot")
    self.lambda_bar_rot = flight_params.get_scalar_from_config(gains_config_file, "lambda_bar_rot")
    self.delta_ebci_rot = flight_params.get_scalar_from_config(gains_config_file, "delta_ebci_rot")
    
    print(f"[INFO] Successfully loaded TwoLayerMRAC Gains")