import math
import numpy as np
from numpy import linalg as LA
import scipy
from scipy import linalg





class Gains:
    # ================================================================================================================================================================
    # PID
    # ================================================================================================================================================================
    def PID():
        
        # Number of states to be integrated by RK4
        number_of_states = 10
        # Length of the array vector that will be exported 
        size_DATA = 47 
        # size_DATA = 70 # OLD data format
        
        # **Translational** PID parameters 
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,1]))

        # **Rotational** PID parameters
        KP_rot = np.matrix(1 * np.diag([100,100,50]))
        KD_rot = np.matrix(1 * np.diag([50,50,50]))
        KI_rot = np.matrix(1 * np.diag([20,20,10]))
        
        # ----------------------------------------------------------------
        #                   Safety Mechanism Parameters
        # ----------------------------------------------------------------
        
        # Mu - sphere intersection
        sphereEpsilon = 1e-2
        maximumThrust = 85 # [N] 85
        
        # Mu - elliptic cone intersection
        EllipticConeEpsilon = 1e-2
        maximumRollAngle = math.radians(32) # [rad] 25
        maximumPitchAngle = math.radians(32) # [rad] 25
        
        # Mu - plane intersection
        planeEpsilon = 1e-2
        alphaPlane = 0.95 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_rot,KD_rot,KI_rot,sphereEpsilon,maximumThrust,
        EllipticConeEpsilon,maximumRollAngle,maximumPitchAngle,planeEpsilon,alphaPlane]
    # ================================================================================================================================================================
    # end of PID
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # MRAC WITH BASELINE
    # ================================================================================================================================================================
    def MRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 100
        # Length of the array vector that will be exported 
        size_DATA = 74 
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1e2 * np.diag([1,1,0.5]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)       
        KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,1]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Translational** parameters Lyapunov equation
        Q_tran = np.matrix(4e-3 * np.diag([1,1,20,1,1,20]))
        P_tran = np.matrix(linalg.solve_continuous_lyapunov(A_ref_tran.T, -Q_tran))
        
        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** parameters Lyapunov equation
        Q_rot = np.matrix(7e-3 * np.diag([1,1,2]))
        P_rot = np.matrix(linalg.solve_continuous_lyapunov(A_ref_rot.T, -Q_rot))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,10])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,Q_tran,P_tran,K_x_tran_bar,K_r_tran_bar,A_rot,
                B_rot,A_ref_rot,B_ref_rot,Q_rot,P_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot]
    # ================================================================================================================================================================
    # end of MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    def TwoLayerMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 136
        # Length of the array vector that will be exported 
        size_DATA = 74
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(8e-1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1e1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1.5 * np.diag([10,10,10]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(70 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(10 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(5e-3 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e3 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(3e1 * np.diag([1,1,10])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e1 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(4e2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(5e-2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #              Additional gains for 2-Layer MRAC
        # ----------------------------------------------------------------

        # **Translational** second layer parameters
        poles_ref_tran = LA.eig(A_ref_tran)[0]
        poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
        K_transient_tran = scipy.signal.place_poles(A_tran, B_ref_tran, poles_transient_tran)
        K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
        A_transient_tran = A_tran - B_ref_tran*K_transient_tran # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_tran_2Layer = np.matrix(2e-2 * np.diag([1,1,12,1,1,2]))
        P_tran_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_tran.T, -Q_tran_2Layer))

        Gamma_g_tran = np.matrix(1e2 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Rotational** second layer parameters
        poles_ref_rot = LA.eig(A_ref_rot)[0]
        poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
        K_transient_rot = scipy.signal.place_poles(A_rot, B_ref_rot, poles_transient_rot)
        K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
        A_transient_rot = A_rot - B_ref_rot*K_transient_rot # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_rot_2Layer = np.matrix(1e-3 * np.diag([1,10,1]))
        P_rot_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_rot.T, -Q_rot_2Layer))

        Gamma_g_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates

        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))
        K_g_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_transient_tran - A_ref_tran)).T * np.matrix(0.001*np.diag([1,1,1]))
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,A_rot,B_rot,A_ref_rot,
                B_ref_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,A_transient_tran,Q_tran_2Layer,P_tran_2Layer,Gamma_g_tran,
                A_transient_rot,Q_rot_2Layer,P_rot_2Layer,Gamma_g_rot,K_x_tran_bar,K_r_tran_bar,K_g_tran_bar]
    # ================================================================================================================================================================
    # end of TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # ROBUST MRAC WITH BASELINE
    # ================================================================================================================================================================
    def RobustMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 100
        # Length of the array vector that will be exported 
        size_DATA = 74 
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1 * np.diag([100,100,50]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        # Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,0.1,1,1,0.1])) # Adaptive rates
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Translational** parameters Lyapunov equation
        Q_tran = np.matrix(4e-3 * np.diag([1,1,20,1,1,20]))
        P_tran = np.matrix(linalg.solve_continuous_lyapunov(A_ref_tran.T, -Q_tran))
        
        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** parameters Lyapunov equation
        Q_rot = np.matrix(7e-3 * np.diag([1,1,1]))
        P_rot = np.matrix(linalg.solve_continuous_lyapunov(A_ref_rot.T, -Q_rot))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        
        # ----------------------------------------------------------------
        #             Gains for Robust MRAC (sigma-mod and e-mod)
        # ----------------------------------------------------------------
        
        # **Translational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!

        # **Rotational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,Q_tran,P_tran,K_x_tran_bar,K_r_tran_bar,A_rot,
                B_rot,A_ref_rot,B_ref_rot,Q_rot,P_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,sigma_x_tran,sigma_r_tran,
                sigma_theta_tran,sigma_x_rot,sigma_r_rot,sigma_theta_rot]
    # ================================================================================================================================================================
    # end of ROBUST MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # ROBUST TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    def RobustTwoLayerMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 136
        # Length of the array vector that will be exported 
        size_DATA = 74
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1 * np.diag([100,100,50]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(20 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,20,1,1,20])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e2 * np.diag([1,1,10])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(2e1 * np.diag([1,10,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e-3 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #              Additional gains for 2-Layer MRAC
        # ----------------------------------------------------------------

        # **Translational** second layer parameters
        poles_ref_tran = LA.eig(A_ref_tran)[0]
        poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
        K_transient_tran = scipy.signal.place_poles(A_tran, B_ref_tran, poles_transient_tran)
        K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
        A_transient_tran = A_tran - B_ref_tran*K_transient_tran # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        # Q_tran_2Layer = np.matrix(1e-2 * np.diag([1,1,12,1,1,2]))
        Q_tran_2Layer = np.matrix(1e-2 * np.diag([1,1,12,1,1,3]))
        P_tran_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_tran.T, -Q_tran_2Layer))

        Gamma_g_tran = np.matrix(1e2 * np.diag([1,1,100,1,1,100])) # Adaptive rates

        # **Rotational** second layer parameters
        poles_ref_rot = LA.eig(A_ref_rot)[0]
        poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
        K_transient_rot = scipy.signal.place_poles(A_rot, B_ref_rot, poles_transient_rot)
        K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
        A_transient_rot = A_rot - B_ref_rot*K_transient_rot # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_rot_2Layer = np.matrix(8e-4 * np.diag([1,1,1]))
        P_rot_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_rot.T, -Q_rot_2Layer))

        Gamma_g_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates

        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))
        K_g_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_transient_tran - A_ref_tran)).T * np.matrix(0.001*np.diag([1,1,1]))
        
        # ----------------------------------------------------------------
        #             Gains for Robust MRAC (sigma-mod and e-mod)
        # ----------------------------------------------------------------
        
        # **Translational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        sigma_g_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!

        # **Rotational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        sigma_g_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,A_rot,B_rot,A_ref_rot,
                B_ref_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,A_transient_tran,Q_tran_2Layer,P_tran_2Layer,Gamma_g_tran,
                A_transient_rot,Q_rot_2Layer,P_rot_2Layer,Gamma_g_rot,K_x_tran_bar,K_r_tran_bar,K_g_tran_bar,sigma_x_tran,sigma_r_tran,
                sigma_theta_tran,sigma_g_tran,sigma_x_rot,sigma_r_rot,sigma_theta_rot,sigma_g_rot]
    # ================================================================================================================================================================
    # end of ROBUST TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # HYBRID MRAC WITH BASELINE
    # ================================================================================================================================================================
    def HybridMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 102
        # Length of the array vector that will be exported 
        size_DATA = 74 
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(10 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(10 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1 * np.diag([100,100,50]))

        # **Rotational** parameters for the PID baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,5,1,1,5])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Translational** parameters Lyapunov equation
        Q_tran = np.matrix(4e-3 * np.diag([1,1,30,1,1,30]))
        P_tran = np.matrix(linalg.solve_continuous_lyapunov(A_ref_tran.T, -Q_tran))
        
        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** parameters Lyapunov equation
        Q_rot = np.matrix(7e-3 * np.diag([1,1,1]))
        P_rot = np.matrix(linalg.solve_continuous_lyapunov(A_ref_rot.T, -Q_rot))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        
        # ----------------------------------------------------------------
        #                   Hybrid Parameters MRAC
        # ----------------------------------------------------------------
        
        flag_first_loop_controller = False
        time_reset_trajectory_tran = []
        time_reset_trajectory_rot = []
        e_tran = np.zeros((6,1))
        e_rot = np.zeros((3,1))
        summation_hybrid_P_tran = 0
        summation_hybrid_P_rot = 0
        s_hybrid_tran = 0
        s_hybrid_rot = 0
        tollerance_time_reset_series = 1e-1 # 5e-1 1e-1
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,Q_tran,P_tran,K_x_tran_bar,K_r_tran_bar,A_rot,
                B_rot,A_ref_rot,B_ref_rot,Q_rot,P_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,flag_first_loop_controller,
                time_reset_trajectory_tran,time_reset_trajectory_rot,e_tran,e_rot,summation_hybrid_P_tran,summation_hybrid_P_rot,
                s_hybrid_tran,s_hybrid_rot,tollerance_time_reset_series]
    # ================================================================================================================================================================
    # end of HYBRID MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # HYBRID TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    def HybridTwoLayerMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 140
        # Length of the array vector that will be exported 
        size_DATA = 74
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(8e-1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1e1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1.5 * np.diag([10,10,10]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(70 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(10 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e3 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(3e1 * np.diag([1,1,10])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e1 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(4e2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(5e-2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #              Additional gains for 2-Layer MRAC
        # ----------------------------------------------------------------

        # **Translational** second layer parameters
        poles_ref_tran = LA.eig(A_ref_tran)[0]
        poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
        K_transient_tran = scipy.signal.place_poles(A_tran, B_ref_tran, poles_transient_tran)
        K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
        A_transient_tran = A_tran - B_ref_tran*K_transient_tran # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_tran_2Layer = np.matrix(2e-2 * np.diag([1,1,12,1,1,2]))
        P_tran_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_tran.T, -Q_tran_2Layer))

        Gamma_g_tran = np.matrix(1e2 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Rotational** second layer parameters
        poles_ref_rot = LA.eig(A_ref_rot)[0]
        poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
        K_transient_rot = scipy.signal.place_poles(A_rot, B_ref_rot, poles_transient_rot)
        K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
        A_transient_rot = A_rot - B_ref_rot*K_transient_rot # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_rot_2Layer = np.matrix(1e-3 * np.diag([1,10,1]))
        P_rot_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_rot.T, -Q_rot_2Layer))

        Gamma_g_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates

        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))
        K_g_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_transient_tran - A_ref_tran)).T * np.matrix(0.001*np.diag([1,1,1]))
        
        # ----------------------------------------------------------------
        #                   Hybrid Parameters MRAC
        # ----------------------------------------------------------------
        
        flag_first_loop_controller = False
        time_reset_trajectory_tran = []
        time_reset_trajectory_rot = []
        epsilon_tran = np.zeros((6,1))
        epsilon_rot = np.zeros((3,1))
        summation_hybrid_P_tran = 0
        summation_hybrid_P_rot = 0
        s_hybrid_tran = 0
        s_hybrid_rot = 0
        tollerance_time_reset_series = 5e-1 # 5e-1
        
        e_transient_tran = np.zeros((6,1))
        e_transient_rot = np.zeros((3,1))
        summation_hybrid_transient_P_tran = 0
        summation_hybrid_transient_P_rot = 0
        s_hybrid_transient_tran = 0
        s_hybrid_transient_rot = 0
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,A_rot,B_rot,A_ref_rot,
                B_ref_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,A_transient_tran,Q_tran_2Layer,P_tran_2Layer,Gamma_g_tran,
                A_transient_rot,Q_rot_2Layer,P_rot_2Layer,Gamma_g_rot,K_x_tran_bar,K_r_tran_bar,K_g_tran_bar,flag_first_loop_controller,
                time_reset_trajectory_tran,time_reset_trajectory_rot,epsilon_tran,epsilon_rot,summation_hybrid_P_tran,summation_hybrid_P_rot,
                s_hybrid_tran,s_hybrid_rot,tollerance_time_reset_series,e_transient_tran,e_transient_rot,summation_hybrid_transient_P_tran,
                summation_hybrid_transient_P_rot,s_hybrid_transient_tran,s_hybrid_transient_rot]
    # ================================================================================================================================================================
    # end of HYBRID TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # HYBRID ROBUST MRAC WITH BASELINE
    # ================================================================================================================================================================
    def HybridRobustMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 102
        # Length of the array vector that will be exported 
        size_DATA = 74 
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(10 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(10 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1 * np.diag([100,100,50]))

        # **Rotational** parameters for the PID baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,5,1,1,5])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Translational** parameters Lyapunov equation
        Q_tran = np.matrix(4e-3 * np.diag([1,1,30,1,1,30]))
        P_tran = np.matrix(linalg.solve_continuous_lyapunov(A_ref_tran.T, -Q_tran))
        
        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** parameters Lyapunov equation
        Q_rot = np.matrix(7e-3 * np.diag([1,1,1]))
        P_rot = np.matrix(linalg.solve_continuous_lyapunov(A_ref_rot.T, -Q_rot))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        
        # ----------------------------------------------------------------
        #             Gains for Robust MRAC (sigma-mod and e-mod)
        # ----------------------------------------------------------------
        
        # **Translational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!

        # **Rotational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        
        # ----------------------------------------------------------------
        #                   Hybrid Parameters MRAC
        # ----------------------------------------------------------------
        
        flag_first_loop_controller = False
        time_reset_trajectory_tran = []
        time_reset_trajectory_rot = []
        e_tran = np.zeros((6,1))
        e_rot = np.zeros((3,1))
        summation_hybrid_P_tran = 0
        summation_hybrid_P_rot = 0
        s_hybrid_tran = 0
        s_hybrid_rot = 0
        tollerance_time_reset_series = 5e-1 # 5e-1
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,Q_tran,P_tran,K_x_tran_bar,K_r_tran_bar,A_rot,
                B_rot,A_ref_rot,B_ref_rot,Q_rot,P_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,sigma_x_tran,sigma_r_tran,
                sigma_theta_tran,sigma_x_rot,sigma_r_rot,sigma_theta_rot,flag_first_loop_controller,time_reset_trajectory_tran,
                time_reset_trajectory_rot,e_tran,e_rot,summation_hybrid_P_tran,summation_hybrid_P_rot,
                s_hybrid_tran,s_hybrid_rot,tollerance_time_reset_series]
    # ================================================================================================================================================================
    # end of HYBRID ROBUST MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # HYBRID ROBUST TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    def HybridRobustTwoLayerMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 140
        # Length of the array vector that will be exported 
        size_DATA = 74
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(8e-1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1e1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1.5 * np.diag([10,10,10]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(70 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(10 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e3 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(3e1 * np.diag([1,1,10])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e1 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(4e2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(5e-2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #              Additional gains for 2-Layer MRAC
        # ----------------------------------------------------------------

        # **Translational** second layer parameters
        poles_ref_tran = LA.eig(A_ref_tran)[0]
        poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
        K_transient_tran = scipy.signal.place_poles(A_tran, B_ref_tran, poles_transient_tran)
        K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
        A_transient_tran = A_tran - B_ref_tran*K_transient_tran # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_tran_2Layer = np.matrix(2e-2 * np.diag([1,1,12,1,1,2]))
        P_tran_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_tran.T, -Q_tran_2Layer))

        Gamma_g_tran = np.matrix(1e2 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Rotational** second layer parameters
        poles_ref_rot = LA.eig(A_ref_rot)[0]
        poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
        K_transient_rot = scipy.signal.place_poles(A_rot, B_ref_rot, poles_transient_rot)
        K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
        A_transient_rot = A_rot - B_ref_rot*K_transient_rot # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_rot_2Layer = np.matrix(1e-3 * np.diag([1,10,1]))
        P_rot_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_rot.T, -Q_rot_2Layer))

        Gamma_g_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates

        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))
        K_g_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_transient_tran - A_ref_tran)).T * np.matrix(0.001*np.diag([1,1,1]))
        
        # ----------------------------------------------------------------
        #             Gains for Robust MRAC (sigma-mod and e-mod)
        # ----------------------------------------------------------------
        
        # **Translational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        sigma_g_tran = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!

        # **Rotational** parameters for \sigma- and e-modifications of MRAC
        sigma_x_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_x # SMALL and POSITIVE!
        sigma_r_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{K}_r # SMALL and POSITIVE!
        sigma_theta_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        sigma_g_rot = 0.01 # Damping parameter for \sigma- and e-modification of MRAC, \hat{\Theta} # SMALL and POSITIVE!
        
        # ----------------------------------------------------------------
        #                   Hybrid Parameters MRAC
        # ----------------------------------------------------------------
        
        flag_first_loop_controller = False
        time_reset_trajectory_tran = []
        time_reset_trajectory_rot = []
        epsilon_tran = np.zeros((6,1))
        epsilon_rot = np.zeros((3,1))
        summation_hybrid_P_tran = 0
        summation_hybrid_P_rot = 0
        s_hybrid_tran = 0
        s_hybrid_rot = 0
        tollerance_time_reset_series = 5e-1 # 5e-1
        
        e_transient_tran = np.zeros((6,1))
        e_transient_rot = np.zeros((3,1))
        summation_hybrid_transient_P_tran = 0
        summation_hybrid_transient_P_rot = 0
        s_hybrid_transient_tran = 0
        s_hybrid_transient_rot = 0
        
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,A_rot,B_rot,A_ref_rot,
                B_ref_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,A_transient_tran,Q_tran_2Layer,P_tran_2Layer,Gamma_g_tran,
                A_transient_rot,Q_rot_2Layer,P_rot_2Layer,Gamma_g_rot,K_x_tran_bar,K_r_tran_bar,K_g_tran_bar,sigma_x_tran,sigma_r_tran,
                sigma_theta_tran,sigma_g_tran,sigma_x_rot,sigma_r_rot,sigma_theta_rot,sigma_g_rot,flag_first_loop_controller,
                time_reset_trajectory_tran,time_reset_trajectory_rot,epsilon_tran,epsilon_rot,summation_hybrid_P_tran,summation_hybrid_P_rot,
                s_hybrid_tran,s_hybrid_rot,tollerance_time_reset_series,e_transient_tran,e_transient_rot,summation_hybrid_transient_P_tran,
                summation_hybrid_transient_P_rot,s_hybrid_transient_tran,s_hybrid_transient_rot]
    # ================================================================================================================================================================
    # end of HYBRID ROBUST TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # FUNNEL MRAC WITH BASELINE
    # ================================================================================================================================================================
    def FunnelMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
         
        # Number of states to be integrated by RK4
        number_of_states = 102
        # Length of the array vector that will be exported 
        size_DATA = 74 
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1 * np.diag([100,100,50]))

        # **Rotational** parameters for the PID baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Translational** parameters Lyapunov equation
        Q_tran = np.matrix(4e-3 * np.diag([1,1,20,1,1,20]))
        P_tran = np.matrix(linalg.solve_continuous_lyapunov(A_ref_tran.T, -Q_tran))
        
        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** parameters Lyapunov equation
        Q_rot = np.matrix(7e-3 * np.diag([1,1,1]))
        P_rot = np.matrix(linalg.solve_continuous_lyapunov(A_ref_rot.T, -Q_rot))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        
        # ----------------------------------------------------------------
        #                   Funnel Parameters MRAC
        # ----------------------------------------------------------------
        
        # **Translational** Funnel parameters
        eta_max_funnel_tran = 1
        M_funnel_tran = np.matrix(1e-1 * np.diag([1,1,1,1,1,1])) # n x n
        u_max = 85
        u_min = 0
        Delta_u_min = 5
        nu_funnel_tran = 0.01
        
        # **Rotational** Funnel parameters
        eta_max_funnel_rot = 1
        M_funnel_rot = np.matrix(1e-3 * np.diag([1,1,1])) # n x n
        Moment_max = 5
        Moment_min = 0
        Delta_Moment_min = 0.01
        nu_funnel_rot = 0.01
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,Q_tran,P_tran,K_x_tran_bar,K_r_tran_bar,A_rot,
                B_rot,A_ref_rot,B_ref_rot,Q_rot,P_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,eta_max_funnel_tran,M_funnel_tran,u_max,u_min,
                Delta_u_min,nu_funnel_tran,eta_max_funnel_rot,M_funnel_rot,Moment_max,Moment_min,Delta_Moment_min,nu_funnel_rot]
    # ================================================================================================================================================================
    # end of FUNNEL MRAC WITH BASELINE
    # ================================================================================================================================================================

    # ================================================================================================================================================================
    # FUNNEL TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    def FunnelTwoLayerMRACwithBASELINE(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        number_of_states = 138
        # Length of the array vector that will be exported 
        size_DATA = 74
        # size_DATA = 94 # OLD data format
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(8e-1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1e1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        KP_rot = np.matrix(1.5 * np.diag([10,10,10]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        KP_rot_PI_baseline = np.matrix(70 * np.diag([1,1,0.5]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(10 * np.diag([1,1,0.5]))

        K_P_omega_ref = np.matrix(5e-1 * np.diag([5,5,10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e3 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(3e1 * np.diag([1,1,10])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e1 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** adaptive parameters
        Gamma_x_rot = np.matrix(4e2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(5e-2 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # ----------------------------------------------------------------
        #              Additional gains for 2-Layer MRAC
        # ----------------------------------------------------------------

        # **Translational** second layer parameters
        poles_ref_tran = LA.eig(A_ref_tran)[0]
        poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
        K_transient_tran = scipy.signal.place_poles(A_tran, B_ref_tran, poles_transient_tran)
        K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
        A_transient_tran = A_tran - B_ref_tran*K_transient_tran # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_tran_2Layer = np.matrix(2e-2 * np.diag([1,1,12,1,1,2]))
        P_tran_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_tran.T, -Q_tran_2Layer))

        Gamma_g_tran = np.matrix(1e2 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Rotational** second layer parameters
        poles_ref_rot = LA.eig(A_ref_rot)[0]
        poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
        K_transient_rot = scipy.signal.place_poles(A_rot, B_ref_rot, poles_transient_rot)
        K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
        A_transient_rot = A_rot - B_ref_rot*K_transient_rot # Eigenvalues of A_transient should be further on the left in the complex plane of those of A_ref!!!!

        Q_rot_2Layer = np.matrix(1e-4 * np.diag([1,0.1,1]))
        P_rot_2Layer = np.matrix(linalg.solve_continuous_lyapunov(A_transient_rot.T, -Q_rot_2Layer))

        Gamma_g_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates

        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))
        K_g_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_transient_tran - A_ref_tran)).T * np.matrix(0.001*np.diag([1,1,1]))
        
        # ----------------------------------------------------------------
        #                   Funnel Parameters MRAC
        # ----------------------------------------------------------------
        
        # **Translational** Funnel parameters
        eta_max_funnel_tran = 1
        M_funnel_tran = np.matrix(1e-4 * np.diag([1,1,1,1,1,1])) # n x n
        u_max = 85
        u_min = 0
        Delta_u_min = 5 # 5
        nu_funnel_tran = 0.1 # 0.01
        
        # **Rotational** Funnel parameters
        eta_max_funnel_rot = 0.1
        M_funnel_rot = np.matrix(1e-4 * np.diag([1,1,1])) # n x n
        Moment_max = 5
        Moment_min = 0
        Delta_Moment_min = 0.01
        nu_funnel_rot = 0.01
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,A_rot,B_rot,A_ref_rot,
                B_ref_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,A_transient_tran,Q_tran_2Layer,P_tran_2Layer,Gamma_g_tran,
                A_transient_rot,Q_rot_2Layer,P_rot_2Layer,Gamma_g_rot,K_x_tran_bar,K_r_tran_bar,K_g_tran_bar,eta_max_funnel_tran,
                M_funnel_tran,u_max,u_min,Delta_u_min,nu_funnel_tran,eta_max_funnel_rot,M_funnel_rot,Moment_max,Moment_min,
                Delta_Moment_min,nu_funnel_rot]
    # ================================================================================================================================================================
    # end of FUNNEL TWO-LAYER MRAC WITH BASELINE
    # ================================================================================================================================================================
    
    # ================================================================================================================================================================
    # MRAC WITH BASELINE with Safety Mechanism
    # ================================================================================================================================================================
    def MRACwithBASELINE_SafetyMechanism(mass_total_estimated,air_density_estimated,surface_area_estimated,drag_coefficient_matrix_estimated):
        
        # Number of states to be integrated by RK4
        # number_of_states = 100
        number_of_states = 106
        # Length of the array vector that will be exported 
        # size_DATA = 74 
        size_DATA = 86 
        
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
        KP_tran = np.matrix(1 * np.diag([5,5,6]))
        KD_tran = np.matrix(1 * np.diag([8,8,3]))
        KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

        # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
        KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
        KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

        # **Rotational** baseline parameters
        # KP_rot = np.matrix(1e2 * np.diag([1,1,0.5]))
        # KP_rot = np.matrix(5e1 * np.diag([0.1,0.1,0.5]))
        KP_rot = np.matrix(5e1 * np.diag([0.05,0.05,0.5]))
        # KI_rot = np.matrix(1e2 * np.diag([1,1,1]))
        KI_rot = np.matrix(1e2 * np.diag([1,1,1]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)       
        # KP_rot_PI_baseline = np.matrix(40 * np.diag([1,1,1]))
        # KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        # KI_rot_PI_baseline = np.matrix(1e-1 * np.diag([1,1,0.5]))
        KP_rot_PI_baseline = np.matrix(200 * np.diag([1,1,0.3]))
        KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,0.5]))
        KI_rot_PI_baseline = np.matrix(1e2 * np.diag([10,10,0.5]))

        # K_P_omega_ref = np.matrix(1.5e-1 * np.diag([5,5,10]))
        K_P_omega_ref = np.matrix(1.5e0 * np.diag([50,50,10]))
        K_I_omega_ref = np.matrix(1e2 * np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Translational** dynamics
        A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                           [np.zeros((3, 3)), np.zeros((3, 3))]])

        B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [np.identity(3)]]))

        A_tran_bar = A_tran # Estimated A_tran matrix
        Lambda_bar = (1/mass_total_estimated) * np.identity(3) # Estimate of \Lambda
        Theta_tran_adaptive_bar = air_density_estimated * surface_area_estimated * drag_coefficient_matrix_estimated

        # **Translational** reference model parameters and estimates
        A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                               [        -KP_tran,        -KD_tran]])

        B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                         [(1/mass_total_estimated)*np.identity(3)]]))

        # **Translational** adaptive parameters
        Gamma_x_tran = np.matrix(5e2 * np.diag([1,1,10,1,1,10])) # Adaptive rates
        Gamma_r_tran = np.matrix(1e0 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_tran = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates

        # **Translational** parameters Lyapunov equation
        Q_tran = np.matrix(4e-3 * np.diag([1,1,20,1,1,20]))
        P_tran = np.matrix(linalg.solve_continuous_lyapunov(A_ref_tran.T, -Q_tran))
        
        # Estimates of the matrices that verify the matching conditions for the **Translational** dynamics.
        # Used for baseline controller and to center constraining ellipsoid when using projection operator
        K_x_tran_bar = (LA.pinv(B_tran*Lambda_bar)*(A_ref_tran - A_tran_bar)).T * np.matrix(1*np.diag([1,1,1]))
        K_r_tran_bar = (LA.pinv(B_tran*Lambda_bar)*B_ref_tran).T * np.matrix(1*np.diag([1,1,1]))

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        # Plant parameters **Rotational** dynamics
        A_rot = np.matrix(np.zeros((3,3)))
        B_rot = np.matrix(np.eye(3))

        # **Rotational** reference model parameters
        A_ref_rot = -K_P_omega_ref
        B_ref_rot = np.matrix(np.eye(3))

        # **Rotational** parameters Lyapunov equation
        # Q_rot = np.matrix(7e-3 * np.diag([1,1,2]))
        Q_rot = np.matrix(7e-3 * np.diag([2,2,2]))
        P_rot = np.matrix(linalg.solve_continuous_lyapunov(A_ref_rot.T, -Q_rot))

        # **Rotational** adaptive parameters
        # Gamma_x_rot = np.matrix(1e1 * np.diag([1,1,10])) # Adaptive rates
        # Gamma_r_rot = np.matrix(1e-4 * np.diag([1,1,1])) # Adaptive rates
        # Gamma_Theta_rot = np.matrix(1e0 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        Gamma_x_rot = np.matrix(1e4 * np.diag([1,1,1])) # Adaptive rates
        Gamma_r_rot = np.matrix(1e1 * np.diag([1,1,1])) # Adaptive rates
        Gamma_Theta_rot = np.matrix(1e2 * np.diag([1,1,1,1,1,1])) # Adaptive rates
        
        # ----------------------------------------------------------------
        #                   Safety Mechanism Parameters
        # ----------------------------------------------------------------
        
        # Mu - sphere intersection
        sphereEpsilon = 1e-2
        maximumThrust = 85 # [N] 85
        
        # Mu - elliptic cone intersection
        EllipticConeEpsilon = 1e-2
        maximumRollAngle = math.radians(60) # [rad] 25 - 32
        maximumPitchAngle = math.radians(60) # [rad] 25 - 32
        
        # Mu - plane intersection
        planeEpsilon = 1e-2
        alphaPlane = 0.95 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
        
        
        return [number_of_states,size_DATA,KP_tran,KD_tran,KI_tran,KP_tran_PD_baseline,KD_tran_PD_baseline,KP_rot,KI_rot,KP_rot_PI_baseline,
                KD_rot_PI_baseline,KI_rot_PI_baseline,K_P_omega_ref,K_I_omega_ref,A_tran,B_tran,A_tran_bar,Lambda_bar,Theta_tran_adaptive_bar,
                A_ref_tran,B_ref_tran,Gamma_x_tran,Gamma_r_tran,Gamma_Theta_tran,Gamma_Theta_tran,Q_tran,P_tran,K_x_tran_bar,K_r_tran_bar,A_rot,
                B_rot,A_ref_rot,B_ref_rot,Q_rot,P_rot,Gamma_x_rot,Gamma_r_rot,Gamma_Theta_rot,sphereEpsilon,maximumThrust,
                EllipticConeEpsilon,maximumRollAngle,maximumPitchAngle,planeEpsilon,alphaPlane]
    # ================================================================================================================================================================
    # end of MRAC WITH BASELINE  with Safety Mechanism
    # ================================================================================================================================================================
   
   
   