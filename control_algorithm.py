import numpy as np
import math


global translational_position_in_I, translational_position_in_I_user, mass_total_estimated, I_matrix_estimated, translational_velocity_in_I
global translational_velocity_in_I_user, translational_acceleration_in_I_user, KP_tran, KD_tran, KI_tran, KP_rot, KD_rot, KI_rot
global G_acc, yaw_ref, yaw_ref_dot, yaw_ref_ddot, pixhawk_euler, angular_velocity, angular_error, x_tran




class controller:
    def PID(t, y):
        """
        This function defines the system of equations of the PID CONTROLLER that need to be integrated 
    
        """
        global mu_x, mu_y, mu_z, u1, roll_ref, pitch_ref, roll_ref_dot, pitch_ref_dot, roll_ref_ddot, pitch_ref_ddot
        global angular_position_ref_dot, angular_position_ref_ddot, Jacobian_matrix_inverse, angular_position_dot
        global angular_error_dot, u2, u3, u4
        
        state_phi_ref_diff = y[0:2]
        state_theta_ref_diff = y[2:4]
        integral_position_tracking = y[4:7]
        integral_angular_error = y[7:10]
        
        
        mu_tran_raw = mass_total_estimated*(-KP_tran * translational_position_error - KD_tran * (translational_velocity_in_I - translational_velocity_in_I_user) - KI_tran * integral_position_tracking + translational_acceleration_in_I_user).reshape(3,1)
        mu_x_raw = mu_tran_raw[0].item()
        mu_y_raw = mu_tran_raw[1].item()
        mu_z_raw = mu_tran_raw[2].item()
        #----------------
        # Safety Mechanism
        
        # Mu - sphere intersection
        tSphereVector = np.zeros((2,1))
        if LA.norm(mu_tran_raw) >= sphereEpsilon:
            tSphereVector[0] = (mu_z_raw*mass_total_estimated*G_acc + math.sqrt((mu_z_raw*mass_total_estimated*G_acc)**2 +
                                LA.norm(mu_tran_raw)**2 * (maximumThrust**2 - (mass_total_estimated*G_acc)**2)))/LA.norm(mu_tran_raw)**2
            tSphereVector[1] = (mu_z_raw*mass_total_estimated*G_acc - math.sqrt((mu_z_raw*mass_total_estimated*G_acc)**2 +
                                LA.norm(mu_tran_raw)**2 * (maximumThrust**2 - (mass_total_estimated*G_acc)**2)))/LA.norm(mu_tran_raw)**2
        else:
            tSphereVector[0] = math.nan
            tSphereVector[1] = math.nan
        
        # Mu - elliptic cone intersection
        tEllipticConeVector = np.zeros((2,1))
        if abs(mu_z_raw + math.sqrt((mu_x_raw/math.tan(maximumPitchAngle))**2 +
                                    (mu_y_raw/math.tan(maximumRollAngle))**2)) >= EllipticConeEpsilon:
            tEllipticConeVector[0] = (mass_total_estimated*G_acc)/(mu_z_raw + 
                                      math.sqrt((mu_x_raw/math.tan(maximumPitchAngle))**2 + (mu_y_raw/math.tan(maximumRollAngle))**2))
        else:
            tEllipticConeVector[0] = math.nan
            
        if abs(-mu_z_raw + math.sqrt((mu_x_raw/math.tan(maximumPitchAngle))**2 +
                                    (mu_y_raw/math.tan(maximumRollAngle))**2)) >= EllipticConeEpsilon:
            tEllipticConeVector[1] = -(mass_total_estimated*G_acc)/(-mu_z_raw + 
                                      math.sqrt((mu_x_raw/math.tan(maximumPitchAngle))**2 + (mu_y_raw/math.tan(maximumRollAngle))**2))
        else:
            tEllipticConeVector[1] = math.nan
            
        # Mu - plane intersection
        if abs(mu_z_raw) >= planeEpsilon:
            tPlane = alphaPlane*mass_total_estimated*G_acc/mu_z_raw
        else:
            tPlane = math.nan
            
        tVector = np.array([tSphereVector[0].item(),tSphereVector[1].item(),
                            tEllipticConeVector[0].item(),tEllipticConeVector[1].item(),tPlane])
        for i in range(0,tVector.size):
            if tVector[i] < 0:
                tVector[i] = math.nan
                
        tValue = min(tVector)
        
        if tValue > 1:
            tValue = 1
        tValue = 1 # If this is uncommented, safety mechanism is OFF   
        mu_tran = tValue * mu_tran_raw
        mu_x = mu_tran[0].item()
        mu_y = mu_tran[1].item()
        mu_z = mu_tran[2].item()
        #----------------
        
        u1 = math.sqrt(mu_x ** 2 + mu_y ** 2 + (mass_total_estimated * G_acc - mu_z) ** 2)
        
        calculation_var_A = -(1/u1) * (mu_x * math.sin(yaw_ref) - mu_y * math.cos(yaw_ref))
        roll_ref = math.atan2(calculation_var_A, math.sqrt(1 - calculation_var_A ** 2))
        
        pitch_ref = math.atan2(-(mu_x * math.cos(yaw_ref) + mu_y * math.sin(yaw_ref)), (mass_total_estimated * G_acc - mu_z))
        
        internal_state_differentiator_phi_ref_diff = A_phi_ref * state_phi_ref_diff + B_phi_ref*roll_ref
        internal_state_differentiator_theta_ref_diff = A_theta_ref * state_theta_ref_diff + B_theta_ref*pitch_ref
        
        roll_ref_dot = np.asarray(C_phi_ref*state_phi_ref_diff).item()
        pitch_ref_dot = np.asarray(C_theta_ref*state_theta_ref_diff).item()
        
        roll_ref_ddot = np.asarray(C_phi_ref*internal_state_differentiator_phi_ref_diff).item()
        pitch_ref_ddot = np.asarray(C_theta_ref*internal_state_differentiator_theta_ref_diff).item()
        
        angular_position_ref_dot = np.array([roll_ref_dot, pitch_ref_dot, yaw_ref_dot]).reshape(3,1)
        angular_position_ref_ddot = np.array([roll_ref_ddot, pitch_ref_ddot, yaw_ref_ddot]).reshape(3,1)
        
        
        Jacobian_matrix_inverse = np.matrix([[1, (math.sin(roll)*math.sin(pitch))/math.cos(pitch), (math.cos(roll)*math.sin(pitch))/math.cos(pitch)],
                                              [0,                                   math.cos(roll),                                  -math.sin(roll)],
                                              [0,                   math.sin(roll)/math.cos(pitch),                   math.cos(roll)/math.cos(pitch)]])
    
        angular_position_dot = Jacobian_matrix_inverse * angular_velocity
        angular_error_dot = angular_position_dot - angular_position_ref_dot
        
        Moment = np.cross(angular_velocity.ravel(), (I_matrix_estimated * angular_velocity).ravel()).reshape(3,1) + I_matrix_estimated * (-KP_rot * angular_error - KD_rot * angular_error_dot - KI_rot * integral_angular_error + angular_position_ref_ddot).reshape(3,1)
        u2 = Moment[0].item()
        u3 = Moment[1].item()
        u4 = Moment[2].item()
        
        dy[0:2] = internal_state_differentiator_phi_ref_diff
        dy[2:4] = internal_state_differentiator_theta_ref_diff
        dy[4:7] = translational_position_error
        dy[7:10] = angular_error
        
    
        return np.array(dy)
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    def MRACwithBASELINE(t, y):
        """
        This function defines the system of equations of the MRAC with Baseline CONTROLLER that need to be integrated 
    
        """
        global mu_x, mu_y, mu_z, u1, roll_ref, pitch_ref, roll_ref_dot, pitch_ref_dot, roll_ref_ddot, pitch_ref_ddot
        global angular_position_ref_dot, angular_position_ref_ddot, Jacobian_matrix_inverse, angular_position_dot
        global angular_error_dot, u2, u3, u4, mu_baseline_tran, mu_adaptive_tran, Moment_baseline, Moment_adaptive
        global mu_PD_baseline_tran, Moment_baseline_PI, omega_ref_dot
    
        
        state_phi_ref_diff = y[0:2] # State of the differentiator for phi_ref (roll_ref)
        state_theta_ref_diff = y[2:4] # State of the differentiator for theta_ref (pitch_ref)
        x_ref_tran = y[4:10] # Reference model state
        integral_position_tracking_ref = y[10:13] # Integral of ('translational_position_in_I_ref' - 'translational_position_in_I_user')
        K_hat_x_tran = y[13:31] # \hat{K}_x (translational)
        K_hat_r_tran = y[31:40] # \hat{K}_r (translational)
        Theta_hat_tran = y[40:58] # \hat{\Theta} (translational)
        omega_ref = y[58:61] # Reference model rotational dynamics
        K_hat_x_rot = y[61:70] # \hat{K}_x (rotational)
        K_hat_r_rot = y[70:79] # \hat{K}_r (rotational)
        Theta_hat_rot = y[79:97] # \hat{\Theta} (rotational)
        integral_e_rot = y[97:100] # Integral of 'e_rot' = (angular_velocity - omega_ref)
        
        K_hat_x_tran = np.matrix(K_hat_x_tran.reshape(6,3))
        K_hat_r_tran = np.matrix(K_hat_r_tran.reshape(3,3))
        Theta_hat_tran = np.matrix(Theta_hat_tran.reshape(6,3))
        K_hat_x_rot = np.matrix(K_hat_x_rot.reshape(3,3))
        K_hat_r_rot = np.matrix(K_hat_r_rot.reshape(3,3))
        Theta_hat_rot = np.matrix(Theta_hat_rot.reshape(6,3))
        
        e_tran = x_tran - x_ref_tran
        e_rot = angular_velocity - omega_ref
        translational_position_in_I_ref = x_ref_tran[0:3]
        
        R3 = np.matrix([[math.cos(yaw), -math.sin(yaw), 0],
                        [math.sin(yaw),  math.cos(yaw), 0],
                        [            0,              0, 1]])
        
        R2 = np.matrix([[ math.cos(pitch), 0, math.sin(pitch)],
                        [               0, 1,               0],
                        [-math.sin(pitch), 0, math.cos(pitch)]])
        
        R1 = np.matrix([[1,              0,               0],
                        [0, math.cos(roll), -math.sin(roll)],
                        [0, math.sin(roll),  math.cos(roll)]])
        
        R_from_loc_to_glob = R3*R2*R1
        R_from_glob_to_loc = R_from_loc_to_glob.transpose()
        
        Phi_adaptive_tran = -0.5 * LA.norm(R_from_glob_to_loc * translational_velocity_in_I) * (R_from_glob_to_loc * translational_velocity_in_I)
        
        Jacobian_matrix_inverse = np.matrix([[1, (math.sin(roll)*math.sin(pitch))/math.cos(pitch), (math.cos(roll)*math.sin(pitch))/math.cos(pitch)],
                                              [0,                                   math.cos(roll),                                  -math.sin(roll)],
                                              [0,                   math.sin(roll)/math.cos(pitch),                   math.cos(roll)/math.cos(pitch)]])
    
        angular_position_dot = Jacobian_matrix_inverse * angular_velocity # Time derivative of the Euler angles
        roll_dot = angular_position_dot[0] # phi_dot
        pitch_dot = angular_position_dot[1] # theta_dot
        # yaw_dot = angular_position_dot[2] # psi_dot
        
        Jacobian_matrix_dot = np.matrix(np.zeros((3,3)))
        Jacobian_matrix_dot[0,2] = -math.cos(pitch) * pitch_dot
        Jacobian_matrix_dot[1,1] = -math.sin(roll) * roll_dot
        Jacobian_matrix_dot[1,2] = math.cos(roll) * math.cos(pitch) * roll_dot - math.sin(roll) * math.sin(pitch) * pitch_dot
        Jacobian_matrix_dot[2,1] = -math.cos(roll) * roll_dot
        Jacobian_matrix_dot[2,2] = -math.cos(pitch) * math.sin(roll) * roll_dot - math.cos(roll) * math.sin(pitch) * pitch_dot
        
        r_tran = mass_total_estimated * (-KI_tran*integral_position_tracking_ref + translational_acceleration_in_I_user + KP_tran*translational_position_in_I_user + KD_tran*translational_velocity_in_I_user)

        x_ref_tran_dot = A_ref_tran*x_ref_tran + B_ref_tran*r_tran
        
        mu_PD_baseline_tran = -mass_total_estimated * (KP_tran_PD_baseline * (translational_position_in_I - translational_position_in_I_ref) + KD_tran_PD_baseline * (translational_velocity_in_I - x_ref_tran[3:6]) - x_ref_tran_dot[3:6])
        
        Phi_adaptive_tran_augmented = np.matrix(np.block([[mu_PD_baseline_tran],
                                                          [Phi_adaptive_tran]]))
        Theta_tran_adaptive_bar_augmented = np.matrix(np.block([[np.identity(3)],
                                                          [Theta_tran_adaptive_bar]]))
        
        mu_baseline_tran = K_x_tran_bar.T * x_tran + K_r_tran_bar.T * r_tran - Theta_tran_adaptive_bar_augmented.T * Phi_adaptive_tran_augmented
        mu_adaptive_tran = K_hat_x_tran.T * x_tran + K_hat_r_tran.T * r_tran - Theta_hat_tran.T * Phi_adaptive_tran_augmented
        mu_tran = mu_PD_baseline_tran + mu_baseline_tran + mu_adaptive_tran
        
        K_hat_x_tran_dot = -Gamma_x_tran * x_tran * e_tran.T * P_tran * B_tran
        K_hat_r_tran_dot = -Gamma_r_tran * r_tran * e_tran.T * P_tran * B_tran
        Theta_hat_tran_dot = Gamma_Theta_tran * Phi_adaptive_tran_augmented * e_tran.T * P_tran * B_tran
 
        mu_x = mu_tran[0].item()
        mu_y = mu_tran[1].item()
        mu_z = mu_tran[2].item()
        
        u1 = math.sqrt(mu_x ** 2 + mu_y ** 2 + (mass_total_estimated * G_acc - mu_z) ** 2)
        
        calculation_var_A = -(1/u1) * (mu_x * math.sin(yaw_ref) - mu_y * math.cos(yaw_ref))
        roll_ref = math.atan2(calculation_var_A, math.sqrt(1 - calculation_var_A ** 2))
        
        pitch_ref = math.atan2(-(mu_x * math.cos(yaw_ref) + mu_y * math.sin(yaw_ref)), (mass_total_estimated * G_acc - mu_z))
        
        internal_state_differentiator_phi_ref_diff = A_phi_ref * state_phi_ref_diff + B_phi_ref*roll_ref
        internal_state_differentiator_theta_ref_diff = A_theta_ref * state_theta_ref_diff + B_theta_ref*pitch_ref
        
        roll_ref_dot = np.asarray(C_phi_ref*state_phi_ref_diff).item()
        pitch_ref_dot = np.asarray(C_theta_ref*state_theta_ref_diff).item()
        
        roll_ref_ddot = np.asarray(C_phi_ref*internal_state_differentiator_phi_ref_diff).item()
        pitch_ref_ddot = np.asarray(C_theta_ref*internal_state_differentiator_theta_ref_diff).item()
        
        angular_position_ref_dot = np.array([roll_ref_dot, pitch_ref_dot, yaw_ref_dot]).reshape(3,1)
        angular_position_ref_ddot = np.array([roll_ref_ddot, pitch_ref_ddot, yaw_ref_ddot]).reshape(3,1)
    
        angular_error_dot = angular_position_dot - angular_position_ref_dot
        
        Jacobian_matrix = np.matrix([[1,               0,                 -math.sin(pitch)],
                                      [0,  math.cos(roll), math.sin(roll) * math.cos(pitch)],
                                      [0, -math.sin(roll), math.cos(roll) * math.cos(pitch)]])
        
        omega_cmd = Jacobian_matrix * (-KP_rot*angular_error + angular_position_ref_dot)
        omega_cmd_dot = Jacobian_matrix_dot * (-KP_rot*angular_error + angular_position_ref_dot) + Jacobian_matrix * (-KP_rot*angular_error_dot + angular_position_ref_ddot)
        
        omega_ref_dot = -K_P_omega_ref*(omega_ref - omega_cmd) + omega_cmd_dot
        
        r_rot = K_P_omega_ref * omega_cmd + omega_cmd_dot
        
        Phi_adaptive_rot = np.array([[angular_velocity[1].item() * angular_velocity[2].item()],
                                      [angular_velocity[0].item() * angular_velocity[2].item()],
                                      [angular_velocity[0].item() * angular_velocity[1].item()]])

        Moment_baseline_PI = -I_matrix_estimated * (KP_rot_PI_baseline*e_rot + KI_rot_PI_baseline*integral_e_rot - omega_ref_dot)
        
        Phi_adaptive_rot_augmented = np.matrix(np.block([[Moment_baseline_PI],
                                                          [Phi_adaptive_rot]]))
        
        K_hat_x_rot_dot = -Gamma_x_rot * angular_velocity * e_rot.T * P_rot * B_rot
        K_hat_r_rot_dot = -Gamma_r_rot * r_rot * e_rot.T * P_rot * B_rot
        Theta_hat_rot_dot = Gamma_Theta_rot * Phi_adaptive_rot_augmented * e_rot.T * P_rot * B_rot
        
        Moment_baseline = np.cross(angular_velocity.ravel(), (I_matrix_estimated * angular_velocity).ravel()).reshape(3,1)
        Moment_adaptive = K_hat_x_rot.T * angular_velocity + K_hat_r_rot.T * r_rot - Theta_hat_rot.T * Phi_adaptive_rot_augmented
        
        Moment = Moment_baseline_PI + Moment_baseline + Moment_adaptive
        
        u2 = Moment[0].item()
        u3 = Moment[1].item()
        u4 = Moment[2].item()
        
        
        dy[0:2] = internal_state_differentiator_phi_ref_diff
        dy[2:4] = internal_state_differentiator_theta_ref_diff
        dy[4:10] = x_ref_tran_dot
        dy[10:13] = translational_position_in_I_ref - translational_position_in_I_user
        dy[13:31] = K_hat_x_tran_dot.reshape(18,1)
        dy[31:40] = K_hat_r_tran_dot.reshape(9,1)
        dy[40:58] = Theta_hat_tran_dot.reshape(18,1)
        dy[58:61] = omega_ref_dot
        dy[61:70] = K_hat_x_rot_dot.reshape(9,1)
        dy[70:79] = K_hat_r_rot_dot.reshape(9,1)
        dy[79:97] = Theta_hat_rot_dot.reshape(18,1)
        dy[97:100] = angular_velocity - omega_ref
     
        return np.array(dy)
    
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    































