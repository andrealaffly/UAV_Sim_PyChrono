import numpy as np  
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.control.FunnelTwoLayerMRAC.funnel_two_layer_mrac_gains import FunnelTwoLayerMRACGains
from acsl_pychrono.control.FunnelTwoLayerMRAC.funnel_two_layer_mrac import FunnelTwoLayerMRAC

class FunnelTwoLayerMRACLogger:
  def __init__(self, gains: FunnelTwoLayerMRACGains) -> None:
    self.gains = gains
    self.data_list = []
    # Length of the array vector that will be exported 
    self.size_DATA = 238

  def collectData(self, controller: FunnelTwoLayerMRAC, simulation_time: float):
    DATA_vector = np.zeros((self.size_DATA, 1))

    # Pad the motor thrusts with zeros if fewer than 8 propellers
    motor_thrusts = Logging.dataVectorPadding(controller.motor_thrusts, 8)

    DATA_vector[0] = controller.odein.time_now
    DATA_vector[1] = simulation_time
    DATA_vector[2:5] = controller.odein.translational_position_in_I
    DATA_vector[5:8] = controller.odein.translational_velocity_in_I
    DATA_vector[8] = controller.odein.roll
    DATA_vector[9] = controller.odein.pitch
    DATA_vector[10] = controller.odein.yaw
    DATA_vector[11:14] = controller.odein.angular_velocity
    DATA_vector[14:20] = controller.x_ref_tran 
    DATA_vector[20] = controller.roll_ref
    DATA_vector[21] = controller.pitch_ref
    DATA_vector[22] = controller.odein.yaw_ref
    DATA_vector[23] = controller.angular_position_ref_dot[0]
    DATA_vector[24] = controller.angular_position_ref_dot[1]
    DATA_vector[25] = controller.angular_position_ref_dot[2]
    DATA_vector[26] = controller.angular_position_ref_ddot[0]
    DATA_vector[27] = controller.angular_position_ref_ddot[1]
    DATA_vector[28] = controller.angular_position_ref_ddot[2]
    DATA_vector[29:32] = controller.omega_ref
    DATA_vector[32:35] = controller.odein.translational_position_in_I_user
    DATA_vector[35:38] = controller.odein.translational_velocity_in_I_user
    DATA_vector[38:41] = controller.odein.translational_acceleration_in_I_user
    DATA_vector[41] = controller.mu_x
    DATA_vector[42] = controller.mu_y
    DATA_vector[43] = controller.mu_z
    DATA_vector[44] = controller.u1
    DATA_vector[45] = controller.u2
    DATA_vector[46] = controller.u3
    DATA_vector[47] = controller.u4
    DATA_vector[48:56] = motor_thrusts.reshape(8,1)
    DATA_vector[56:59] = np.zeros((3, 1))
    DATA_vector[59:62] = controller.mu_adaptive_tran
    DATA_vector[62:65] = controller.mu_PD_baseline_tran
    DATA_vector[65:68] = controller.Moment_baseline
    DATA_vector[68:71] = controller.Moment_adaptive
    DATA_vector[71:74] = controller.Moment_baseline_PI
    DATA_vector[74:77] = controller.angular_position_dot
    DATA_vector[77:80] = controller.omega_cmd
    DATA_vector[80:83] = controller.omega_cmd_dot
    DATA_vector[83:86] = controller.omega_ref_dot
    DATA_vector[86:89] = controller.r_tran
    DATA_vector[89:92] = controller.r_rot

    if self.gains.use_projection_operator:
      DATA_vector[92] = controller.proj_op_activated_K_hat_x_tran
      DATA_vector[93] = controller.proj_op_activated_K_hat_r_tran
      DATA_vector[94] = controller.proj_op_activated_Theta_hat_tran
      DATA_vector[95] = controller.proj_op_activated_K_hat_x_rot
      DATA_vector[96] = controller.proj_op_activated_K_hat_r_rot
      DATA_vector[97] = controller.proj_op_activated_Theta_hat_rot
    else:
      DATA_vector[92:98] = False

    DATA_vector[98:116] = controller.K_hat_x_tran.flatten(order='F').reshape(-1, 1)
    DATA_vector[116:125] = controller.K_hat_r_tran.flatten(order='F').reshape(-1, 1)
    DATA_vector[125:143] = controller.Theta_hat_tran.flatten(order='F').reshape(-1, 1)

    DATA_vector[143:152] = controller.K_hat_x_rot.flatten(order='F').reshape(-1, 1)
    DATA_vector[152:161] = controller.K_hat_r_rot.flatten(order='F').reshape(-1, 1)
    DATA_vector[161:179] = controller.Theta_hat_rot.flatten(order='F').reshape(-1, 1)

    DATA_vector[179] = controller.dead_zone_value_tran
    DATA_vector[180] = controller.dead_zone_value_rot

    if self.gains.use_projection_operator:
      DATA_vector[181] = controller.proj_op_activated_K_hat_g_tran
      DATA_vector[182] = controller.proj_op_activated_K_hat_g_rot
    else:
      DATA_vector[181:183] = False

    DATA_vector[183:201] = controller.K_hat_g_tran.flatten(order='F').reshape(-1, 1)
    DATA_vector[201:210] = controller.K_hat_g_rot.flatten(order='F').reshape(-1, 1)

    DATA_vector[210:213] = controller.mu_adaptive_mrac_tran
    DATA_vector[213:216] = controller.mu_adaptive_ebci_tran
    DATA_vector[216:219] = controller.Moment_adaptive_mrac
    DATA_vector[219:222] = controller.Moment_adaptive_ebci

    DATA_vector[222] = controller.eta_funnel_tran
    DATA_vector[223] = controller.eta_dot_funnel_tran
    DATA_vector[224] = controller.xi_funnel_tran

    if not self.gains.use_funnel_romoco_old_tran:
      DATA_vector[225] = controller.lambda_sat_funnel_tran
      DATA_vector[226] = controller.sigma_nom_funnel_tran
      DATA_vector[227] = controller.sigma_ideal_funnel_tran
    else:
      DATA_vector[225:228] = np.nan

    DATA_vector[228] = controller.H_function_funnel_tran
    DATA_vector[229] = controller.Ve_funnel_tran
    DATA_vector[230] = controller.eT_M_e_tran
    DATA_vector[231] = controller.diameter_funnel_tran
    DATA_vector[232:238] = controller.e_tran_dot
    
    self.data_list.append(DATA_vector.flatten())

  def toDictionary(self):
    DATA_np = np.array(self.data_list)

    log_dict = {
      "time": DATA_np[:, 0].reshape(-1, 1),
      "position": {
        "x": DATA_np[:, 2].reshape(-1, 1),
        "y": DATA_np[:, 3].reshape(-1, 1),
        "z": DATA_np[:, 4].reshape(-1, 1),
      },
      "velocity": {
        "x": DATA_np[:, 5].reshape(-1, 1),
        "y": DATA_np[:, 6].reshape(-1, 1),
        "z": DATA_np[:, 7].reshape(-1, 1),
      },
      "euler_angles": {
        "roll": DATA_np[:, 8].reshape(-1, 1),
        "pitch": DATA_np[:, 9].reshape(-1, 1),
        "yaw": DATA_np[:, 10].reshape(-1, 1),
      },
      "angular_velocity": {
        "x": DATA_np[:, 11].reshape(-1, 1),
        "y": DATA_np[:, 12].reshape(-1, 1),
        "z": DATA_np[:, 13].reshape(-1, 1),
      },
      "outer_loop": {
        "reference_model": {
          "position": {
            "x": DATA_np[:, 14].reshape(-1, 1),
            "y": DATA_np[:, 15].reshape(-1, 1),
            "z": DATA_np[:, 16].reshape(-1, 1),
          },
          "velocity": {
            "x": DATA_np[:, 17].reshape(-1, 1),
            "y": DATA_np[:, 18].reshape(-1, 1),
            "z": DATA_np[:, 19].reshape(-1, 1),
          }
        },
        "mu_adaptive": {
          "x": DATA_np[:, 59].reshape(-1, 1),
          "y": DATA_np[:, 60].reshape(-1, 1),
          "z": DATA_np[:, 61].reshape(-1, 1),
        },
        "mu_PID_baseline": {
          "x": DATA_np[:, 62].reshape(-1, 1),
          "y": DATA_np[:, 63].reshape(-1, 1),
          "z": DATA_np[:, 64].reshape(-1, 1),
        },
        "funnel": {
          "eta": DATA_np[:, 222].reshape(-1, 1),
          "eta_dot": DATA_np[:, 223].reshape(-1, 1),
          "xi": DATA_np[:, 224].reshape(-1, 1),
          "lambda_sat": DATA_np[:, 225].reshape(-1, 1),
          "sigma_nom": DATA_np[:, 226].reshape(-1, 1),
          "sigma_ideal": DATA_np[:, 227].reshape(-1, 1),
          "H_function": DATA_np[:, 228].reshape(-1, 1),
          "Ve_function": DATA_np[:, 229].reshape(-1, 1),
          "eT_M_e": DATA_np[:, 230].reshape(-1, 1),
          "diameter": DATA_np[:, 231].reshape(-1, 1),
        },
        "r_cmd": {
          "x": DATA_np[:, 86].reshape(-1, 1),
          "y": DATA_np[:, 87].reshape(-1, 1),
          "z": DATA_np[:, 88].reshape(-1, 1),
        },
        "e_tran_dot": {
          "x": DATA_np[:, 232].reshape(-1, 1),
          "y": DATA_np[:, 233].reshape(-1, 1),
          "z": DATA_np[:, 234].reshape(-1, 1),
          "vx": DATA_np[:, 235].reshape(-1, 1),
          "vy": DATA_np[:, 236].reshape(-1, 1),
          "vz": DATA_np[:, 237].reshape(-1, 1),
        },
        "proj_op_activated_K_hat_x": DATA_np[:, 92].reshape(-1, 1),
        "proj_op_activated_K_hat_r": DATA_np[:, 93].reshape(-1, 1),
        "proj_op_activated_Theta_hat": DATA_np[:, 94].reshape(-1, 1),
        "K_hat_x": {
          "ind00": DATA_np[:, 98].reshape(-1, 1),
          "ind10": DATA_np[:, 99].reshape(-1, 1),
          "ind20": DATA_np[:, 100].reshape(-1, 1),
          "ind30": DATA_np[:, 101].reshape(-1, 1),
          "ind40": DATA_np[:, 102].reshape(-1, 1),
          "ind50": DATA_np[:, 103].reshape(-1, 1),
          "ind01": DATA_np[:, 104].reshape(-1, 1),
          "ind11": DATA_np[:, 105].reshape(-1, 1),
          "ind21": DATA_np[:, 106].reshape(-1, 1),
          "ind31": DATA_np[:, 107].reshape(-1, 1),
          "ind41": DATA_np[:, 108].reshape(-1, 1),
          "ind51": DATA_np[:, 109].reshape(-1, 1),
          "ind02": DATA_np[:, 110].reshape(-1, 1),
          "ind12": DATA_np[:, 111].reshape(-1, 1),
          "ind22": DATA_np[:, 112].reshape(-1, 1),
          "ind32": DATA_np[:, 113].reshape(-1, 1),
          "ind42": DATA_np[:, 114].reshape(-1, 1),
          "ind52": DATA_np[:, 115].reshape(-1, 1),
        },
        "K_hat_r": {
          "ind00": DATA_np[:, 116].reshape(-1, 1),
          "ind10": DATA_np[:, 117].reshape(-1, 1),
          "ind20": DATA_np[:, 118].reshape(-1, 1),
          "ind01": DATA_np[:, 119].reshape(-1, 1),
          "ind11": DATA_np[:, 120].reshape(-1, 1),
          "ind21": DATA_np[:, 121].reshape(-1, 1),
          "ind02": DATA_np[:, 122].reshape(-1, 1),
          "ind12": DATA_np[:, 123].reshape(-1, 1),
          "ind22": DATA_np[:, 124].reshape(-1, 1),
        },
        "Theta_hat": {
          "ind00": DATA_np[:, 125].reshape(-1, 1),
          "ind10": DATA_np[:, 126].reshape(-1, 1),
          "ind20": DATA_np[:, 127].reshape(-1, 1),
          "ind30": DATA_np[:, 128].reshape(-1, 1),
          "ind40": DATA_np[:, 129].reshape(-1, 1),
          "ind50": DATA_np[:, 130].reshape(-1, 1),
          "ind01": DATA_np[:, 131].reshape(-1, 1),
          "ind11": DATA_np[:, 132].reshape(-1, 1),
          "ind21": DATA_np[:, 133].reshape(-1, 1),
          "ind31": DATA_np[:, 134].reshape(-1, 1),
          "ind41": DATA_np[:, 135].reshape(-1, 1),
          "ind51": DATA_np[:, 136].reshape(-1, 1),
          "ind02": DATA_np[:, 137].reshape(-1, 1),
          "ind12": DATA_np[:, 138].reshape(-1, 1),
          "ind22": DATA_np[:, 139].reshape(-1, 1),
          "ind32": DATA_np[:, 140].reshape(-1, 1),
          "ind42": DATA_np[:, 141].reshape(-1, 1),
          "ind52": DATA_np[:, 142].reshape(-1, 1),
        },
        "dead_zone_value": DATA_np[:, 179].reshape(-1, 1),
        "proj_op_activated_K_hat_g": DATA_np[:, 181].reshape(-1, 1),
        "K_hat_g": {
          "ind00": DATA_np[:, 183].reshape(-1, 1),
          "ind10": DATA_np[:, 184].reshape(-1, 1),
          "ind20": DATA_np[:, 185].reshape(-1, 1),
          "ind30": DATA_np[:, 186].reshape(-1, 1),
          "ind40": DATA_np[:, 187].reshape(-1, 1),
          "ind50": DATA_np[:, 188].reshape(-1, 1),
          "ind01": DATA_np[:, 189].reshape(-1, 1),
          "ind11": DATA_np[:, 190].reshape(-1, 1),
          "ind21": DATA_np[:, 191].reshape(-1, 1),
          "ind31": DATA_np[:, 192].reshape(-1, 1),
          "ind41": DATA_np[:, 193].reshape(-1, 1),
          "ind51": DATA_np[:, 194].reshape(-1, 1),
          "ind02": DATA_np[:, 195].reshape(-1, 1),
          "ind12": DATA_np[:, 196].reshape(-1, 1),
          "ind22": DATA_np[:, 197].reshape(-1, 1),
          "ind32": DATA_np[:, 198].reshape(-1, 1),
          "ind42": DATA_np[:, 199].reshape(-1, 1),
          "ind52": DATA_np[:, 200].reshape(-1, 1),
        },
        "mu_adaptive_mrac": {
          "x": DATA_np[:, 210].reshape(-1, 1),
          "y": DATA_np[:, 211].reshape(-1, 1),
          "z": DATA_np[:, 212].reshape(-1, 1),
        },
        "mu_adaptive_ebci": {
          "x": DATA_np[:, 213].reshape(-1, 1),
          "y": DATA_np[:, 214].reshape(-1, 1),
          "z": DATA_np[:, 215].reshape(-1, 1),
        },
      },
      "desired_euler_angles": {
        "roll": DATA_np[:, 20].reshape(-1, 1),
        "pitch": DATA_np[:, 21].reshape(-1, 1),
        "roll_dot": DATA_np[:, 23].reshape(-1, 1),
        "pitch_dot": DATA_np[:, 24].reshape(-1, 1),
        "roll_dot_dot": DATA_np[:, 26].reshape(-1, 1),
        "pitch_dot_dot": DATA_np[:, 27].reshape(-1, 1),
      },
      "user_defined_yaw": DATA_np[:, 22].reshape(-1, 1),
      "user_defined_yaw_dot": DATA_np[:, 25].reshape(-1, 1),
      "user_defined_yaw_dot_dot": DATA_np[:, 28].reshape(-1, 1),
      "inner_loop": {
        "reference_model": {
          "angular_velocity": {
            "x": DATA_np[:, 29].reshape(-1, 1),
            "y": DATA_np[:, 30].reshape(-1, 1),
            "z": DATA_np[:, 31].reshape(-1, 1),
          }
        },
        "tau_adaptive": {
          "x": DATA_np[:, 68].reshape(-1, 1),
          "y": DATA_np[:, 69].reshape(-1, 1),
          "z": DATA_np[:, 70].reshape(-1, 1),
        },
        "tau_PID_baseline": {
          "x": DATA_np[:, 71].reshape(-1, 1),
          "y": DATA_np[:, 72].reshape(-1, 1),
          "z": DATA_np[:, 73].reshape(-1, 1),
        },
        "omega_cmd": {
          "x": DATA_np[:, 77].reshape(-1, 1),
          "y": DATA_np[:, 78].reshape(-1, 1),
          "z": DATA_np[:, 79].reshape(-1, 1),
        },
        "omega_cmd_dot": {
          "x": DATA_np[:, 80].reshape(-1, 1),
          "y": DATA_np[:, 81].reshape(-1, 1),
          "z": DATA_np[:, 82].reshape(-1, 1),
        },
        "omega_ref_dot": {
          "x": DATA_np[:, 83].reshape(-1, 1),
          "y": DATA_np[:, 84].reshape(-1, 1),
          "z": DATA_np[:, 85].reshape(-1, 1),
        },
        "r_cmd": {
          "x": DATA_np[:, 89].reshape(-1, 1),
          "y": DATA_np[:, 90].reshape(-1, 1),
          "z": DATA_np[:, 91].reshape(-1, 1),
        },
        "proj_op_activated_K_hat_x": DATA_np[:, 95].reshape(-1, 1),
        "proj_op_activated_K_hat_r": DATA_np[:, 96].reshape(-1, 1),
        "proj_op_activated_Theta_hat": DATA_np[:, 97].reshape(-1, 1),
        "K_hat_x": {
          "ind00": DATA_np[:, 143].reshape(-1, 1),
          "ind10": DATA_np[:, 144].reshape(-1, 1),
          "ind20": DATA_np[:, 145].reshape(-1, 1),
          "ind01": DATA_np[:, 146].reshape(-1, 1),
          "ind11": DATA_np[:, 147].reshape(-1, 1),
          "ind21": DATA_np[:, 148].reshape(-1, 1),
          "ind02": DATA_np[:, 149].reshape(-1, 1),
          "ind12": DATA_np[:, 150].reshape(-1, 1),
          "ind22": DATA_np[:, 151].reshape(-1, 1),
        },
        "K_hat_r": {
          "ind00": DATA_np[:, 152].reshape(-1, 1),
          "ind10": DATA_np[:, 153].reshape(-1, 1),
          "ind20": DATA_np[:, 154].reshape(-1, 1),
          "ind01": DATA_np[:, 155].reshape(-1, 1),
          "ind11": DATA_np[:, 156].reshape(-1, 1),
          "ind21": DATA_np[:, 157].reshape(-1, 1),
          "ind02": DATA_np[:, 158].reshape(-1, 1),
          "ind12": DATA_np[:, 159].reshape(-1, 1),
          "ind22": DATA_np[:, 160].reshape(-1, 1),
        },
        "Theta_hat": {
          "ind00": DATA_np[:, 161].reshape(-1, 1),
          "ind10": DATA_np[:, 162].reshape(-1, 1),
          "ind20": DATA_np[:, 163].reshape(-1, 1),
          "ind30": DATA_np[:, 164].reshape(-1, 1),
          "ind40": DATA_np[:, 165].reshape(-1, 1),
          "ind50": DATA_np[:, 166].reshape(-1, 1),
          "ind01": DATA_np[:, 167].reshape(-1, 1),
          "ind11": DATA_np[:, 168].reshape(-1, 1),
          "ind21": DATA_np[:, 169].reshape(-1, 1),
          "ind31": DATA_np[:, 170].reshape(-1, 1),
          "ind41": DATA_np[:, 171].reshape(-1, 1),
          "ind51": DATA_np[:, 172].reshape(-1, 1),
          "ind02": DATA_np[:, 173].reshape(-1, 1),
          "ind12": DATA_np[:, 174].reshape(-1, 1),
          "ind22": DATA_np[:, 175].reshape(-1, 1),
          "ind32": DATA_np[:, 176].reshape(-1, 1),
          "ind42": DATA_np[:, 177].reshape(-1, 1),
          "ind52": DATA_np[:, 178].reshape(-1, 1),
        },
        "dead_zone_value": DATA_np[:, 180].reshape(-1, 1),
        "proj_op_activated_K_hat_g": DATA_np[:, 182].reshape(-1, 1),
        "K_hat_g": {
          "ind00": DATA_np[:, 201].reshape(-1, 1),
          "ind10": DATA_np[:, 202].reshape(-1, 1),
          "ind20": DATA_np[:, 203].reshape(-1, 1),
          "ind01": DATA_np[:, 204].reshape(-1, 1),
          "ind11": DATA_np[:, 205].reshape(-1, 1),
          "ind21": DATA_np[:, 206].reshape(-1, 1),
          "ind02": DATA_np[:, 207].reshape(-1, 1),
          "ind12": DATA_np[:, 208].reshape(-1, 1),
          "ind22": DATA_np[:, 209].reshape(-1, 1),
        },
        "Moment_adaptive_mrac": {
          "x": DATA_np[:, 216].reshape(-1, 1),
          "y": DATA_np[:, 217].reshape(-1, 1),
          "z": DATA_np[:, 218].reshape(-1, 1),
        },
        "Moment_adaptive_ebci": {
          "x": DATA_np[:, 219].reshape(-1, 1),
          "y": DATA_np[:, 220].reshape(-1, 1),
          "z": DATA_np[:, 221].reshape(-1, 1),
        },
      },
      "user_defined_position": {
        "x": DATA_np[:, 32].reshape(-1, 1),
        "y": DATA_np[:, 33].reshape(-1, 1),
        "z": DATA_np[:, 34].reshape(-1, 1),
      },
      "user_defined_velocity": {
        "x": DATA_np[:, 35].reshape(-1, 1),
        "y": DATA_np[:, 36].reshape(-1, 1),
        "z": DATA_np[:, 37].reshape(-1, 1),
      },
      "user_defined_acceleration": {
        "x": DATA_np[:, 38].reshape(-1, 1),
        "y": DATA_np[:, 39].reshape(-1, 1),
        "z": DATA_np[:, 40].reshape(-1, 1),
      },
      "mu_translational": {
        "x": DATA_np[:, 41].reshape(-1, 1),
        "y": DATA_np[:, 42].reshape(-1, 1),
        "z": DATA_np[:, 43].reshape(-1, 1),
      },
      "control_input": {
        "U1": DATA_np[:, 44].reshape(-1, 1),
        "U2": DATA_np[:, 45].reshape(-1, 1),
        "U3": DATA_np[:, 46].reshape(-1, 1),
        "U4": DATA_np[:, 47].reshape(-1, 1),
      },
      "thrust_motors_N": {
        "T1": DATA_np[:, 48].reshape(-1, 1),
        "T2": DATA_np[:, 49].reshape(-1, 1),
        "T3": DATA_np[:, 50].reshape(-1, 1),
        "T4": DATA_np[:, 51].reshape(-1, 1),
        "T5": DATA_np[:, 52].reshape(-1, 1),
        "T6": DATA_np[:, 53].reshape(-1, 1),
        "T7": DATA_np[:, 54].reshape(-1, 1),
        "T8": DATA_np[:, 55].reshape(-1, 1),
      },
      "euler_angles_dot": {
        "roll_dot": DATA_np[:, 74].reshape(-1, 1),
        "pitch_dot": DATA_np[:, 75].reshape(-1, 1),
        "yaw_dot": DATA_np[:, 76].reshape(-1, 1),
      },
      "omega_cmd": {
        "x": DATA_np[:, 77].reshape(-1, 1),
        "y": DATA_np[:, 78].reshape(-1, 1),
        "z": DATA_np[:, 79].reshape(-1, 1),
      },
      "omega_cmd_dot": {
        "x": DATA_np[:, 77].reshape(-1, 1),
        "y": DATA_np[:, 78].reshape(-1, 1),
        "z": DATA_np[:, 79].reshape(-1, 1),
      }
    }

    return log_dict