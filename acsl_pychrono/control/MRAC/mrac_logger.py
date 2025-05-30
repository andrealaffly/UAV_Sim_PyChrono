import numpy as np  
from acsl_pychrono.control.MRAC.mrac_gains import MRACGains
from acsl_pychrono.control.MRAC.mrac import MRAC

class MRACLogger:
  def __init__(self, gains: MRACGains) -> None:
    self.gains = gains
    self.data_list = []

  def collectData(self, controller: MRAC, simulation_time: float):
    DATA_vector = np.zeros((self.gains.size_DATA, 1))

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
    DATA_vector[48:56] = controller.motor_thrusts.reshape(8,1)
    DATA_vector[56:59] = controller.mu_baseline_tran
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
        "r_cmd": {
          "x": DATA_np[:, 86].reshape(-1, 1),
          "y": DATA_np[:, 87].reshape(-1, 1),
          "z": DATA_np[:, 88].reshape(-1, 1),
        }
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
        }
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