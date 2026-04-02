from acsl_pychrono.uav.X8 import X8
import numpy as np
import acsl_pychrono.uav as UAV_module

class X8_Controller_Params:
  def __init__(self, uav: X8):
    
    # UAV specific parameters
    cfg = UAV_module.get_uav_config("X8") # Config Dictionary
        
    self.mass_total_estimated = cfg["controller"]["mass_total_estimated"] # [kg]
    
    # Estimated quantities computed from the actual values provided by the UAV class
    self.I_matrix_estimated = np.matrix(uav.Inertia_mat_pixhawk)
    self.surface_area_estimated = uav.surface_area
    self.drag_coefficient_estimated = uav.drag_coefficient
    self.air_density_estimated = uav.air_density
    
    self.drag_coefficient_matrix_estimated = np.matrix(np.diag([self.drag_coefficient_estimated,self.drag_coefficient_estimated,0]))

    # Roll filters gains (from matlab)
    self.A_phi_ref = np.matrix(cfg["controller"]["gains"]["A_phi_ref"])
    self.B_phi_ref = np.array(cfg["controller"]["gains"]["B_phi_ref"])
    self.C_phi_ref = np.matrix(cfg["controller"]["gains"]["C_phi_ref"])
    self.D_phi_ref = cfg["controller"]["gains"]["D_phi_ref"]
    # Pitch filters gains (from matlab)
    self.A_theta_ref = np.matrix(cfg["controller"]["gains"]["A_theta_ref"])
    self.B_theta_ref = np.array(cfg["controller"]["gains"]["B_theta_ref"])
    self.C_theta_ref = np.matrix(cfg["controller"]["gains"]["C_theta_ref"])
    self.D_theta_ref = cfg["controller"]["gains"]["D_theta_ref"]
    
    self.K_omega = cfg["controller"]["K_omega"] # Propellers'thrust to square of the angular velocity
    self.K_torque = cfg["controller"]["K_torque"] # Propellers'torque to square of the angular velocity
    
    self.PID_config_file = cfg["controller"]["PID"]["config_file"]
    self.MRAC_config_file = cfg["controller"]["MRAC"]["config_file"]
    self.TwoLayerMRAC_config_file = cfg["controller"]["TwoLayerMRAC"]["config_file"]
  #   self.init_PID_Gains(cfg)
    
  # # PID_Controller Parameters
  # def init_PID_Gains(self, cfg):
    
  #   # Number of states to be integrated by RK4
  #   self.PID_number_of_states = cfg["controller"]["PID"]["number_of_states"]
  #   # Length of the array vector that will be exported 
  #   self.PID_size_DATA = cfg["controller"]["PID"]["size_DATA"]
    
  #   # **Translational** PID parameters 
  #   self.PID_KP_tran = cfg["controller"]["PID"]["KP_tran"]
  #   self.PID_KD_tran = cfg["controller"]["PID"]["KD_tran"]
  #   self.PID_KI_tran = cfg["controller"]["PID"]["KI_tran"]

  #   # **Rotational** PID parameters
  #   self.PID_KP_rot = cfg["controller"]["PID"]["KP_rot"]
  #   self.PID_KD_rot = cfg["controller"]["PID"]["KD_rot"]
  #   self.PID_KI_rot = cfg["controller"]["PID"]["KI_rot"]
    
  #   # ----------------------------------------------------------------
  #   #                   Safety Mechanism Parameters
  #   # ----------------------------------------------------------------
  #   self.PID_use_safety_mechanism = cfg["controller"]["PID"]["use_safety_mechanism"]
    
  #   # Mu - sphere intersection
  #   self.PID_sphereEpsilon = cfg["controller"]["PID"]["sphereEpsilon"]
  #   self.PID_maximumThrust = cfg["controller"]["PID"]["maximumThrust"] # [N]
    
  #   # Mu - elliptic cone intersection
  #   self.PID_EllipticConeEpsilon = cfg["controller"]["PID"]["EllipticConeEpsilon"]
  #   self.PID_maximumRollAngle = cfg["controller"]["PID"]["maximumRollAngle"] # [rad]
  #   self.PID_maximumPitchAngle = cfg["controller"]["PID"]["maximumPitchAngle"] # [rad]
    
  #   # Mu - plane intersection
  #   self.PID_planeEpsilon = cfg["controller"]["PID"]["planeEpsilon"]
  #   self.PID_alphaPlane = cfg["controller"]["PID"]["alphaPlane"] # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
    