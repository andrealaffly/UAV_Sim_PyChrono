import math
import numpy as np
from numpy import linalg as LA
import scipy
from scipy import linalg
from acsl_pychrono.flight_params import FlightParams

class PIDGains:
  def __init__(self, flight_params: FlightParams):
    # General vehicle properties
    self.I_matrix_estimated = flight_params.I_matrix_estimated
    self.mass_total_estimated = flight_params.mass_total_estimated
    self.air_density_estimated = flight_params.air_density_estimated
    self.surface_area_estimated = flight_params.surface_area_estimated
    self.drag_coefficient_matrix_estimated = flight_params.drag_coefficient_matrix_estimated

    # Number of states to be integrated by RK4
    self.number_of_states = 10
    # Length of the array vector that will be exported 
    self.size_DATA = 50
    
    # **Translational** PID parameters 
    self.KP_tran = np.matrix(1 * np.diag([5,5,6]))
    self.KD_tran = np.matrix(1 * np.diag([8,8,3]))
    self.KI_tran = np.matrix(1 * np.diag([1,1,1]))

    # **Rotational** PID parameters
    self.KP_rot = np.matrix(1 * np.diag([100,100,50]))
    self.KD_rot = np.matrix(1 * np.diag([50,50,50]))
    self.KI_rot = np.matrix(1 * np.diag([20,20,10]))
    
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------
    self.use_safety_mechanism = False
    
    # Mu - sphere intersection
    self.sphereEpsilon = 1e-2
    self.maximumThrust = 85 # [N] 85
    
    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = 1e-2
    self.maximumRollAngle = math.radians(32) # [rad] 25
    self.maximumPitchAngle = math.radians(32) # [rad] 25
    
    # Mu - plane intersection
    self.planeEpsilon = 1e-2
    self.alphaPlane = 0.95 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
