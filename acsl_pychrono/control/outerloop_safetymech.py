import numpy as np
import math
from numpy import linalg as LA

class OuterLoopSafetyMechanism:
  """
  Safety mechanism that acts between the outer and inner loop. It allows constraints on:
  - Maximum desired pitch and roll angles
  - Maximum total thrust
  - Prevention of free-fall maneuvers
  """

  def __init__(self, gains, G_acc):
    """
    Initialize the safety mechanism.
    """
    self.gains = gains
    self.G_acc = G_acc
    self.use_safety_mechanism = gains.use_safety_mechanism

  def apply(self, mu_tran_raw):
    """
    Apply the safety mechanism to a raw mu_tran vector.
    """
    mu_x_raw = mu_tran_raw[0].item()
    mu_y_raw = mu_tran_raw[1].item()
    mu_z_raw = mu_tran_raw[2].item()

    if not self.use_safety_mechanism:
      return mu_x_raw, mu_y_raw, mu_z_raw
    
    else:
      # Mu - sphere intersection
      tSphereVector = np.zeros((2,1))
      if LA.norm(mu_tran_raw) >= self.gains.sphereEpsilon:
        tSphereVector[0] = (mu_z_raw*self.gains.mass_total_estimated*self.G_acc + math.sqrt((mu_z_raw*self.gains.mass_total_estimated*self.G_acc)**2 +
                            LA.norm(mu_tran_raw)**2 * (self.gains.maximumThrust**2 - (self.gains.mass_total_estimated*self.G_acc)**2)))/LA.norm(mu_tran_raw)**2
        tSphereVector[1] = (mu_z_raw*self.gains.mass_total_estimated*self.G_acc - math.sqrt((mu_z_raw*self.gains.mass_total_estimated*self.G_acc)**2 +
                            LA.norm(mu_tran_raw)**2 * (self.gains.maximumThrust**2 - (self.gains.mass_total_estimated*self.G_acc)**2)))/LA.norm(mu_tran_raw)**2
      else:
        tSphereVector[0] = math.nan
        tSphereVector[1] = math.nan
      
      # Mu - elliptic cone intersection
      tEllipticConeVector = np.zeros((2,1))
      if abs(mu_z_raw + math.sqrt((mu_x_raw/math.tan(self.gains.maximumPitchAngle))**2 +
                                  (mu_y_raw/math.tan(self.gains.maximumRollAngle))**2)) >= self.gains.EllipticConeEpsilon:
        tEllipticConeVector[0] = (self.gains.mass_total_estimated*self.G_acc)/(mu_z_raw + 
                                  math.sqrt((mu_x_raw/math.tan(self.gains.maximumPitchAngle))**2 + (mu_y_raw/math.tan(self.gains.maximumRollAngle))**2))
      else:
        tEllipticConeVector[0] = math.nan
          
      if abs(-mu_z_raw + math.sqrt((mu_x_raw/math.tan(self.gains.maximumPitchAngle))**2 +
                                  (mu_y_raw/math.tan(self.gains.maximumRollAngle))**2)) >= self.gains.EllipticConeEpsilon:
        tEllipticConeVector[1] = -(self.gains.mass_total_estimated*self.G_acc)/(-mu_z_raw + 
                                  math.sqrt((mu_x_raw/math.tan(self.gains.maximumPitchAngle))**2 + (mu_y_raw/math.tan(self.gains.maximumRollAngle))**2))
      else:
        tEllipticConeVector[1] = math.nan
          
      # Mu - plane intersection
      if abs(mu_z_raw) >= self.gains.planeEpsilon:
        tPlane = self.gains.alphaPlane*self.gains.mass_total_estimated*self.G_acc/mu_z_raw
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

      mu_tran = tValue * mu_tran_raw
      mu_x = mu_tran[0].item()
      mu_y = mu_tran[1].item()
      mu_z = mu_tran[2].item()

      return mu_x, mu_y, mu_z
