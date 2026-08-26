import numpy as np

from acsl_pychrono.control.rkhs.feature_map import RKHSRegressorConfig, build_rkhs_feature_maps


class RKHSRegressorMixin:
  def initialize_rkhs_regressors(self, config=None):
    self.rkhs_config = config or getattr(self.gains, "rkhs_config", RKHSRegressorConfig())
    self.rkhs_tran, self.rkhs_rot = build_rkhs_feature_maps(self.rkhs_config)

  def computeRegressorVectorOuterLoop(self):
    from acsl_pychrono.control.control import Control

    _, R_from_glob_to_loc = Control.computeRotationMatrices(
      self.odein.roll,
      self.odein.pitch,
      self.odein.yaw,
    )
    translational_velocity_in_J = R_from_glob_to_loc * self.odein.translational_velocity_in_I
    self.Phi_adaptive_tran = self.rkhs_tran.phi(translational_velocity_in_J)
    return np.matrix(np.block([[self.mu_PD_baseline_tran], [self.Phi_adaptive_tran]]))

  def computeRegressorVectorInnerLoop(self):
    Phi_adaptive_rot = self.rkhs_rot.phi(self.odein.angular_velocity)
    Phi_adaptive_rot_augmented = np.matrix(np.block([[self.Moment_baseline_PI], [Phi_adaptive_rot]]))
    return Phi_adaptive_rot, Phi_adaptive_rot_augmented
