import numpy as np
from dataclasses import dataclass, field
import pychrono as chrono

@dataclass
class PixhawkState:
  # Global coordinates of the pixhawk containing both its position and rotation (quaternion)
  coord_GLOB: chrono.ChCoordsysD = field(default_factory=chrono.ChCoordsysD)
  # Global velocities of the pixhawk derived from its position and rotation (quaternion)
  coord_dt_GLOB: chrono.ChCoordsysD = field(default_factory=chrono.ChCoordsysD)
  # Global accelerations of the pixhawk derived from its position and rotation (quaternion)
  coord_dtdt_GLOB: chrono.ChCoordsysD = field(default_factory=chrono.ChCoordsysD)
  # Angular velocity of the pixhawk respect to global coordinates, expressed in global coordinates
  Wvel_GLOB: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Angular acceleration of the pixhawk respect to global coordinates, expressed in global coordinates
  Wacc_GLOB: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Rotation matrix of the pixhawk given by pychrono (computed using pixhawk quaternion expressed in glob coord)
  rotmat: chrono.ChMatrix33D = field(default_factory=chrono.ChMatrix33D)
  # Rotation matrix of the pixhawk to go from Global to Local coordinates
  # (computed using Mattia's function: fun.rotmat_fromQ_Glob_to_Loc_asChMatrix33)
  rotmat_F: chrono.ChMatrix33D = field(default_factory=chrono.ChMatrix33D)
  # Local position of the pixhawk
  pos_LOC: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Local velocities of the pixhawk
  vel_LOC: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Local accelerations of the pixhawk 
  acc_LOC: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Local Angular velocity of the pixhawk 
  Wvel_LOC: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Local Angular acceleration of the pixhawk 
  Wacc_LOC: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Global position of pixhawk obtained starting from the Local position
  # and premultiplying times the rotation matrix
  pos_LOC_to_GLOB: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Global position of pixhawk in NED convention obtained starting from the Local position
  # and premultiplying times the rotation matrix
  pos_LOC_to_GLOB_NED: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Global velocity of pixhawk obtained starting from the Local velocity
  # and premultiplying times the rotation matrix
  vel_LOC_to_GLOB: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Global velocity of pixhawk in NED convention obtained starting from the Local position
  # and premultiplying times the rotation matrix
  vel_LOC_to_GLOB_NED: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Global angular velocity of pixhawk obtained starting from the Local position
  # and premultiplying times the rotation matrix
  Wvel_LOC_to_GLOB: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Global angular velocity of pixhawk in NED convention obtained starting from the Local position
  # and premultiplying times the rotation matrix
  Wvel_LOC_to_GLOB_NED: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))
  # Pixhawk quaternion with y and z components flipped
  quat_fixed: chrono.ChQuaternionD = field(default_factory=lambda: chrono.ChQuaternionD(1, 0, 0, 0))
  # 321 sequence of euler angle (roll, pitch, yaw)
  euler321: chrono.ChVectorD = field(default_factory=lambda: chrono.ChVectorD(0, 0, 0))

@dataclass
class VehicleState:
  roll: float = field(default_factory=lambda: 0.0)
  pitch: float = field(default_factory=lambda: 0.0)
  yaw: float = field(default_factory=lambda: 0.0)
  position_global: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  velocity_global: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  angular_velocity_local: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))