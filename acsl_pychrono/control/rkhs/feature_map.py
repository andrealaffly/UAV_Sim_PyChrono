"""
THis script build RKHS translational and rotational feature maps based on the Gaussian function.

"""

from dataclasses import dataclass, field
import numpy as np

from acsl_pychrono.control.rkhs.octree_dictionary import OctreeLibrary3D, as_col3
from acsl_pychrono.control.rkhs.rkhs_instances_default import RKHSFeatureMap as DefaultRKHSFeatureMap

# builds all 3D center points from separate x, y, and z grid arrays.
def build_full_grid_centers(x_grid, y_grid, z_grid):
  centers = []     # Empty list 
  for x in np.asarray(x_grid, dtype=float).reshape(-1):
    for y in np.asarray(y_grid, dtype=float).reshape(-1):
      for z in np.asarray(z_grid, dtype=float).reshape(-1):
        centers.append(np.array([[x], [y], [z]], dtype=float))   # Create a column vector and append to the list
  return centers


# Define configuration dataclass for RKHS feature maps
@dataclass
class RKHSRegressorConfig:
  center_mode: str = "octree" # "octree", "full", or "default"
  sigma_tran: float = 1.0
  sigma_rot: float = 1.0
  default_epsilon_tran: float = 0.15  # Distance threshold for adding new centers in the default mode (translational)
  default_epsilon_rot: float = 0.15 # Distance threshold for adding new centers in the default mode (rotational)
  default_max_centers_tran: int = 300
  default_max_centers_rot: int = 300
  default_last_k_tran: int | None = None  # Max number of centers for last-k strategy in default mode (translational)
  default_last_k_rot: int | None = None # Max number of centers for last-k strategy in default mode (rotational)
  gramian_regularization: float = 1e-6
  max_depth: int = 4   # Max depth of the octree 
  refine_after_stay: int = 2  # Deprecated: L2-window refinement is used in octree mode
  rkhs_l2_window_seconds: float = 1.0
  rkhs_l2_high_tran: float = 0.60
  rkhs_l2_low_tran: float = 0.15
  rkhs_l2_high_rot: float = 0.60
  rkhs_l2_low_rot: float = 0.15
  rkhs_initial_depth_tran: int = 0
  rkhs_initial_depth_rot: int = 0
  rho_tran: tuple = (1.0, 1.0, 1.0)
  rho_rot: tuple = (1.0, 1.0, 1.0)
  # Making sure end points are included
  tran_grid: tuple = field(default_factory=lambda: (
    np.arange(-2.0, 8.0 + 1e-9, 1.0),
    np.arange(-3.0, 3.0 + 1e-9, 1.0),
    np.arange(-4.0, 4.0 + 1e-9, 1.0),
  ))
  rot_grid: tuple = field(default_factory=lambda: (
    np.arange(-3.0, 3.0 + 1e-9, 0.5),
    np.arange(-3.0, 3.0 + 1e-9, 0.5),
    np.arange(-4.0, 4.0 + 1e-9, 0.5),
  ))


class RKHSFeatureMap:
  def __init__(self, library=None, sigma=1.0, center_mode="octree", full_centers=None, gramian_regularization=1e-6):
    self.lib = library
    self.sigma = float(sigma)
    self.gramian_regularization = float(gramian_regularization)
    # Normalize center_mode to lowercase string and validate
    self.center_mode = str(center_mode).lower().strip()
    if self.center_mode not in ("octree", "full", "default"):
      raise ValueError("center_mode must be one of 'octree', 'full', or 'default'.")
    
    # For "full" modes, use the provided full candidate list. For "octree", full_centers is ignored.
    self.full_centers = full_centers if full_centers is not None else []
    if self.center_mode in ("full", "default") and len(self.full_centers) == 0:
      raise ValueError(f"center_mode='{self.center_mode}' requires non-empty full_centers.")
 
    # Initializes event tracking variables
    self.last_box_id = None   
    self.last_corner_idx = None   
    self.last_x = None   
    # Event flags
    self.event_box_changed = False   
    self.event_library_grew = False   
    # Number of centers in the feature map 
    self.num_centers = len(self.full_centers) if self.center_mode in ("full", "default") else 0
    # Store last added center
    self.last_added_center = None

  # Update active centers based on x and the library state
  def update_centers(self, x, time_now=None, error_signal=None):
    x = as_col3(x)
    # Copy input x
    self.last_x = x.copy()
    # Clear old event flags
    self.event_box_changed = False
    self.event_library_grew = False
    self.last_added_center = None

    # No library updates for "full" or "default" modes
    if self.center_mode in ("full", "default"):
      self.last_box_id = -1   # Dummy
      self.last_corner_idx = np.array([], dtype=int)
      self.num_centers = len(self.full_centers)
      return

    # Update octree library and get new box_id and corner_idx
    box_id, corner_idx = self.lib.step(x, time_now=time_now, error_signal=error_signal)
    self.last_box_id = int(box_id)
    self.last_corner_idx = np.asarray(corner_idx, dtype=int).copy()
    self.event_box_changed = bool(getattr(self.lib, "event_box_changed", False))
    self.event_library_grew = bool(getattr(self.lib, "event_library_grew", False))
    self.num_centers = int(getattr(self.lib, "num_centers", len(self.lib.Xi_lib)))
    self.last_added_center = getattr(self.lib, "last_added_center", None)

  # Compute RKHS regressor
  def phi(self, x):
    x = as_col3(x)
    if self.last_x is None:
      self.update_centers(x)
    else:
      self.last_x = x.copy()

    # Get the active centers
    centers = self._active_centers()
    value = np.zeros((len(centers), 1))
    for i, center in enumerate(centers):
      d = np.asarray(x - center).reshape(-1)
      value[i, 0] = np.exp(-float(np.dot(d,d)) / (2.0 * self.sigma**2))
    return value

  def inverse_gramian_phi(self, x):
    phi = self.phi(x)
    if phi.size == 0:
      return phi
    return np.matrix(self._gramian_inverse()) * np.matrix(phi)

  def _active_centers(self):
    if self.center_mode == "octree":
      return self.lib.get_centers(self.last_corner_idx)
    return self.full_centers

  def _gramian_inverse(self):
    centers = self._active_centers()
    Xi = np.column_stack([as_col3(center) for center in centers])
    num_centers = Xi.shape[1]
    gramian = np.zeros((num_centers, num_centers))
    for i in range(num_centers):
      for j in range(num_centers):
        d = np.asarray(Xi[:, [i]] - Xi[:, [j]]).reshape(-1)
        gramian[i, j] = np.exp(-float(d.T @ d) / (2.0 * self.sigma**2))
    gramian += self.gramian_regularization * np.eye(num_centers)
    return np.linalg.pinv(gramian)

  # Poll events for logging
  def poll_events(self, tag):
    return {
      "tag": tag,
      "box_changed": bool(self.event_box_changed),
      "library_grew": bool(self.event_library_grew),
      "box_id": int(self.last_box_id) if self.last_box_id is not None else -1,
      "corner_idx": self.last_corner_idx.tolist() if self.last_corner_idx is not None else [],
      "dict_size": int(self.num_centers),
      "x": np.asarray(self.last_x).reshape(-1).tolist() if self.last_x is not None else None,
      "last_added_center": (
        np.asarray(self.last_added_center).reshape(-1).tolist()
        if self.last_added_center is not None else None
      ),
      "depth": int(getattr(self.lib, "desired_depth", -1)) if self.center_mode == "octree" else -1,
      "window_l2": float(getattr(self.lib, "last_window_l2", 0.0)) if self.center_mode == "octree" else 0.0,
      "refinement_action": getattr(self.lib, "last_refinement_action", "none") if self.center_mode == "octree" else "none",
    }

  def clear_events(self):
    self.event_box_changed = False
    self.event_library_grew = False
    self.last_added_center = None


def build_rkhs_feature_maps(config=None):
  config = config or RKHSRegressorConfig()
  center_mode = str(config.center_mode).lower().strip()
  tran_grid = config.tran_grid
  rot_grid = config.rot_grid

  if center_mode == "default":
    rkhs_tran = DefaultRKHSFeatureMap(
      epsilon=config.default_epsilon_tran,
      max_centers=config.default_max_centers_tran,
      last_k=config.default_last_k_tran,
    )
    rkhs_rot = DefaultRKHSFeatureMap(
      epsilon=config.default_epsilon_rot,
      max_centers=config.default_max_centers_rot,
      last_k=config.default_last_k_rot,
    )
    rkhs_tran.center_mode = "default"
    rkhs_rot.center_mode = "default"
    return rkhs_tran, rkhs_rot

  if center_mode == "octree":
    tran_lib = OctreeLibrary3D(
      *tran_grid,
      max_depth=config.max_depth,
      rho=config.rho_tran,
      refine_after_stay=config.refine_after_stay,
      l2_window_seconds=config.rkhs_l2_window_seconds,
      l2_high=config.rkhs_l2_high_tran,
      l2_low=config.rkhs_l2_low_tran,
      initial_depth=config.rkhs_initial_depth_tran,
    )
    rot_lib = OctreeLibrary3D(
      *rot_grid,
      max_depth=config.max_depth,
      rho=config.rho_rot,
      refine_after_stay=config.refine_after_stay,
      l2_window_seconds=config.rkhs_l2_window_seconds,
      l2_high=config.rkhs_l2_high_rot,
      l2_low=config.rkhs_l2_low_rot,
      initial_depth=config.rkhs_initial_depth_rot,
    )
    return (
      RKHSFeatureMap(tran_lib, sigma=config.sigma_tran, center_mode="octree", gramian_regularization=config.gramian_regularization),
      RKHSFeatureMap(rot_lib, sigma=config.sigma_rot, center_mode="octree", gramian_regularization=config.gramian_regularization),
    )

  return (
    RKHSFeatureMap(sigma=config.sigma_tran, center_mode=center_mode, full_centers=build_full_grid_centers(*tran_grid)),
    RKHSFeatureMap(sigma=config.sigma_rot, center_mode=center_mode, full_centers=build_full_grid_centers(*rot_grid)),
  )
