import numpy as np


class RKHSWorkspaceLogger:
  """
  Collect RKHS center-selection state for inclusion in the standard MATLAB
  workspace log.
  """
  DOMAIN_IDS = {
    "tran": 0,
    "rot": 1,
  }

  MODE_IDS = {
    "octree": 0,
    "full": 1,
    "default": 2,
  }

  ACTION_IDS = {
    "none": 0,
    "refine": 1,
    "unrefine": -1,
  }

  MAX_ACTIVE_CENTERS = 8
  ROW_SIZE = 50

  def __init__(self) -> None:
    self.data_list = []
    self._feature_maps = {}

  def collectData(self, controller, simulation_time: float) -> None:
    # Record both translational and rotational RKHS feature-map state.
    for domain, feature_map in (
      ("tran", getattr(controller, "rkhs_tran", None)),
      ("rot", getattr(controller, "rkhs_rot", None)),
    ):
      if feature_map is None:
        continue

      self._feature_maps[domain] = feature_map
      self.data_list.append(
        self._row_from_feature_map(
          domain,
          feature_map,
          getattr(controller.odein, "time_now", np.nan),
          simulation_time,
        )
      )

  def toDictionary(self) -> dict:
    if self.data_list:
      data = np.asarray(self.data_list, dtype=float)
    else:
      data = np.empty((0, self.ROW_SIZE), dtype=float)

    return {
      "samples": {
        "time": data[:, 0].reshape(-1, 1),
        "simulation_time": data[:, 1].reshape(-1, 1),
        "domain_id": data[:, 2].reshape(-1, 1),
        "center_mode_id": data[:, 3].reshape(-1, 1),
        "box_changed": data[:, 4].reshape(-1, 1),
        "library_grew": data[:, 5].reshape(-1, 1),
        "box_id": data[:, 6].reshape(-1, 1),
        "dictionary_size": data[:, 7].reshape(-1, 1),
        "active_center_count": data[:, 8].reshape(-1, 1),
        "x": self._columns_to_xyz_dict(data[:, 9:12]),
        "last_added_center": self._columns_to_xyz_dict(data[:, 12:15]),
        "active_center_indices": data[:, 15:23],
        "active_centers": self._active_centers_to_dict(data[:, 23:47]),
        "desired_depth": data[:, 47].reshape(-1, 1),
        "window_l2": data[:, 48].reshape(-1, 1),
        "refinement_action_id": data[:, 49].reshape(-1, 1),
      },
      "domain_ids": self.DOMAIN_IDS,
      "center_mode_ids": self.MODE_IDS,
      "refinement_action_ids": self.ACTION_IDS,
      "final_center_libraries": self._final_center_libraries_to_dict(),
    }

  def _row_from_feature_map(self, domain: str, feature_map, t_sim: float, t_cpu: float) -> np.ndarray:
    # map from method into logging.
    event = feature_map.poll_events(domain) if hasattr(feature_map, "poll_events") else {}
    mode = str(getattr(feature_map, "center_mode", "")).lower().strip()
    corner_idx = np.asarray(event.get("corner_idx", []), dtype=float).reshape(-1)
    active_centers = self._active_centers(feature_map, corner_idx)

    row = np.full((self.ROW_SIZE,), np.nan, dtype=float)
    row[0] = float(t_sim)
    row[1] = float(t_cpu)
    row[2] = self.DOMAIN_IDS.get(domain, -1)
    row[3] = self.MODE_IDS.get(mode, -1)
    row[4] = float(bool(event.get("box_changed", False)))
    row[5] = float(bool(event.get("library_grew", False)))
    row[6] = float(event.get("box_id", -1))
    row[7] = float(event.get("dict_size", getattr(feature_map, "num_centers", 0)))
    row[8] = float(len(active_centers))
    row[9:12] = self._vector3(event.get("x"))
    row[12:15] = self._vector3(event.get("last_added_center"))

    # Keep a fixed-width row: octree normally uses 8 active corner centers.
    n_idx = min(corner_idx.size, self.MAX_ACTIVE_CENTERS)
    row[15:15 + n_idx] = corner_idx[:n_idx]

    for i, center in enumerate(active_centers[:self.MAX_ACTIVE_CENTERS]):
      start = 23 + 3 * i
      row[start:start + 3] = self._vector3(center)

    row[47] = float(event.get("depth", -1))
    row[48] = float(event.get("window_l2", 0.0))
    row[49] = float(self.ACTION_IDS.get(str(event.get("refinement_action", "none")), 0))

    return row

  def _active_centers(self, feature_map, corner_idx: np.ndarray) -> list:
    # Octree stores active centers by index in its underlying library.
    if getattr(feature_map, "center_mode", "") == "octree" and getattr(feature_map, "lib", None) is not None:
      if corner_idx.size == 0:
        return []
      return feature_map.lib.get_centers(corner_idx.astype(int))

    # Default mode can choose a moving subset of learned centers.
    if hasattr(feature_map, "_centers_to_use"):
      centers = list(feature_map._centers_to_use())
      return centers[-self.MAX_ACTIVE_CENTERS:]

    # Full mode has fixed centers; log the tail to fit the fixed row size.
    centers = list(getattr(feature_map, "full_centers", []))
    return centers[-self.MAX_ACTIVE_CENTERS:]

  def _final_center_libraries_to_dict(self) -> dict:
    # Export the complete final center dictionary for each domain.
    libraries = {}
    for domain, feature_map in self._feature_maps.items():
      if getattr(feature_map, "center_mode", "") == "octree" and getattr(feature_map, "lib", None) is not None:
        centers = getattr(feature_map.lib, "Xi_lib", [])
      elif hasattr(feature_map, "centers"):
        centers = getattr(feature_map, "centers", [])
      else:
        centers = getattr(feature_map, "full_centers", [])
      libraries[domain] = self._centers_to_matrix(centers)
    return libraries

  @staticmethod
  def _vector3(value) -> np.ndarray:
    if value is None:
      return np.full((3,), np.nan, dtype=float)
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size < 3:
      out = np.full((3,), np.nan, dtype=float)
      out[:arr.size] = arr
      return out
    return arr[:3]

  @staticmethod
  def _columns_to_xyz_dict(columns: np.ndarray) -> dict:
    return {
      "x": columns[:, 0].reshape(-1, 1),
      "y": columns[:, 1].reshape(-1, 1),
      "z": columns[:, 2].reshape(-1, 1),
    }

  @staticmethod
  def _active_centers_to_dict(columns: np.ndarray) -> dict:
    result = {}
    for i in range(RKHSWorkspaceLogger.MAX_ACTIVE_CENTERS):
      start = 3 * i
      result[f"center_{i}"] = RKHSWorkspaceLogger._columns_to_xyz_dict(columns[:, start:start + 3])
    return result

  @staticmethod
  def _centers_to_matrix(centers: list) -> np.ndarray:
    if not centers:
      return np.empty((0, 3), dtype=float)
    return np.asarray([np.asarray(center, dtype=float).reshape(3,) for center in centers], dtype=float)
