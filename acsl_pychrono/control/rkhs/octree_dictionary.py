import numpy as np


def as_col3(x):
  return np.asarray(x, dtype=float).reshape(3, 1)


def key3(x, nd=12):
  x = np.asarray(x, dtype=float).reshape(3,)
  return (round(float(x[0]), nd), round(float(x[1]), nd), round(float(x[2]), nd))


def rkhs_event_dictionary(tag: str, x_in, rkhs_obj, method="dictionary"):
  x = np.asarray(x_in, dtype=float).reshape(3,)
  k_used = -1

  if hasattr(rkhs_obj, "poll_events"):
    ev = rkhs_obj.poll_events(tag)

    if ev.get("library_grew", False):
      event_type = "library_grew"
    elif ev.get("box_changed", False):
      event_type = "box_changed"
    else:
      event_type = "none"

    n = int(ev.get("dict_size", ev.get("num_centers", 0)))
    box_id = int(ev.get("box_id", -1))
    corner_idx = ev.get("corner_idx", [])
    c0 = int(corner_idx[0]) if len(corner_idx) > 0 else -1
    c1 = int(corner_idx[1]) if len(corner_idx) > 1 else -1
    payload = [float(box_id), float(c0), float(c1)]

    if hasattr(rkhs_obj, "clear_events"):
      rkhs_obj.clear_events()
  else:
    event_type = "none"
    n = int(getattr(rkhs_obj, "num_centers", 0))
    payload = [np.nan, np.nan, np.nan]

  return {
    "domain": tag,
    "method": method,
    "event_type": event_type,
    "n_centers": n,
    "k_used": k_used,
    "x": [float(x[0]), float(x[1]), float(x[2])],
    "payload": payload,
  }


class Box3D:
  __slots__ = (
    "xL", "xR", "yL", "yR", "zL", "zR", "center", "corner_idx",
    "depth", "active", "parent_id", "children_ids"
  )

  def __init__(self, xL, xR, yL, yR, zL, zR, center, corner_idx,
               depth=0, active=True, parent_id=None):
    self.xL = float(xL)
    self.xR = float(xR)
    self.yL = float(yL)
    self.yR = float(yR)
    self.zL = float(zL)
    self.zR = float(zR)
    self.center = np.asarray(center, dtype=float).reshape(3,)   # Store box center for distance calculations
    self.corner_idx = np.asarray(corner_idx, dtype=int).reshape(8,)
    self.depth = int(depth)    # Initialize box depth
    self.active = bool(active)   # If a box is refined, it becomes inactive and its children become active
    self.parent_id = None if parent_id is None else int(parent_id)
    self.children_ids = []

  # Check whether a point x is contained within the box
  def contains(self, x):
    x = np.asarray(x, dtype=float).reshape(3,)
    return (
      self.xL <= x[0] <= self.xR
      and self.yL <= x[1] <= self.yR
      and self.zL <= x[2] <= self.zR
    )



class OctreeLibrary3D:
  def __init__(
      self,
      x_grid,
      y_grid,
      z_grid,
      max_depth=3,
      rho=(1.0, 1.0, 1.0),
      refine_after_stay=2,
      l2_window_seconds=1.0,
      l2_high=np.inf,
      l2_low=-np.inf,
      initial_depth=0,
    ):
    self.max_depth = int(max_depth)
    # rho is used to weight distance within a box
    self.rho = np.asarray(rho, dtype=float).reshape(3,)
    self.refine_after_stay = int(refine_after_stay)
    self.l2_window_seconds = float(l2_window_seconds)
    self.l2_high = float(l2_high)
    self.l2_low = float(l2_low)
    self.desired_depth = int(np.clip(initial_depth, 0, self.max_depth))
    self.last_window_l2 = 0.0
    self.last_refinement_action = "none"

    self._window_start_time = None
    self._last_error_time = None
    self._l2_accumulator = 0.0

    self.Xi_lib = []
    self._vertex_map = {}
    self.Boxes = []

    self._last_box_id = None
    self._stay_count = 0

    self.event_box_changed = False
    self.event_library_grew = False
    self.last_box_id = None
    self.last_corner_idx = None
    self.last_x = None
    self.num_centers = 0
    self.last_added_center = None

    self._build_uniform_boxes(x_grid, y_grid, z_grid)
    self.num_centers = len(self.Xi_lib)

  def _get_or_add_vertex(self, v):
    v = np.asarray(v, dtype=float).reshape(3,)
    key = key3(v)
    if key in self._vertex_map:
      return self._vertex_map[key]

    idx = len(self.Xi_lib)
    self.Xi_lib.append(v.reshape(3, 1))
    self._vertex_map[key] = idx
    self.event_library_grew = True
    self.last_added_center = v.reshape(3, 1).copy()
    self.num_centers = len(self.Xi_lib)
    return idx

  # Create boxes  
  def _corners_from_bounds(self, xL, xR, yL, yR, zL, zR):
    corners = np.array([
      [xL, yL, zL],
      [xR, yL, zL],
      [xL, yR, zL],
      [xR, yR, zL],
      [xL, yL, zR],
      [xR, yL, zR],
      [xL, yR, zR],
      [xR, yR, zR],
    ], dtype=float)
    return np.asarray([self._get_or_add_vertex(corner) for corner in corners], dtype=int)

  # Convert grids to array of boxes and maintain a vertex library
  def _build_uniform_boxes(self, x_grid, y_grid, z_grid):
    xg = np.asarray(x_grid, dtype=float).reshape(-1,)
    yg = np.asarray(y_grid, dtype=float).reshape(-1,)
    zg = np.asarray(z_grid, dtype=float).reshape(-1,)
    if len(xg) < 2 or len(yg) < 2 or len(zg) < 2:
      raise ValueError("Need at least two grid points along each axis.")
    
    # Find box centers
    for i in range(len(xg) - 1):
      for j in range(len(yg) - 1):
        for k in range(len(zg) - 1):
          xL, xR = xg[i], xg[i + 1]
          yL, yR = yg[j], yg[j + 1]
          zL, zR = zg[k], zg[k + 1]
          center = np.array([(xL + xR) / 2, (yL + yR) / 2, (zL + zR) / 2], dtype=float)
          corner_idx = self._corners_from_bounds(xL, xR, yL, yR, zL, zR)
          self.Boxes.append(Box3D(xL, xR, yL, yR, zL, zR, center, corner_idx))
 
  # Uses contains to find trajectory belong to a box
  def _find_containing_leaf(self, x):
    x = np.asarray(x, dtype=float).reshape(3,)
    candidates = [
      box_id for box_id, box in enumerate(self.Boxes)
      if box.active and box.contains(x)
    ]
    if not candidates:
      return None
    depths = [self.Boxes[box_id].depth for box_id in candidates]
    return candidates[int(np.argmax(depths))]

  # If not contained in any box, find the nearest leaf box to x
  def _nearest_leaf(self, x):
    x = np.asarray(x, dtype=float).reshape(3,)
    best_id = None
    best_d2 = np.inf
    for box_id, box in enumerate(self.Boxes):
      if not box.active:
        continue
      d = self.rho * (x - box.center)
      d2 = float(d @ d)
      if d2 < best_d2:
        best_d2 = d2
        best_id = box_id
    return best_id

  # Refinement
  def _refine_box(self, box_id):
    box = self.Boxes[box_id]
    if box.depth >= self.max_depth:
      return
    if box.children_ids:
      return

    xm = (box.xL + box.xR) / 2
    ym = (box.yL + box.yR) / 2
    zm = (box.zL + box.zR) / 2
    ranges = [
      (box.xL, xm, box.yL, ym, box.zL, zm),
      (xm, box.xR, box.yL, ym, box.zL, zm),
      (box.xL, xm, ym, box.yR, box.zL, zm),
      (xm, box.xR, ym, box.yR, box.zL, zm),
      (box.xL, xm, box.yL, ym, zm, box.zR),
      (xm, box.xR, box.yL, ym, zm, box.zR),
      (box.xL, xm, ym, box.yR, zm, box.zR),
      (xm, box.xR, ym, box.yR, zm, box.zR),
    ]

    box.active = False
    for xL, xR, yL, yR, zL, zR in ranges:
      center = np.array([(xL + xR) / 2, (yL + yR) / 2, (zL + zR) / 2], dtype=float)
      corner_idx = self._corners_from_bounds(xL, xR, yL, yR, zL, zR)
      child_id = len(self.Boxes)
      self.Boxes.append(Box3D(
        xL, xR, yL, yR, zL, zR, center, corner_idx,
        depth=box.depth + 1, parent_id=box_id
      ))
      box.children_ids.append(child_id)

  def _descend_to_depth(self, box_id, x, depth):
    x = np.asarray(x, dtype=float).reshape(3,)
    target_depth = int(np.clip(depth, 0, self.max_depth))

    while self.Boxes[box_id].depth < target_depth:
      self._refine_box(box_id)
      children = self.Boxes[box_id].children_ids
      if not children:
        break

      containing = [
        child_id for child_id in children
        if self.Boxes[child_id].contains(x)
      ]
      if containing:
        box_id = containing[0]
      else:
        box_id = min(
          children,
          key=lambda child_id: float(
            (self.rho * (x - self.Boxes[child_id].center))
            @ (self.rho * (x - self.Boxes[child_id].center))
          )
        )
    return box_id

  def _ancestor_at_depth(self, box_id, depth):
    target_depth = int(np.clip(depth, 0, self.max_depth))
    while self.Boxes[box_id].depth > target_depth and self.Boxes[box_id].parent_id is not None:
      box_id = self.Boxes[box_id].parent_id
    return box_id

  def _update_l2_window(self, time_now, error_signal):
    self.last_refinement_action = "none"
    if time_now is None or error_signal is None or self.l2_window_seconds <= 0.0:
      return

    time_now = float(time_now)
    error_signal = np.asarray(error_signal, dtype=float).reshape(-1,)

    if self._window_start_time is None:
      self._window_start_time = time_now
      self._last_error_time = time_now
      self._l2_accumulator = 0.0
      return

    dt = max(0.0, time_now - self._last_error_time)
    self._last_error_time = time_now
    self._l2_accumulator += float(error_signal @ error_signal) * dt

    if (time_now - self._window_start_time) < self.l2_window_seconds:
      return

    self.last_window_l2 = float(np.sqrt(max(self._l2_accumulator, 0.0)))
    if self.last_window_l2 > self.l2_high and self.desired_depth < self.max_depth:
      self.desired_depth += 1
      self.last_refinement_action = "refine"
    elif self.last_window_l2 < self.l2_low and self.desired_depth > 0:
      self.desired_depth -= 1
      self.last_refinement_action = "unrefine"

    self._window_start_time = time_now
    self._l2_accumulator = 0.0

  # Main update function: given x, update library state and return box_id and corner_idx
  def step(self, x, time_now=None, error_signal=None):
    x = as_col3(x)
    self.event_box_changed = False
    self.event_library_grew = False
    self.last_added_center = None
    self.last_x = x.copy()
    self._update_l2_window(time_now, error_signal)

    box_id = self._find_containing_leaf(x)
    if box_id is None:
      box_id = self._nearest_leaf(x)
    if box_id is None:
      raise RuntimeError("Octree dictionary has no active boxes.")

    box_id = self._ancestor_at_depth(box_id, self.desired_depth)
    box_id = self._descend_to_depth(box_id, x, self.desired_depth)

    if box_id == self._last_box_id:
      self._stay_count += 1
    else:
      self.event_box_changed = True
      self._last_box_id = box_id
      self._stay_count = 1

    if self.last_refinement_action != "none":
      self.event_box_changed = True

    # Store id
    self.last_box_id = int(box_id)
    self.last_corner_idx = self.Boxes[box_id].corner_idx.copy()
    self.num_centers = len(self.Xi_lib)
    return self.last_box_id, self.last_corner_idx
  
  # Return kernel centers corresponding to corner_idx for feature map calculation
  def get_centers(self, corner_idx):
    return [self.Xi_lib[int(idx)] for idx in np.asarray(corner_idx, dtype=int).reshape(-1,)]
