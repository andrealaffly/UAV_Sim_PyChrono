# rkhs_instances_default.py
import os
import csv
import numpy as np

class RKHSFeatureMap:
    def __init__(self, epsilon=0.2, max_centers=300, last_k=None):
        self.epsilon = float(epsilon)
        self.max_centers = int(max_centers)
        self.last_k = None if last_k is None else int(last_k)

        self.centers = []
        self.num_centers = 0

        # store last input passed to update_centers (for logging)
        self.last_x = None

        # event flags
        self.center_updated = False
        self.last_added_center = None

    def reset_event_flag(self):
        self.center_updated = False
        self.last_added_center = None

    def set_last_k(self, last_k):
        self.last_k = None if last_k is None else int(last_k)

    def update_centers(self, x):
        x = np.asarray(x).reshape(3, 1)
        self.last_x = x.copy()
        self.reset_event_flag()

        if self.num_centers == 0:
            self._add_center(x)
            return

        for c in self.centers:
            if np.linalg.norm(x - c) < self.epsilon:
                return

        if self.num_centers < self.max_centers:
            self._add_center(x)

    def _add_center(self, x):
        self.centers.append(x.copy())
        self.num_centers += 1
        self.last_added_center = x.copy()
        self.center_updated = True

    def _centers_to_use(self):
        if self.num_centers == 0:
            return []

        if self.last_k is None:
            return self.centers

        k = max(1, int(self.last_k))
        if k >= self.num_centers:
            return self.centers
        return self.centers[-k:]

    def phi(self, x):
        x = np.asarray(x).reshape(3, 1)

        if self.num_centers == 0:
            return (-0.5 * np.linalg.norm(x)) * x

        centers_use = self._centers_to_use()

        v = np.zeros((3, 1))
        for c in centers_use:
            diff = x - c
            kappa = np.exp(-np.linalg.norm(diff) ** 2)
            v += kappa * diff

        return v
    
    def poll_events(self, tag: str):
        return {
            "tag": tag,
            "box_changed": bool(self.center_updated),
            "library_grew": bool(self.center_updated),
            "box_id": -1,
            "corner_idx": [],
            "dict_size": int(self.num_centers),
            "x": np.asarray(self.last_x).reshape(-1).tolist() if self.last_x is not None else None,
            "last_added_center": (
                np.asarray(self.last_added_center).reshape(-1).tolist()
                if self.last_added_center is not None else None
            ),
        }

    def clear_events(self):
        self.reset_event_flag()
    

# Module-level helper: return unified event dict for simple RKHS implementation
def rkhs_event_default(tag: str, x_in, rkhs_obj, method="default"):
    """
    Returns a unified event dict for trajectory-drop RKHS.
    tag: "tran" or "rot"
    x_in: current input (3,) or (3,1)
    """
    x = np.asarray(x_in).reshape(3,)
    n = int(getattr(rkhs_obj, "num_centers", 0))

    # last_k: None => use all centers => encode as -1
    lk = getattr(rkhs_obj, "last_k", None)
    k_used = -1 if (lk is None) else int(lk)

    # event detection
    if bool(getattr(rkhs_obj, "center_updated", False)):
        event_type = "center_added"
        last_c = getattr(rkhs_obj, "last_added_center", None)
        if last_c is not None:
            last_c = np.asarray(last_c).reshape(3,)
            payload = [float(last_c[0]), float(last_c[1]), float(last_c[2])]
        else:
            payload = [np.nan, np.nan, np.nan]
    else:
        event_type = "none"
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

# DEFAULT PARAMS
RKHS_EPS_TRAN = 0.15
RKHS_EPS_ROT  = 0.15
RKHS_MAX_CENTERS_TRAN = 300
RKHS_MAX_CENTERS_ROT  = 300

# Adaptive last-k tuning
K_MIN_TRAN  = 10
K_MAX_TRAN  = 18
K_INIT_TRAN = 12

K_MIN_ROT  = 10
K_MAX_ROT  = 18
K_INIT_ROT = 12

ERR_GOOD_TRAN = 0.20
ERR_BAD_TRAN  = 0.60
ERR_GOOD_ROT  = 0.20
ERR_BAD_ROT   = 0.60

ERR_EMA_BETA = 0.90
K_UPDATE_DT  = 0.15  # seconds

# INTERNAL STATE (kept here so wrapper stays simple)

Use_Adaptive_last_k = False

_rkhs_k_tran = K_INIT_TRAN
_rkhs_k_rot  = K_INIT_ROT

_err_ema_tran = 0.0
_err_ema_rot  = 0.0

_last_k_update_time_tran = -1e9
_last_k_update_time_rot  = -1e9


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def adaptive_lastk_update_tran(time_now, err_scalar, rkhs_obj: RKHSFeatureMap):
    """
    If Use_Adaptive_last_k:
        adapt rkhs_obj.last_k in [K_MIN_TRAN, K_MAX_TRAN]
    Else:
        force rkhs_obj.last_k = None (use ALL centers)
    Returns (k_tran_or_None, err_ema_tran).
    """
    global _rkhs_k_tran, _err_ema_tran, _last_k_update_time_tran

    if not Use_Adaptive_last_k:
        rkhs_obj.set_last_k(None)  # use ALL centers
        return (None, _err_ema_tran)

    e = float(err_scalar)
    _err_ema_tran = ERR_EMA_BETA * _err_ema_tran + (1.0 - ERR_EMA_BETA) * e

    if (time_now - _last_k_update_time_tran) >= K_UPDATE_DT:
        if _err_ema_tran < ERR_GOOD_TRAN:
            _rkhs_k_tran = _clamp(_rkhs_k_tran - 1, K_MIN_TRAN, K_MAX_TRAN)
        elif _err_ema_tran > ERR_BAD_TRAN:
            _rkhs_k_tran = _clamp(_rkhs_k_tran + 1, K_MIN_TRAN, K_MAX_TRAN)

        rkhs_obj.set_last_k(_rkhs_k_tran)
        _last_k_update_time_tran = time_now

    return (_rkhs_k_tran, _err_ema_tran)


def adaptive_lastk_update_rot(time_now, err_scalar, rkhs_obj: RKHSFeatureMap):
    global _rkhs_k_rot, _err_ema_rot, _last_k_update_time_rot

    if not Use_Adaptive_last_k:
        rkhs_obj.set_last_k(None)  # use ALL centers
        return (None, _err_ema_rot)

    e = float(err_scalar)
    _err_ema_rot = ERR_EMA_BETA * _err_ema_rot + (1.0 - ERR_EMA_BETA) * e

    if (time_now - _last_k_update_time_rot) >= K_UPDATE_DT:
        if _err_ema_rot < ERR_GOOD_ROT:
            _rkhs_k_rot = _clamp(_rkhs_k_rot - 1, K_MIN_ROT, K_MAX_ROT)
        elif _err_ema_rot > ERR_BAD_ROT:
            _rkhs_k_rot = _clamp(_rkhs_k_rot + 1, K_MIN_ROT, K_MAX_ROT)

        rkhs_obj.set_last_k(_rkhs_k_rot)
        _last_k_update_time_rot = time_now

    return (_rkhs_k_rot, _err_ema_rot)

# Logger
_RKHS_LOGGER_INITIALIZED = False
_RKHS_LOG_PATH = None

def init_rkhs_logger(log_dir, filename="rkhs_k_log.csv"):
    global _RKHS_LOGGER_INITIALIZED, _RKHS_LOG_PATH
    if log_dir is None:
        return None

    os.makedirs(log_dir, exist_ok=True)
    # Ensure logger filename is unique per run by appending a timestamp if not present
    import datetime, re
    ts_pattern = re.compile(r"\d{8}_\d{6}(?:_\d{6})?")
    name, ext = os.path.splitext(filename)
    if not ts_pattern.search(filename):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{name}_{ts}{ext}"

    _RKHS_LOG_PATH = os.path.join(log_dir, filename)

    if not os.path.exists(_RKHS_LOG_PATH):
        with open(_RKHS_LOG_PATH, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t",
                "use_adaptive_last_k",
                "k_tran", "err_ema_tran", "n_centers_tran",
                "k_rot",  "err_ema_rot",  "n_centers_rot",
            ])

    _RKHS_LOGGER_INITIALIZED = True
    return _RKHS_LOG_PATH


def log_rkhs_k(time_now, k_tran, err_ema_tran, k_rot, err_ema_rot, rkhs_tran_obj, rkhs_rot_obj):
    if not _RKHS_LOGGER_INITIALIZED or (_RKHS_LOG_PATH is None):
        return

    n_tran = int(getattr(rkhs_tran_obj, "num_centers", 0))
    n_rot  = int(getattr(rkhs_rot_obj,  "num_centers", 0))

    # write -1 for k when "use all centers" so plots don't lie
    k_tran_out = -1 if (k_tran is None) else int(k_tran)
    k_rot_out  = -1 if (k_rot  is None) else int(k_rot)

    with open(_RKHS_LOG_PATH, "a", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            float(time_now),
            int(bool(Use_Adaptive_last_k)),
            k_tran_out, float(err_ema_tran), n_tran,
            k_rot_out,  float(err_ema_rot),  n_rot
        ])


# ============================================================
# GLOBAL SINGLETONS expected by simulator
# ============================================================
rkhs_tran = RKHSFeatureMap(
    epsilon=RKHS_EPS_TRAN,
    max_centers=RKHS_MAX_CENTERS_TRAN,
    last_k=(K_INIT_TRAN if Use_Adaptive_last_k else None)
)

rkhs_rot = RKHSFeatureMap(
    epsilon=RKHS_EPS_ROT,
    max_centers=RKHS_MAX_CENTERS_ROT,
    last_k=(K_INIT_ROT if Use_Adaptive_last_k else None)
)
