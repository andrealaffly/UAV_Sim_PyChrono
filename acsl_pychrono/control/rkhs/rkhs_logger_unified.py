# rkhs_logger_unified.py
import os
import csv
import datetime
import re

class RKHSUnifiedLogger:
    def __init__(self, log_dir, filename):
        os.makedirs(log_dir, exist_ok=True)     # Create log directory if it doesn't exist

        # If filename does not already contain a timestamp, append one to make it unique per run
        # Detect common timestamp patterns like YYYYMMDD_HHMMSS or with microseconds
        ts_pattern = re.compile(r"\d{8}_\d{6}(?:_\d{6})?")
        name, ext = os.path.splitext(filename)
        if not ts_pattern.search(filename): 
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{name}_{ts}{ext}"

        self.path = os.path.join(log_dir, filename)
        self._init()

    def _init(self):
        if not os.path.exists(self.path):
            with open(self.path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "t_sim", "t_cpu",
                    "domain", "method", "event_type",
                    "n_centers", "k_used",
                    "x0", "x1", "x2",
                    "p0", "p1", "p2",
                ])

    def write(self, t_sim, t_cpu, ev):
        x = ev["x"]
        p = ev["payload"]
        with open(self.path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                float(t_sim), float(t_cpu),
                ev["domain"], ev["method"], ev["event_type"],
                int(ev["n_centers"]), int(ev["k_used"]),
                float(x[0]), float(x[1]), float(x[2]),
                float(p[0]), float(p[1]), float(p[2]),
            ])