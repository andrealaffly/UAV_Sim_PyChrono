"""
Metrics Package

Inner Loop Metrics (attitude/rotational control):
- attitude_tracking_error: RMS error in roll/pitch/yaw tracking
- angular_velocity_tracking_error: RMS error in angular rate tracking
- rotational_control_effort: RMS control effort (moments u2, u3, u4)

Outer Loop Metrics (position/translational control):
- position_error: RMS position tracking error
- velocity_error: RMS velocity tracking error
- translational_control_effort: RMS control effort (total thrust from all motors)
"""

from .inner_loop_metrics import (
    InnerLoopMetrics,
    InnerLoopMetricsCalculator
)

from .outer_loop_metrics import (
    OuterLoopMetrics,
    OuterLoopMetricsCalculator,
    load_simulation_log
)

from .normalizer import MetricNormalizer

__all__ = [
    # Inner loop
    'InnerLoopMetrics',
    'InnerLoopMetricsCalculator',
    
    # Outer loop
    'OuterLoopMetrics',
    'OuterLoopMetricsCalculator',
    
    # Normalizer
    'MetricNormalizer',
    
    # Utility
    'load_simulation_log'
]
