"""
Metrics configuration for GA tuning.

This module defines configuration for inner loop, outer loop, or combined evaluators,
supporting both single-objective and multi-objective optimization modes.

Multi-objective:
- True: Pareto front optimization (number of objectives depends on evaluator_type)
- False: Single-objective weighted sum

Evaluator type (configured in evaluator_config.py) determines objectives:
- "inner": 3 objectives (attitude, angular_velocity, rotational_effort)
- "outer": 3 objectives (position, velocity, translational_effort)
- "combined": 6 objectives (all inner + all outer metrics)
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class InnerLoopMetricWeights:
    """
    Weights for inner loop metrics (attitude/rotational control).
    """
    attitude_tracking_error: float = 1.0          # RMS error in roll/pitch/yaw tracking (rad)
    angular_velocity_tracking_error: float = 1.0  # RMS error in angular rate tracking (rad/s)
    rotational_control_effort: float = 0.1        # RMS control moments u2, u3, u4 (N·m)


@dataclass(frozen=True)
class OuterLoopMetricWeights:
    """
    Weights for outer loop metrics (position/translational control).
    """
    position_error: float = 3.0                      # RMS position tracking error (m)
    velocity_error: float = 0.1                      # RMS velocity tracking error (m/s)
    translational_control_effort: float = 0.01       # RMS total thrust from motors (N)


@dataclass(frozen=True)
class MetricsConfig:
    """
    Configuration for metrics used in GA tuning.

    Attributes:
        multi_objective: Optimization mode
            - True: Multi-objective Pareto front optimization
            - False: Single-objective weighted sum
        normalize_metrics: Whether to normalize metrics before optimization
        inner_loop_weights: Weights for inner loop (used when evaluator_type="inner" and multi_objective=False)
        outer_loop_weights: Weights for outer loop (used when evaluator_type="outer" and multi_objective=False)
    """
    multi_objective: bool = True  # True: Pareto front, False: weighted sum
    normalize_metrics: bool = True  # Normalize metrics
    inner_loop_weights: InnerLoopMetricWeights = field(default_factory=InnerLoopMetricWeights)
    outer_loop_weights: OuterLoopMetricWeights = field(default_factory=OuterLoopMetricWeights)
