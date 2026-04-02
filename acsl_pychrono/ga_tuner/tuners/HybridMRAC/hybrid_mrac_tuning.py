"""HybridMRAC controller tuning interface using standard MRAC matrices."""

from typing import Dict
from acsl_pychrono.ga_tuner.tuners.base_mrac_tuning import BaseMRACTuning
from acsl_pychrono.control.HybridMRAC.hybrid_mrac_gains import HybridMRACGains


class HybridMRACTuning(BaseMRACTuning):
    """
    Tuning interface for HybridMRAC controller.
    
    HybridMRAC uses standard MRAC matrices with an additional hybrid safety mechanism.
    The safety mechanism parameters (maximumThrust, maximumRollAngle, etc.) are not
    tuned by the GA, so this class uses the base MRAC parameter set.
    """
    
    CONTROLLER_MODULE = "acsl_pychrono.control.HybridMRAC"
    
    def __init__(self):
        """Initialize HybridMRAC tuning interface."""
        self.GAINS_CLASS = HybridMRACGains
        super().__init__()
