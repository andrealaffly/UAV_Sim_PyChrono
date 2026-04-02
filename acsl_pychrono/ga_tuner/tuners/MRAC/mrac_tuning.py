"""MRAC controller parameter tuning implementation using base class."""

from typing import Dict, Any
from ..base_mrac_tuning import BaseMRACTuning
from acsl_pychrono.control.MRAC.mrac_gains import MRACGains


class MRACTuning(BaseMRACTuning):
    """
    MRAC controller tuning interface.
    
    Uses BaseMRACTuning for standard MRAC matrices with Cholesky parameterization.
    All matrix discovery and parameter bounds generation is handled automatically.
    """
    
    CONTROLLER_MODULE = "acsl_pychrono.control.MRAC"
    
    def __init__(self, tuning_config: Dict[str, Any] = None):
        """Initialize MRAC tuning interface."""
        self.GAINS_CLASS = MRACGains
        super().__init__(tuning_config)
