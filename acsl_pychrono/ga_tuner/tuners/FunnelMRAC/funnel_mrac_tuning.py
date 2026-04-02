"""
FunnelMRAC controller parameter tuning implementation.

Extends base MRAC with funnel-specific Q_M_funnel matrices.
"""

from typing import Dict, Any
import numpy as np

from ..base_mrac_tuning import BaseMRACTuning


class FunnelMRACTuning(BaseMRACTuning):
    """
    FunnelMRAC controller tuning interface.
    
    Adds funnel-specific parameters:
    - Q_M_funnel_tran: Funnel matrix for translational dynamics
    - Q_M_funnel_rot: Funnel matrix for rotational dynamics
    """
    
    CONTROLLER_MODULE = "acsl_pychrono.control.FunnelMRAC"
    
    def __init__(self, tuning_config: Dict[str, Any] = None):
        # Import here to avoid circular dependency
        from acsl_pychrono.control.FunnelMRAC import FunnelMRACGains
        self.GAINS_CLASS = FunnelMRACGains
        super().__init__(tuning_config)
    
    def _extract_gains_from_object(self, gains_obj) -> Dict[str, Any]:
        """Extract FunnelMRAC-specific gains."""
        # Get base MRAC gains
        defaults = super()._extract_gains_from_object(gains_obj)
        
        # Add funnel-specific matrices
        for matrix_name in ['Q_M_funnel_tran', 'Q_M_funnel_rot']:
            if hasattr(gains_obj, matrix_name):
                value = getattr(gains_obj, matrix_name)
                defaults[matrix_name] = np.asarray(value)
        
        return defaults
