"""NonAdaptiveEBCI controller tuning interface using standard MRAC matrices."""

from typing import Dict
from acsl_pychrono.ga_tuner.tuners.base_mrac_tuning import BaseMRACTuning
from acsl_pychrono.control.NonAdaptiveEBCI.nonadaptive_ebci_gains import NonAdaptiveEBCIGains


class NonAdaptiveEBCITuning(BaseMRACTuning):
    """
    Tuning interface for NonAdaptiveEBCI controller.
    
    NonAdaptiveEBCI uses standard MRAC matrices with Extended Berkeley Control Initiative
    parameters. The EBCI-specific parameters are not tuned by the GA for now, so this class uses
    the base MRAC parameter set.
    """
    
    CONTROLLER_MODULE = "acsl_pychrono.control.NonAdaptiveEBCI"
    
    def __init__(self):
        """Initialize NonAdaptiveEBCI tuning interface."""
        self.GAINS_CLASS = NonAdaptiveEBCIGains
        super().__init__()
