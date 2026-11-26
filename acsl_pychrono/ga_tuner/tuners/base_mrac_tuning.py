"""
Base MRAC tuning implementation for all MRAC-based controllers.

This module provides a common base class that handles:
- SPD matrix parameterization via Cholesky decomposition
- Reference model gains
- E-modification parameters
- Common parameter bound generation

Derived controllers (MRAC, TwoLayerMRAC, HybridMRAC, etc.) extend this base.
"""

from typing import Dict, List, Any, Optional, Type
import numpy as np

from ..core.controller_tuning import ControllerTuningInterface
from ..core.parameter_bounds import ParameterBounds
from ..encoding.parameter_converter import ControllerParameterConverter
from ..encoding.cholesky_utils import build_cholesky_parameter_bounds


class BaseMRACTuning(ControllerTuningInterface):
    """
    Base tuning interface for MRAC-based controllers.
    
    Provides common functionality for:
    - Gamma matrices (adaptive gains)
    - Q matrices (Lyapunov matrices)
    - Reference model gains (K_P_omega_ref, K_I_omega_ref)
    - E-modification parameters (sigma_*)
    """
    
    # Subclasses should override these
    GAINS_CLASS: Type = None  # e.g., MRACGains
    CONTROLLER_MODULE: str = None  # e.g., "acsl_pychrono.control.MRAC"
    
    def __init__(self, tuning_config: Dict[str, Any] = None):
        """Initialize base MRAC tuning.
        
        Args:
            tuning_config: Optional configuration dictionary that can include:
                - 'search_space_type': 'local' or 'global' 
        """
        self.config = tuning_config or {}
        
        # Extract search space type
        self.search_space_type = self.config.get('search_space_type', 'local')
        if self.search_space_type not in ['local', 'global']:
            raise ValueError(f"search_space_type must be 'local' or 'global', got '{self.search_space_type}'")
        
        # Load baseline gains
        self._baseline_gains = self._load_controller_gains()
        
        # Initialize bounds and groups
        self.bounds: Dict[str, List[float]] = {}
        self.gamma_parameter_groups: Dict[str, List[str]] = {}
        self.matrix_sizes: Dict[str, int] = {}
        
        # Build parameter bounds
        self._build_parameter_bounds()
        
        # Create parameter bounds object and converter
        self._parameter_names = list(self.bounds.keys())
        lower_bounds = [self.bounds[name][0] for name in self._parameter_names]
        upper_bounds = [self.bounds[name][1] for name in self._parameter_names]
        
        self._parameter_bounds = ParameterBounds(
            lower_bounds,
            upper_bounds,
            self._parameter_names,
        )
        self.converter = ControllerParameterConverter(
            self._parameter_bounds,
            reference_gains=self._baseline_gains
        )
    
    def _load_controller_gains(self) -> Dict[str, Any]:
        """Load reference gains from controller implementation."""
        if self.GAINS_CLASS is None or self.CONTROLLER_MODULE is None:
            raise NotImplementedError(
                "Subclass must define GAINS_CLASS and CONTROLLER_MODULE"
            )
        
        # Dynamic import to avoid circular dependencies
        module_parts = self.CONTROLLER_MODULE.rsplit('.', 1)
        controller_name = module_parts[1]
        module = __import__(self.CONTROLLER_MODULE, fromlist=[controller_name])
        gains_class = getattr(module, self.GAINS_CLASS.__name__)
        
        # Import FlightParams
        import acsl_pychrono.uav as UAV_Module
        from acsl_pychrono.config.config import VehicleConfig
        from acsl_pychrono.simulation.flight_params import FlightParams
        
        # Instantiation of the UAV object
        uav = UAV_Module.instantiateUAV(
            uav_name=VehicleConfig().uav_name,
            controller_name=controller_name
        )
        flight_params = FlightParams(uav)
        gains_obj = gains_class(flight_params)
        
        return self._extract_gains_from_object(gains_obj)
    
    def _extract_gains_from_object(self, gains_obj) -> Dict[str, Any]:
        """Extract relevant gains from gains object - subclasses should override."""
        defaults = {}
        
        # Standard MRAC matrices
        for matrix_name in ['Gamma_x_tran', 'Gamma_r_tran', 'Gamma_Theta_tran',
                           'Gamma_x_rot', 'Gamma_r_rot', 'Gamma_Theta_rot',
                           'Q_tran', 'Q_rot', 'K_P_omega_ref', 'K_I_omega_ref']:
            if hasattr(gains_obj, matrix_name):
                value = getattr(gains_obj, matrix_name)
                defaults[matrix_name] = np.asarray(value)
        
        # E-modification parameters
        for param in ['sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
                     'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot']:
            if hasattr(gains_obj, param):
                defaults[param] = float(getattr(gains_obj, param))
        
        return defaults
    
    def _build_parameter_bounds(self):
        """Build parameter bounds for all available parameters."""
        
        if self.search_space_type == 'global':
            # Global search: Absolute bounds independent of baseline values
            
            # Gamma matrices - adaptation gain bounds (in log space)
            gamma_diag_bound_fn = lambda value: [-6.0, 10.0]  # exp(-6)=0.0025 to exp(10)=22026
            gamma_off_diag_bound_fn = lambda value: [-100.0, 100.0]  # Absolute bounds for off-diagonal
            
            # Q matrices - Lyapunov matrix bounds (in log space)
            q_diag_bound_fn = lambda value: [-6.0, 5.0]  # exp(-6)=0.0025 to exp(5)=148.4
            q_off_diag_bound_fn = lambda value: [-100.0, 100.0]  # Absolute bounds for off-diagonal
            
            def _positive_bounds(value: float, factor: float = None,
                               *, min_lower: float = 1e-6,
                               max_upper: Optional[float] = None) -> List[float]:
                # For global search, ignore 'value' and 'factor' - use absolute bounds
                return [min_lower, max_upper if max_upper is not None else 20000.0]
        
        else:
            # Local search: Relative bounds around baseline (original behavior)
            scale_factor = 15.0
            log_scale_span = np.log(scale_factor)
            vector_factor = 5.0
            
            # Tighter bounds for Q matrices (affect Lyapunov stability)
            q_scale_factor = 5.0
            q_log_scale_span = np.log(q_scale_factor)
            
            # Gamma matrices
            gamma_diag_bound_fn = lambda value: [value - log_scale_span, value + log_scale_span]
            gamma_off_diag_bound_fn = None  # Use relative bounds
            gamma_off_diag_factor = vector_factor
            gamma_off_diag_span = 1.0
            
            # Q matrices
            q_diag_bound_fn = lambda value: [value - q_log_scale_span, value + q_log_scale_span]
            q_off_diag_bound_fn = None  # Use relative bounds
            q_off_diag_factor = 2.0
            q_off_diag_span = 0.5
            
            def _positive_bounds(value: float, factor: float = 10.0,
                               *, min_lower: float = 1e-6,
                               max_upper: Optional[float] = None) -> List[float]:
                lower = max(min_lower, value / factor)
                upper = max(lower * 2.0, value * factor)
                if max_upper is not None:
                    upper = min(upper, max_upper)
                    if upper <= lower:
                        upper = lower * 1.5
                return [lower, upper]
        
        # SPD matrices (Gamma/Q) with Cholesky parameterization - ALWAYS added to bounds
        # User configuration will filter which ones are actually tuned
        for matrix_name, value in self._baseline_gains.items():
            if not (matrix_name.startswith('Gamma_') or matrix_name in ('Q_tran', 'Q_rot')):
                continue
            if not isinstance(value, np.ndarray) or value.ndim != 2:
                continue
            
            prefix = matrix_name.replace('Gamma_', 'gamma_').lower() if matrix_name.startswith('Gamma_') else matrix_name.lower()
            baseline_matrix = value
            matrix_size = baseline_matrix.shape[0]
            self.matrix_sizes[prefix] = matrix_size
            
            # Select bounds based on matrix type and search mode
            is_q_matrix = matrix_name.startswith('Q_')
            
            if self.search_space_type == 'global':
                # Global search: use absolute bounds
                diag_fn = q_diag_bound_fn if is_q_matrix else gamma_diag_bound_fn
                off_diag_fn = q_off_diag_bound_fn if is_q_matrix else gamma_off_diag_bound_fn
                
                matrix_bounds, parameter_names = build_cholesky_parameter_bounds(
                    baseline_matrix,
                    prefix,
                    diag_fn,
                    off_diag_bound_fn=off_diag_fn,
                    log_diagonals=True,
                )
            else:
                # Local search: use relative bounds
                diag_fn = q_diag_bound_fn if is_q_matrix else gamma_diag_bound_fn
                off_factor = q_off_diag_factor if is_q_matrix else gamma_off_diag_factor
                off_span = q_off_diag_span if is_q_matrix else gamma_off_diag_span
                
                matrix_bounds, parameter_names = build_cholesky_parameter_bounds(
                    baseline_matrix,
                    prefix,
                    diag_fn,
                    off_diag_factor=off_factor,
                    min_off_diag_span=off_span,
                    log_diagonals=True,
                )
            
            for name, bounds in matrix_bounds.items():
                self.bounds[name] = bounds
            self.gamma_parameter_groups[prefix] = parameter_names
        
        # Reference model gains (K_P_omega_ref, K_I_omega_ref)
        if 'K_P_omega_ref' in self._baseline_gains:
            kp_ref = np.diag(self._baseline_gains['K_P_omega_ref'])
            for i, value in enumerate(kp_ref, start=1):
                if self.search_space_type == 'global':
                    self.bounds[f"K_P_omega_ref_{i}"] = [0.001, 500.0]
                else:
                    self.bounds[f"K_P_omega_ref_{i}"] = _positive_bounds(float(value), factor=5.0)
        
        if 'K_I_omega_ref' in self._baseline_gains:
            ki_ref = np.diag(self._baseline_gains['K_I_omega_ref'])
            for i, value in enumerate(ki_ref, start=1):
                if self.search_space_type == 'global':
                    self.bounds[f"K_I_omega_ref_{i}"] = [0.0001, 200.0]
                else:
                    self.bounds[f"K_I_omega_ref_{i}"] = _positive_bounds(float(value), factor=10.0)
        
        # E-modification parameters
        for name in ['sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
                     'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot']:
            if name in self._baseline_gains:
                if self.search_space_type == 'global':
                    self.bounds[name] = [0.001, 50.0]
                else:
                    ref_value = float(self._baseline_gains[name])
                    self.bounds[name] = _positive_bounds(ref_value, factor=3.0,
                                                         min_lower=0.05, max_upper=1.5)
    
    def get_parameter_bounds(self) -> ParameterBounds:
        """Get parameter bounds."""
        return self._parameter_bounds
    
    def get_parameter_names(self) -> List[str]:
        """Get ordered list of parameter names."""
        return self._parameter_names
    
    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """Get parameter grouping information."""
        return self._parameter_bounds.get_parameter_groups()
    
    def get_matrix_sizes(self) -> Dict[str, int]:
        """Get sizes of all SPD matrices."""
        return self.matrix_sizes.copy()
    
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """Convert parameter vector to gain structure."""
        gains = self.converter.vector_to_gains(parameter_vector)
        
        # Convert matrices to arrays
        normalized_gains = {}
        for key, value in gains.items():
            if isinstance(value, np.matrix):
                normalized_gains[key] = np.asarray(value)
            else:
                normalized_gains[key] = value
        return normalized_gains
    
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """Convert gain structure to parameter vector."""
        prepared_gains = {}
        for key, value in gains.items():
            if isinstance(value, np.ndarray):
                prepared_gains[key] = np.asmatrix(value)
            elif isinstance(value, (list, tuple)):
                prepared_gains[key] = np.asmatrix(value)
            else:
                prepared_gains[key] = value
        
        return self.converter.gains_to_vector(prepared_gains)
    
    def get_default_gains(self) -> Dict[str, Any]:
        """Get default gains from baseline."""
        defaults = {}
        for key, value in self._baseline_gains.items():
            if isinstance(value, np.ndarray):
                defaults[key] = value.copy()
            else:
                defaults[key] = float(value)
        return defaults
