"""
Parameter converter for comprehensive MRAC parameter handling.
Converts between parameter vectors and MRAC gain structures based on actual mrac_gains.py.
"""

import ast
import pathlib
import numpy as np
from typing import Dict, List, Any, Tuple

from .cholesky_utils import (
    flatten_cholesky,
    get_cholesky_parameter_names,
    reconstruct_from_cholesky,
)


def _is_diagonal_matrix(value: Any) -> bool:
    """Check if value is a diagonal matrix."""
    if not isinstance(value, np.ndarray) or value.ndim != 2:
        return False
    if value.shape[0] != value.shape[1]:
        return False
    # Check if matrix is diagonal (all off-diagonal elements are zero)
    return np.allclose(value, np.diag(np.diag(value)))

def _discover_parameter_structure(gains: Dict[str, Any]) -> Tuple[Dict[str, Dict], Dict[str, tuple], List[str]]:
    """
    Discover parameter structure from loaded gains.
    
    Returns:
        - spd_matrices: Dict mapping matrix names to config (prefix, size)
        - diagonal_matrices: Dict mapping matrix names to parameter names tuple
        - scalars: List of scalar parameter names
    """
    spd_matrices = {}
    diagonal_matrices = {}
    scalars = []
    
    for name, value in gains.items():
        if isinstance(value, np.ndarray) and value.ndim == 2:
            if name.startswith('Gamma_') or name in ('Q_tran', 'Q_rot'):
                prefix = name.replace('Gamma_', 'gamma_').lower() if name.startswith('Gamma_') else name.lower()
                spd_matrices[name] = {
                    'prefix': prefix,
                    'size': value.shape[0]
                }
            elif _is_diagonal_matrix(value):
                size = value.shape[0]
                param_names = tuple(f"{name}_{i+1}" for i in range(size))
                diagonal_matrices[name] = param_names
        elif isinstance(value, (int, float, np.number)):
            scalars.append(name)
    
    return spd_matrices, diagonal_matrices, scalars


_DEFAULT_GAINS_PATH = pathlib.Path(__file__).resolve().parents[2] / "control" / "MRAC" / "mrac_gains.py"

def _build_spd_matrix_configs(gains: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Build matrix configs from loaded gains dynamically."""
    spd_matrices, _, _ = _discover_parameter_structure(gains)
    return spd_matrices

def load_default_gains_from_source() -> Dict[str, Any]:
    """Parse gains file to extract default gain values."""
    try:
        source = _DEFAULT_GAINS_PATH.read_text()
    except OSError as e:
        raise RuntimeError(f"Failed to read gains file: {e}")

    tree = ast.parse(source, filename=str(_DEFAULT_GAINS_PATH))
    extracted: Dict[str, Any] = {}

    class _GainsExtractor(ast.NodeVisitor):
        def visit_Assign(self, node):
            for target in node.targets:
                if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == 'self':
                    name = target.attr
                    # Extract ALL self.* assignments (no hardcoded filter)
                    if name not in extracted:
                        expr = ast.Expression(node.value)
                        code = compile(expr, str(_DEFAULT_GAINS_PATH), 'eval')
                        try:
                            extracted[name] = eval(code, {'np': np})
                        except Exception:
                            pass
            self.generic_visit(node)

    _GainsExtractor().visit(tree)

    # Normalize matrix types
    normalized = {}
    for name, value in extracted.items():
        if isinstance(value, np.matrix):
            normalized[name] = np.asarray(value)
        elif isinstance(value, np.ndarray):
            normalized[name] = value.copy()
        elif isinstance(value, (int, float, np.number)):
            normalized[name] = float(value)
        # Skip other types (like booleans, strings, etc.)
    
    return normalized

class ControllerParameterConverter:
    """
    Converts between parameter vectors and controller gain structures.
    Handles controller parameters with dynamic structure discovery.
    """
    
    def __init__(self, parameter_bounds, reference_gains=None):
        self.bounds = parameter_bounds
        self.parameter_names = parameter_bounds.parameter_names
        get_groups = getattr(parameter_bounds, "get_parameter_groups", None)
        self.parameter_groups = get_groups() if callable(get_groups) else {}
        
        # Load gains from source and discover structure dynamically
        self._reference_gains = reference_gains if reference_gains is not None else load_default_gains_from_source()
        self._spd_matrix_configs, self._diagonal_matrix_params, self._scalar_params = _discover_parameter_structure(self._reference_gains)
    
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Convert parameter vector to controller gain structure.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Dictionary containing controller gain structure
        """
        if len(parameter_vector) != len(self.parameter_names):
            raise ValueError(f"Parameter vector length {len(parameter_vector)} != expected {len(self.parameter_names)}")
        
        # Create parameter dictionary
        params = dict(zip(self.parameter_names, parameter_vector))
        
        # Build controller gain structure
        gains = {}

        # SPD matrices (Gamma/Q) via Cholesky factors
        for matrix_name, config in self._spd_matrix_configs.items():
            prefix = config['prefix']
            size = config['size']
            entry_keys = get_cholesky_parameter_names(prefix, size)

            if not all(key in params for key in entry_keys):
                continue

            cholesky_entries = [float(params[key]) for key in entry_keys]
            spd_matrix = reconstruct_from_cholesky(
                cholesky_entries,
                size,
                log_diagonals=True,
            )
            gains[matrix_name] = np.matrix(spd_matrix)
        
        # Diagonal matrices from individual parameters
        for matrix_name, keys in self._diagonal_matrix_params.items():
            if keys and keys[0] in params:
                values = [params[key] for key in keys]
                gains[matrix_name] = np.matrix(np.diag(values))
        
        # Scalar parameters
        for scalar_name in self._scalar_params:
            if scalar_name in params:
                gains[scalar_name] = params[scalar_name]
        
        return gains
    
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """
        Convert controller gain structure to parameter vector.
        
        Args:
            gains: Dictionary containing controller gain structure
            
        Returns:
            List of parameter values
        """
        params = {}
        
        # Extract SPD matrices via Cholesky factorization
        for matrix_name, config in self._spd_matrix_configs.items():
            if matrix_name not in gains:
                continue

            matrix = np.array(gains[matrix_name], dtype=float)
            size = config['size']
            prefix = config['prefix']

            if matrix.shape != (size, size):
                raise ValueError(f"{matrix_name} must be a {size}x{size} matrix, got {matrix.shape}")

            entries = flatten_cholesky(matrix, log_diagonals=True)
            entry_keys = get_cholesky_parameter_names(prefix, size)
            for key, value in zip(entry_keys, entries):
                params[key] = value
        
        # Extract diagonal matrices to individual parameters
        for matrix_name, keys in self._diagonal_matrix_params.items():
            if matrix_name in gains:
                for idx, key in enumerate(keys):
                    params[key] = gains[matrix_name][idx, idx]
        
        # Extract scalar parameters
        for scalar_name in self._scalar_params:
            if scalar_name in gains:
                params[scalar_name] = gains[scalar_name]
        
        # Convert to vector in correct order
        return [params[name] for name in self.parameter_names]
    
    def get_reference_gains(self) -> Dict[str, Any]:
        """
        Get reference controller gains from the source file.

        Returns:
            Dictionary containing reference controller gain structure
        """
        result = {}
        for key, value in self._reference_gains.items():
            if isinstance(value, np.ndarray):
                result[key] = value.copy()
            else:
                result[key] = float(value)
        return result
    
    def get_parameter_summary(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Get a summary of parameter values organized by groups.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Dictionary with parameter summaries by group
        """
        params = dict(zip(self.parameter_names, parameter_vector))
        summary = {}
        
        # SPD matrices (Gamma/Q)
        if self._spd_matrix_configs:
            adaptive_summary = {}
            for matrix_name, config in self._spd_matrix_configs.items():
                prefix = config['prefix']
                size = config['size']
                entry_keys = get_cholesky_parameter_names(prefix, size)
                if all(key in params for key in entry_keys):
                    cholesky_entries = [params[key] for key in entry_keys]
                    spd_matrix = reconstruct_from_cholesky(
                        cholesky_entries,
                        size,
                        log_diagonals=True,
                    )
                    adaptive_summary[matrix_name] = {
                        'cholesky_entries': cholesky_entries,
                        'matrix': spd_matrix.tolist()
                    }
            if adaptive_summary:
                summary['spd_matrices'] = adaptive_summary
        
        # Diagonal matrices
        if self._diagonal_matrix_params:
            diag_summary = {}
            for matrix_name, keys in self._diagonal_matrix_params.items():
                if keys and keys[0] in params:
                    diag_summary[matrix_name] = [params[k] for k in keys]
            if diag_summary:
                summary['diagonal_matrices'] = diag_summary
        
        # Scalar parameters
        if self._scalar_params:
            scalar_summary = {name: params[name] for name in self._scalar_params if name in params}
            if scalar_summary:
                summary['scalars'] = scalar_summary
        
        return summary
