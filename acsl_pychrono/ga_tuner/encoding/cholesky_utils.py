"""
Utilities for parameterizing symmetric positive definite (SPD) matrices via Cholesky factors.
"""

from typing import Callable, Dict, List, Tuple
import numpy as np


def get_cholesky_parameter_names(prefix: str, size: int) -> List[str]:
    """Generate ordered parameter names for the lower-triangular entries of a Cholesky factor."""
    return [f"{prefix}_L{i + 1}{j + 1}" for i in range(size) for j in range(i + 1)]


def flatten_cholesky(matrix: np.ndarray, *, log_diagonals: bool = False) -> np.ndarray:
    """Return the lower-triangular Cholesky entries, optionally storing diagonal terms in log space."""
    symmetrical = 0.5 * (matrix + matrix.T)
    try:
        chol = np.linalg.cholesky(symmetrical)
    except np.linalg.LinAlgError:
        eigvals, _ = np.linalg.eigh(symmetrical)
        min_eig = np.min(eigvals)
        jitter = abs(min_eig) + 1e-9 if min_eig <= 0.0 else 1e-9
        chol = np.linalg.cholesky(symmetrical + np.eye(symmetrical.shape[0]) * jitter)

    entries: List[float] = []
    for i in range(chol.shape[0]):
        for j in range(i + 1):
            value = float(chol[i, j])
            if log_diagonals and i == j:
                value = np.log(max(value, 1e-12))
            entries.append(value)
    return np.array(entries, dtype=float)


def reconstruct_from_cholesky(
    entries: List[float],
    size: int,
    *,
    log_diagonals: bool = False,
    min_diagonal: float = 1e-8
) -> np.ndarray:
    """Rebuild an SPD matrix from lower-triangular Cholesky entries."""
    expected = size * (size + 1) // 2
    if len(entries) != expected:
        raise ValueError(f"Expected {expected} Cholesky parameters for size {size}, got {len(entries)}")

    chol = np.zeros((size, size), dtype=float)
    idx = 0
    for i in range(size):
        for j in range(i + 1):
            value = float(entries[idx])
            idx += 1
            if i == j:
                actual = np.exp(value) if log_diagonals else value
                actual = max(actual, min_diagonal)
                chol[i, j] = actual
            else:
                chol[i, j] = value
    return np.matmul(chol, chol.T)


def build_cholesky_parameter_bounds(
    baseline_matrix: np.ndarray,
    prefix: str,
    diag_bound_fn: Callable[[float], List[float]],
    *,
    off_diag_bound_fn: Callable[[float], List[float]] = None,
    off_diag_factor: float = 5.0,
    min_off_diag_span: float = 1.0,
    log_diagonals: bool = False,
) -> Tuple[Dict[str, List[float]], List[str]]:
    """Build GA parameter bounds for the Cholesky entries of an SPD matrix.
    
    Args:
        baseline_matrix: The baseline SPD matrix
        prefix: Parameter name prefix
        diag_bound_fn: Function that takes diagonal value and returns [lower, upper] bounds
        off_diag_bound_fn: Optional function that takes off-diagonal value and returns [lower, upper] bounds.
                          If provided, off_diag_factor and min_off_diag_span are ignored.
        off_diag_factor: Factor for relative off-diagonal bounds (used if off_diag_bound_fn is None)
        min_off_diag_span: Minimum span for off-diagonal bounds (used if off_diag_bound_fn is None)
        log_diagonals: Whether diagonal values are in log space
    """
    matrix = np.asarray(baseline_matrix, dtype=float)
    if matrix.shape[0] != matrix.shape[1]:
        raise ValueError(f"Matrix for prefix '{prefix}' must be square, got {matrix.shape}")

    size = matrix.shape[0]
    cholesky_entries = flatten_cholesky(matrix, log_diagonals=log_diagonals)
    entry_names = get_cholesky_parameter_names(prefix, size)
    if len(entry_names) != len(cholesky_entries):
        raise ValueError(f"Cholesky entry mismatch for prefix '{prefix}'")

    bounds: Dict[str, List[float]] = {}
    ordered_names: List[str] = []
    idx = 0
    for i in range(size):
        for j in range(i + 1):
            name = entry_names[idx]
            value = float(cholesky_entries[idx])
            idx += 1

            if i == j:
                bounds[name] = diag_bound_fn(value)
            else:
                if off_diag_bound_fn is not None:
                    # Use absolute bounds function
                    bounds[name] = off_diag_bound_fn(value)
                else:
                    # Use relative bounds (original behavior)
                    span = max(abs(value) * off_diag_factor, min_off_diag_span)
                    bounds[name] = [value - span, value + span]

            ordered_names.append(name)

    return bounds, ordered_names
