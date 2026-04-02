"""
Metric normalization utilities for GA optimization.
"""

from typing import Dict, List, Tuple, Optional
import numpy as np


class MetricNormalizer:
    """
    Normalizer for metrics using reference-relative normalization with capping.
    
    Normalization: normalized = raw / reference (capped to MAX_NORMALIZED)
    
    - Extreme/unstable values -> capped to prevent dominating optimization
    
    Args:
        metric_names: List of metric names for this normalizer
        max_normalized: Cap for normalized values (default: 100.0)
    """
    
    MIN_REFERENCE = 1e-6  # Floor to prevent division by tiny values
    DEFAULT_MAX_NORMALIZED = 100.0  # Cap normalized values to this
    
    def __init__(self, metric_names: List[str] = None, max_normalized: float = None):
        self.reference_values: Optional[np.ndarray] = None
        self.metric_names = metric_names or []
        self.n_metrics = len(self.metric_names)
        self.fitted = False
        self.max_normalized = max_normalized if max_normalized is not None else self.DEFAULT_MAX_NORMALIZED
    
    def fit_from_reference(self, reference_metrics: np.ndarray):
        """
        Fit the normalizer using reference metrics (from default gains) as reference.
        
        Normalized value = raw_metric / max(reference_metric, MIN_REFERENCE)
        
        Args:
            reference_metrics: Array of shape (n_metrics,) from evaluating default gains
        """
        reference_metrics = np.atleast_1d(reference_metrics).flatten()
        
        if len(reference_metrics) != self.n_metrics:
            raise ValueError(f"Expected {self.n_metrics} metrics, got {len(reference_metrics)}")
        
        # Apply floor to prevent division by tiny values
        self.reference_values = np.maximum(np.abs(reference_metrics), self.MIN_REFERENCE)
        self.fitted = True
        
        print(f"\n[METRIC NORMALIZER] Fitted with reference values (max_normalized={self.max_normalized}):")
        for i, name in enumerate(self.metric_names):
            print(f"  {name:40s}: {self.reference_values[i]:.6f}")
    
    def fit(self, metrics_array: np.ndarray):
        """
        Fit the normalizer by computing mean of valid metrics as reference.
        
        This is a fallback method if default gains are not available.
        Prefer fit_from_reference() for more consistent normalization.
        
        Args:
            metrics_array: Array of shape (n_samples, n_metrics) containing all metric values
        """
        if metrics_array.ndim != 2:
            raise ValueError(f"Expected 2D array, got shape {metrics_array.shape}")
        
        if metrics_array.shape[1] != self.n_metrics:
            raise ValueError(f"Expected {self.n_metrics} metrics, got {metrics_array.shape[1]}")
        
        # Filter out inf values and compute mean as reference
        valid_mask = ~np.isinf(metrics_array)
        reference = np.zeros(self.n_metrics)
        
        for i in range(self.n_metrics):
            valid_values = metrics_array[valid_mask[:, i], i]
            if len(valid_values) > 0:
                reference[i] = np.mean(valid_values)
            else:
                reference[i] = 1.0  # Default if all values are inf
        
        # Apply floor to prevent division by tiny values
        self.reference_values = np.maximum(np.abs(reference), self.MIN_REFERENCE)
        self.fitted = True
        
        print(f"\n[METRIC NORMALIZER] Fitted with mean reference values (max_normalized={self.max_normalized}):")
        for i, name in enumerate(self.metric_names):
            print(f"  {name:40s}: {self.reference_values[i]:.6f}")
    
    def transform(self, metrics_array: np.ndarray) -> np.ndarray:
        """
        Normalize metrics by dividing by reference values, with capping.

        Args:
            metrics_array: Array of shape (n_samples, n_metrics) or (n_metrics,)
            
        Returns:
            Normalized array with same shape as input, capped to max_normalized
        """
        if not self.fitted:
            raise RuntimeError("Normalizer must be fitted before transform. Call fit() or fit_from_reference() first.")
        
        original_shape = metrics_array.shape
        metrics_array = np.atleast_2d(metrics_array).copy()
        
        if metrics_array.shape[1] != self.n_metrics:
            raise ValueError(f"Expected {self.n_metrics} metrics, got {metrics_array.shape[1]}")
        
        # Handle inf/nan values BEFORE division - set to a value that will be capped
        inf_nan_mask = np.isinf(metrics_array) | np.isnan(metrics_array)
        metrics_array[inf_nan_mask] = self.max_normalized * self.reference_values[np.where(inf_nan_mask)[1]]
        
        # Divide by reference with MIN_REFERENCE floor applied
        normalized = metrics_array / self.reference_values
        
        # Cap normalized values to prevent extreme values from dominating
        normalized = np.clip(normalized, 0.0, self.max_normalized)
        
        # Restore original shape if input was 1D
        if len(original_shape) == 1:
            return normalized[0]
        
        return normalized
    
    def fit_transform(self, metrics_array: np.ndarray) -> np.ndarray:
        """
        Fit the normalizer and transform in one step.
        
        Args:
            metrics_array: Array of shape (n_samples, n_metrics)
            
        Returns:
            Normalized array
        """
        self.fit(metrics_array)
        return self.transform(metrics_array)
    
    def inverse_transform(self, normalized_array: np.ndarray) -> np.ndarray:
        """Convert normalized metrics back to raw values: raw = normalized * reference."""
        if not self.fitted:
            return normalized_array
        normalized_array = np.atleast_1d(normalized_array)
        return normalized_array * self.reference_values

    def get_reference_values(self) -> Dict[str, float]:
        """
        Get the reference values used for normalization.
        
        Returns:
            Dictionary mapping metric names to reference values
        """
        if not self.fitted:
            raise RuntimeError("Normalizer must be fitted first. Call fit() or fit_from_reference() first.")
        
        return {
            name: float(self.reference_values[i])
            for i, name in enumerate(self.metric_names)
        }
