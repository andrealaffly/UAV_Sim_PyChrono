"""
Seeded sampling strategy for pymoo.

Includes the hand-tuned default gains from the controller's gains class
in the initial population to help the GA start with at least one feasible solution.
"""

from typing import List, Optional, Sequence
import numpy as np
from pymoo.core.sampling import Sampling


class SeededSampling(Sampling):
    """
    Custom sampling that seeds the initial population with known good solutions.
    
    The first individual(s) in the population are set to the provided seed values
    (e.g., hand-tuned gains from mrac_gains.py). The remaining individuals are
    randomly sampled within the parameter bounds.
    
    This helps the GA start with at least one feasible solution that can survive
    the simulation, providing a gradient for the optimization to follow.
    """
    
    def __init__(self, seeds: Optional[Sequence[Sequence[float]]] = None):
        """
        Initialize seeded sampling.
        
        Args:
            seeds: List of seed vectors to include in the initial population.
                   Each seed should be a sequence of parameter values matching
                   the problem dimensions. The first seeds replace the first
                   individuals in the randomly generated population.
        """
        super().__init__()
        self.seeds = [np.asarray(s, dtype=float) for s in seeds] if seeds else []
    
    def _do(self, problem, n_samples, **kwargs):
        """
        Generate initial population with seeded individuals.
        
        Args:
            problem: The pymoo Problem instance (provides xl, xu bounds)
            n_samples: Number of individuals to generate
            
        Returns:
            np.ndarray of shape (n_samples, n_var) containing the initial population
        """
        xl = problem.xl
        xu = problem.xu
        n_var = problem.n_var
        
        # Generate random population
        X = np.random.uniform(xl, xu, size=(n_samples, n_var))
        
        # Replace first individuals with seeds (if any)
        for i, seed in enumerate(self.seeds):
            if i >= n_samples:
                print(f"[SEEDED SAMPLING] Warning: More seeds ({len(self.seeds)}) than population size ({n_samples})")
                break
            
            if len(seed) != n_var:
                print(f"[SEEDED SAMPLING] Warning: Seed {i} has {len(seed)} values, expected {n_var}. Skipping.")
                continue
            
            # Clip seed to bounds (in case hand-tuned values are slightly outside)
            clipped_seed = np.clip(seed, xl, xu)
            X[i] = clipped_seed
            
            # Check if clipping changed the values
            if not np.allclose(seed, clipped_seed, rtol=1e-5):
                print(f"[SEEDED SAMPLING] Seed {i} was clipped to fit within bounds")
        
        if self.seeds:
            print(f"[SEEDED SAMPLING] Injected {min(len(self.seeds), n_samples)} seed(s) into initial population")
        
        return X


def create_seeded_sampling_from_tuning(
    tuning,
    tuned_indices: Optional[Sequence[int]] = None,
) -> SeededSampling:
    """
    Create a SeededSampling instance using default gains from a tuning interface.
    
    Args:
        tuning: A tuning interface instance (e.g., MRACTuning) that has
                get_default_gains() and gains_to_vector() methods.
        tuned_indices: Optional indices of parameters being tuned (for partial tuning).
                       If None, the full parameter vector is used as the seed.
    
    Returns:
        SeededSampling instance with the default gains as a seed.
    """
    # Get default gains and convert to vector
    default_gains = tuning.get_default_gains()
    full_vector = tuning.gains_to_vector(default_gains)
    
    # Extract only the tuned parameters if partial tuning
    if tuned_indices is not None:
        seed_vector = [full_vector[i] for i in tuned_indices]
    else:
        seed_vector = full_vector
    
    print(f"[SEEDED SAMPLING] Created seed from default gains ({len(seed_vector)} parameters)")
    
    return SeededSampling(seeds=[seed_vector])


def create_seeded_sampling_from_vector(
    seed_vector: Sequence[float],
    additional_seeds: Optional[Sequence[Sequence[float]]] = None,
) -> SeededSampling:
    """
    Create a SeededSampling instance from a parameter vector.
    
    Args:
        seed_vector: The primary seed vector (e.g., hand-tuned gains).
        additional_seeds: Optional additional seed vectors to include.
    
    Returns:
        SeededSampling instance with the provided seeds.
    """
    all_seeds = [seed_vector]
    if additional_seeds:
        all_seeds.extend(additional_seeds)
    
    return SeededSampling(seeds=all_seeds)
