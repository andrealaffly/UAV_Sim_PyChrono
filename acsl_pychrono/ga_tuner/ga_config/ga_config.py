from dataclasses import dataclass, field
from typing import Any, Dict, Optional, List


# Custom reference vector for iterative tuning. Set to None to use default gains from mrac_gains.py
# NOTE: This must match the number of parameters selected in param_config.py
_CUSTOM_REFERENCE_VECTOR = None


@dataclass
class SurvivalPenaltySettings:
    """Configuration for survival time penalty (penalizes crashed simulations).
    
    The penalty formula is: penalty = base * (1 + time_deficit_ratio)
    Where time_deficit_ratio = (expected_duration - survival_time) / expected_duration
    
    This creates penalties in range [base, 2*base]:
    - Immediate crash (t=0): penalty = 2 * base (e.g., 2000)
    - Half-way crash: penalty = 1.5 * base (e.g., 1500)
    - Near completion: penalty = ~base (e.g., ~1000)
    
    Default base=1000 ensures crashed solutions are dominated by completed ones
    (normalized metrics are typically 0-100) while maintaining numeric stability.
    """
    
    enabled: bool = True  # Enable survival time penalty
    penalty_base: float = 1000.0  # Base penalty value (should be >> max normalized metric ~100)
    time_tolerance: float = 0.5  # Tolerance in seconds for considering simulation complete
    expected_duration: Optional[float] = None  # Expected simulation duration (None = 31.5s)


@dataclass
class GAConfig:
    """Central configuration for GA tuning runs."""

    controller_type: str = "PID"  # 'PID', 'MRAC'
    algorithm: str = "PYMOO"  # 'DEAP' or 'PYMOO'

    population_size: int = 3
    num_generations: int = 2
    crossover_rate: float = 0.6
    mutation_rate: float = 0.05

    selection_method: str = "tournament"
    tournament_size: int = 5
    random_seed: Optional[int] = None
    verbose: bool = True  # Print generation progress during optimization

    # Search space configuration
    # - 'local': Relative bounds around reference (±10× scaling) - for fine-tuning existing gains
    # - 'global': Absolute bounds for exploring diverse parameter regions
    search_space_type: str = "local"  # Changed to 'local' for refinement around global search results
    
    # Custom reference for iterative tuning (use tuned parameters as new reference point for fine-tuning)
    # Set to None to use default gains from mrac_gains.py, set to a list to use custom reference (see _CUSTOM_REFERENCE_VECTOR above)
    custom_reference_vector: Optional[list] = None  # Set to _CUSTOM_REFERENCE_VECTOR to use it
    
    # =========================================================================
    # Seeding Configuration
    # =========================================================================
    # Whether to inject hand-tuned default gains into the initial population
    # This helps the GA start with at least one known-good (feasible) solution
    seed_with_default_gains: bool = True
    
    # Additional custom seed vectors to inject into the initial population
    # Each seed should be a list of parameter values matching the tuned parameter dimensions
    additional_seed_vectors: Optional[List[List[float]]] = None
    
    # =========================================================================
    # Survival Penalty Configuration
    # =========================================================================
    # Survival penalty settings for penalizing crashed simulations
    # Gives the GA a gradient to follow: longer survival = better, but any completed
    # simulation is better than any crashed simulation
    survival_penalty: SurvivalPenaltySettings = field(default_factory=SurvivalPenaltySettings)

    # Pymoo-specific parameters
    pymoo_variant: str = "NSGA3"
    # For NSGA3 with many objectives: manually set partitions to reduce reference directions
    pymoo_algorithm_params: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        self.algorithm = self.algorithm.upper()
        # Set default n_partitions for NSGA3 if not specified
        if self.pymoo_variant == "NSGA3" and self.pymoo_algorithm_params is None:
            self.pymoo_algorithm_params = {"n_partitions": 3}  # Reduces ref_dirs from 6188 to ~84

