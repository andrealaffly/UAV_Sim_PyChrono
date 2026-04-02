"""
Pymoo-based genetic algorithm implementation
"""

from typing import List, Union, Optional, Dict, Any
import os
import pickle
import numpy as np

from pymoo.core.problem import Problem
from pymoo.optimize import minimize
from pymoo.util.ref_dirs import get_reference_directions

# Algorithms
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.algorithms.moo.spea2 import SPEA2
from pymoo.algorithms.moo.moead import MOEAD
from pymoo.algorithms.moo.sms import SMSEMOA
from pymoo.algorithms.moo.rvea import RVEA

# Seeded sampling for injecting hand-tuned gains
from .seeded_sampling import SeededSampling

from .base_ga import BaseGATuner
from ..core.parameter_bounds import ParameterBounds
from ..core.fitness_evaluator import FitnessEvaluator
from ..core.optimization_result import OptimizationResult

class PymooGATuner(BaseGATuner):
    """
    genetic algorithm tuner using Pymoo library
    include various multi-objective optimization algorithms
    """
    
    AVAILABLE_ALGORITHMS = {
        'NSGA2': (NSGA2, "Non-dominated Sorting Genetic Algorithm II"),
        'NSGA3': (NSGA3, "Non-dominated Sorting Genetic Algorithm III"),
        'SPEA2': (SPEA2, "Strength Pareto Evolutionary Algorithm 2"),
        'MOEAD': (MOEAD, "Multi-objective Evolutionary Algorithm based on Decomposition"),
        'SMS-EMOA': (SMSEMOA, "S-Metric Selection EMOA"),
        'RVEA': (RVEA, "Reference Vector guided Evolutionary Algorithm")
    }
    
    def __init__(self, 
                 parameter_bounds: ParameterBounds,
                 fitness_evaluator: FitnessEvaluator,
                 population_size: int = 100,
                 n_generations: int = 50,
                 random_seed: Optional[int] = None,
                 algorithm: str = 'NSGA2',
                 algorithm_params: Optional[Dict[str, Any]] = None,
                 n_objectives: int = 3,
                 adaptive_sampling: bool = False,
                 seed_population: Optional[List[List[float]]] = None):
        """
        Initialize Pymoo GA tuner
        
        Args:
            parameter_bounds: Parameter bounds and constraints
            fitness_evaluator: Fitness evaluation function
            population_size: Size of the population
            n_generations: Number of generations
            random_seed: Random seed for reproducibility
            algorithm: Optimization algorithm to use
            algorithm_params: Additional parameters for the algorithm
            n_objectives: Number of objectives
            adaptive_sampling: Whether to use adaptive sampling
            seed_population: Optional list of seed vectors (e.g., hand-tuned gains)
                            to include in the initial population. The first seed(s)
                            replace the first individual(s) in the randomly generated
                            population, ensuring at least one known-good solution.
        """
        super().__init__(parameter_bounds, fitness_evaluator, population_size, n_generations, random_seed)
        
        self.algorithm_name = algorithm
        self.algorithm_params = algorithm_params or {}
        self.n_objectives = n_objectives
        self.adaptive_sampling = adaptive_sampling
        self.seed_population = seed_population
        
        # Set random seed for Pymoo
        if random_seed is not None:
            np.random.seed(random_seed)
        
        # Create optimization problem 
        self.problem = PymooProblem(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            n_variables=self.parameter_bounds.n_parameters,
            n_objectives=n_objectives,
            adaptive_sampling=adaptive_sampling
        )
        
        # Create algorithm with optional seeded sampling
        self.algorithm = self._create_algorithm()
        
        # Store configuration
        self.result.optimization_parameters.update({
            'algorithm': algorithm,
            'algorithm_params': algorithm_params,
            'n_objectives': n_objectives,
            'adaptive_sampling': adaptive_sampling,
            'seed_population_count': len(seed_population) if seed_population else 0,
        })
    
    def _create_algorithm(self):
        """Create the specified optimization algorithm with optional seeded sampling."""
        if self.algorithm_name not in self.AVAILABLE_ALGORITHMS:
            raise ValueError(f"Unknown algorithm: {self.algorithm_name}. "
                          f"Available algorithms: {list(self.AVAILABLE_ALGORITHMS.keys())}")
        
        algorithm_class = self.AVAILABLE_ALGORITHMS[self.algorithm_name][0]
        
        # Setup reference directions for algorithms that need them
        if self.algorithm_name in ['NSGA3', 'MOEAD', 'RVEA']:
            # Use n_partitions from algorithm_params if provided, otherwise use default based on objectives
            n_partitions = self.algorithm_params.get('n_partitions', 12)
            ref_dirs = get_reference_directions(
                "das-dennis",
                self.n_objectives,
                n_partitions=n_partitions
            )
            self.algorithm_params['ref_dirs'] = ref_dirs
        
        # Setup seeded sampling if seed population is provided
        # This injects hand-tuned gains into the initial population
        algorithm_kwargs = {
            'pop_size': self.population_size,
            **self.algorithm_params
        }
        
        if self.seed_population and len(self.seed_population) > 0:
            seeded_sampling = SeededSampling(seeds=self.seed_population)
            algorithm_kwargs['sampling'] = seeded_sampling
            print(f"[SEEDED POPULATION] Will inject {len(self.seed_population)} hand-tuned solution(s) into initial population")
        
        return algorithm_class(**algorithm_kwargs)
    
    def optimize(self,
                verbose: bool = True,
                save_intermediate: bool = False,
                intermediate_save_path: str = "intermediate_results") -> OptimizationResult:
        """
        Run the Pymoo optimization
        
        Args:
            verbose: Whether to print progress information
            save_intermediate: Whether to save intermediate results
            intermediate_save_path: Path to save intermediate results
            
        Returns:
            OptimizationResult object with enhanced metrics
        """
        if verbose:
            print(f"Starting Pymoo {self.algorithm_name} optimization")
            print(f"Algorithm description: {self.AVAILABLE_ALGORITHMS[self.algorithm_name][1]}")
            print("\nConfiguration:")
            for key, value in self.result.optimization_parameters.items():
                if key != 'algorithm_params':  # Skip detailed params
                    print(f"  {key}: {value}")
            print("=" * 50)
        
        # Run optimization with enhanced monitoring
        res = minimize(
            self.problem,
            self.algorithm,
            ('n_gen', self.n_generations),
            seed=self.random_seed,
            verbose=verbose,
            save_history=save_intermediate
        )
        
        # Process and store results with enhanced metrics
        self._process_results(res)
        
        # Save intermediate results if requested
        if save_intermediate:
            self._save_optimization_history(res, intermediate_save_path)
        
        return self.result

    # The following abstract methods are unused because PymooGATuner overrides
    # optimize with its own flow, but they are implemented to satisfy the base
    # class interface.
    def _initialize_population(self):
        return []

    def _select_parents(self, population, fitnesses):
        return []

    def _crossover(self, parents):
        return []

    def _mutate(self, offspring):
        return []

    def _survive(self, population, offspring, population_fitnesses, offspring_fitnesses):
        return []
    
    def _process_results(self, res):
        """Process optimization results with enhanced metrics."""
        # Extract final population and their fitnesses
        last_population = [ind.X for ind in res.pop]
        last_fitnesses = [ind.F for ind in res.pop]
        
        # Extract Pareto front from Pymoo
        raw_pareto_front = [ind.X for ind in res.opt]
        raw_pareto_fitnesses = [ind.F for ind in res.opt]
        
        # Filter out invalid solutions (inf/nan values indicate failed/diverged simulations)
        pareto_front = []
        pareto_fitnesses = []
        n_filtered = 0
        
        for params, fitness in zip(raw_pareto_front, raw_pareto_fitnesses):
            is_valid = all(
                not (np.isnan(f) or np.isinf(f)) for f in fitness
            )
            if is_valid:
                pareto_front.append(params)
                pareto_fitnesses.append(fitness)
            else:
                n_filtered += 1
        
        if n_filtered > 0:
            print(f"\n[Pareto Filter] Removed {n_filtered} invalid solutions (inf/nan) from Pareto front")
            print(f"[Pareto Filter] Valid Pareto solutions: {len(pareto_front)} / {len(raw_pareto_front)}")
        
        if not pareto_front and raw_pareto_front:
            print("Warning: All Pareto solutions were invalid (inf/nan). No valid Pareto front.")

        # Use best-performing solutions (Pareto set for multi-objective, best single for single-objective)
        if pareto_front:
            final_population = pareto_front
            final_fitnesses = pareto_fitnesses
        else:
            # Single-objective or empty Pareto: take best of last generation
            best_idx = int(np.argmin([f[0] if isinstance(f, (list, np.ndarray)) else f for f in last_fitnesses]))
            final_population = [last_population[best_idx]]
            final_fitnesses = [last_fitnesses[best_idx]]
            pareto_front = None
            pareto_fitnesses = None
        
        # Store results with additional metrics
        self.result.set_final_results(
            population=final_population,
            fitnesses=final_fitnesses,
            pareto_front=pareto_front,
            pareto_fitnesses=pareto_fitnesses
        )

        # Track best solutions explicitly for downstream consumers (MAT export, summaries)
        self.result.best_individuals = final_population
        self.result.best_fitnesses = final_fitnesses
        
        # Store additional metrics
        metrics = {}
        if hasattr(res, "metrics") and isinstance(res.metrics, dict):
            metrics['convergence_metric'] = res.metrics.get('conv_metric')
            metrics['diversity_metric'] = res.metrics.get('diversity')
            metrics['hypervolume'] = res.metrics.get('hv')
        else:
            metrics['convergence_metric'] = None
            metrics['diversity_metric'] = None
            metrics['hypervolume'] = None

        metrics['execution_time'] = getattr(res, 'exec_time', None)
        metrics['n_evaluations'] = getattr(res, 'n_eval', None)
        metrics['last_population'] = last_population
        metrics['last_fitnesses'] = last_fitnesses

        self.result.additional_metrics = metrics
    
    def _save_optimization_history(self, res, save_path: str):
        """Save optimization history"""
        
        os.makedirs(save_path, exist_ok=True)
        
        # Save evolution history
        history_file = os.path.join(save_path, f"{self.algorithm_name}_history.pkl")
        with open(history_file, 'wb') as f:
            pickle.dump(res.history, f)
        
        # Save metrics history
        metrics_file = os.path.join(save_path, f"{self.algorithm_name}_metrics.pkl")
        metrics_history = {
            'gen': [],
            'conv_metric': [],
            'diversity': [],
            'hv': []  # hypervolume
        }
        
        for entry in res.history:
            metrics_history['gen'].append(entry.n_gen)
            if hasattr(entry, 'metrics') and isinstance(entry.metrics, dict):
                metrics_history['conv_metric'].append(entry.metrics.get('conv_metric'))
                metrics_history['diversity'].append(entry.metrics.get('diversity'))
                metrics_history['hv'].append(entry.metrics.get('hv'))
            else:
                metrics_history['conv_metric'].append(None)
                metrics_history['diversity'].append(None)
                metrics_history['hv'].append(None)
        
        with open(metrics_file, 'wb') as f:
            pickle.dump(metrics_history, f)
    
    @property
    def available_algorithms(self) -> Dict[str, str]:
        """Get available algorithms with their descriptions."""
        return {name: desc for name, (_, desc) in self.AVAILABLE_ALGORITHMS.items()}
    
    def set_algorithm(self, 
                     algorithm: str,
                     algorithm_params: Optional[Dict[str, Any]] = None):
        """
        Change the optimization algorithm during runtime.
        
        Args:
            algorithm: Name of the algorithm to use
            algorithm_params: Additional parameters for the algorithm
        """
        self.algorithm_name = algorithm
        self.algorithm_params = algorithm_params or {}
        self.algorithm = self._create_algorithm()
        
        # Update configuration
        self.result.optimization_parameters.update({
            'algorithm': algorithm,
            'algorithm_params': algorithm_params
        })


class PymooProblem(Problem):
    """
    Enhanced Pymoo optimization problem with additional features.
    """
    
    def __init__(self,
                 parameter_bounds: ParameterBounds,
                 fitness_evaluator: FitnessEvaluator,
                 n_variables: int,
                 n_objectives: int = 4,
                 adaptive_sampling: bool = False):
        """
        Initialize the optimization problem with enhanced features.
        
        Args:
            parameter_bounds: Parameter bounds and constraints
            fitness_evaluator: Fitness evaluation function
            n_variables: Number of variables to optimize
            n_objectives: Number of objectives
            adaptive_sampling: Whether to use adaptive sampling
        """
        self.parameter_bounds = parameter_bounds
        self.fitness_evaluator = fitness_evaluator
        self.adaptive_sampling = adaptive_sampling
        
        # Initialize problem with enhanced features
        # Set elementwise_evaluation=False to enable batch/parallel evaluation
        super().__init__(
            n_var=n_variables,
            n_obj=n_objectives,
            n_constr=0,
            xl=parameter_bounds.lower_bounds,
            xu=parameter_bounds.upper_bounds,
            elementwise_evaluation=False  # False = batch mode for parallel evaluation
        )
        
        # Initialize adaptive sampling if enabled
        if adaptive_sampling:
            self.sampling_history = []
            self.sampling_metrics = {}
    
    def _evaluate(self, x, out, *args, **kwargs):
        """
        Evaluate the objective functions with enhanced features.
        
        Args:
            x: Individual to evaluate or population matrix
            out: Output dictionary for objective values
        """
        if isinstance(x, np.ndarray) and x.ndim > 1:
            # Vectorized call from pymoo: evaluate entire population in parallel
            population = [row.tolist() if isinstance(row, np.ndarray) else row for row in x]
            
            # Use evaluate_population for parallel evaluation
            fitness_results = self.fitness_evaluator.evaluate_population(population)
            
            # Convert results to numpy matrix
            fitness_matrix = []
            for fitness in fitness_results:
                if not isinstance(fitness, (list, np.ndarray)):
                    fitness = [fitness]
                fitness_array = np.array(fitness, dtype=float).flatten()
                if fitness_array.size != self.n_obj:
                    raise ValueError(
                        f"Fitness evaluator returned {fitness_array.size} objectives, "
                        f"expected {self.n_obj}"
                    )
                fitness_matrix.append(fitness_array)
            
            out["F"] = np.vstack(fitness_matrix)
        else:
            out["F"] = self._evaluate_single(x)
    
    def _evaluate_single(self, x):
        if self.adaptive_sampling:
            x = self._apply_adaptive_sampling(x)

        individual = x.tolist() if isinstance(x, np.ndarray) else x
        fitness = self.fitness_evaluator.evaluate_individual(individual)

        if not isinstance(fitness, (list, np.ndarray)):
            fitness = [fitness]

        fitness_array = np.array(fitness, dtype=float).flatten()
        if fitness_array.size != self.n_obj:
            raise ValueError(
                f"Fitness evaluator returned {fitness_array.size} objectives, "
                f"expected {self.n_obj}"
            )

        if self.adaptive_sampling:
            self._update_sampling_metrics(np.array(individual), fitness_array)

        return fitness_array
    def _apply_adaptive_sampling(self, x):
        """Apply adaptive sampling strategy to improve exploration."""
        if len(self.sampling_history) > 0:
            # Calculate distance to previous samples
            distances = [np.linalg.norm(x - prev) for prev in self.sampling_history[-10:]]
            
            # If too close to previous samples, apply perturbation
            if min(distances) < 0.1:
                perturbation = np.random.normal(0, 0.1, size=len(x))
                x = np.clip(x + perturbation, self.xl, self.xu)
        
        return x
    
    def _update_sampling_metrics(self, x, fitness):
        """Update metrics for adaptive sampling."""
        self.sampling_history.append(x)
        
        if len(self.sampling_history) > 100:
            self.sampling_history.pop(0)
        
        # Update sampling metrics
        self.sampling_metrics['diversity'] = self._calculate_diversity()
        self.sampling_metrics['convergence'] = self._calculate_convergence(fitness)
    
    def _calculate_diversity(self) -> float:
        """Calculate population diversity metric."""
        if len(self.sampling_history) < 2:
            return 0.0
        
        distances = []
        for i in range(len(self.sampling_history)):
            for j in range(i + 1, len(self.sampling_history)):
                distances.append(np.linalg.norm(
                    self.sampling_history[i] - self.sampling_history[j]
                ))
        
        return np.mean(distances)
    
    def _calculate_convergence(self, current_fitness) -> float:
        """Calculate convergence metric."""
        if not hasattr(self, 'best_fitness'):
            self.best_fitness = current_fitness
            return 0.0
        
        improvement = np.mean(self.best_fitness) - np.mean(current_fitness)
        if improvement > 0:
            self.best_fitness = current_fitness
        
        return improvement
