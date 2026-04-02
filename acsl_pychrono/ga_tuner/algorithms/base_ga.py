"""
Base genetic algorithm class providing common functionality.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional, Union, Tuple
import numpy as np
import time

from ..core.parameter_bounds import ParameterBounds
from ..core.fitness_evaluator import FitnessEvaluator
from ..core.optimization_result import OptimizationResult

class BaseGATuner(ABC):
    """
    Abstract base class for genetic algorithm tuners.
    This provides a common interface for different GA implementations.
    """
    
    def __init__(self, 
                 parameter_bounds: ParameterBounds,
                 fitness_evaluator: FitnessEvaluator,
                 population_size: int = 100,
                 n_generations: int = 50,
                 random_seed: Optional[int] = None):
        """
        Initialize base GA tuner.
        
        Args:
            parameter_bounds: Parameter bounds and constraints
            fitness_evaluator: Fitness evaluation function
            population_size: Size of the population
            n_generations: Number of generations
            random_seed: Random seed for reproducibility
        """
        self.parameter_bounds = parameter_bounds
        self.fitness_evaluator = fitness_evaluator
        self.population_size = population_size
        self.n_generations = n_generations
        self.random_seed = random_seed
        
        # Set random seed
        if random_seed is not None:
            np.random.seed(random_seed)
        
        # Results storage
        self.result = OptimizationResult(
            algorithm_name=self.__class__.__name__,
            parameter_bounds=parameter_bounds
        )
        
        # Callbacks
        self.generation_callback = None
        self.evaluation_callback = None
    
    @abstractmethod
    def _initialize_population(self) -> List[np.ndarray]:
        """Initialize the initial population."""
        pass
    
    @abstractmethod
    def _select_parents(self, population: List[np.ndarray], 
                       fitnesses: List[Union[float, List[float]]]) -> List[np.ndarray]:
        """Select parents for reproduction."""
        pass
    
    @abstractmethod
    def _crossover(self, parents: List[np.ndarray]) -> List[np.ndarray]:
        """Perform crossover to create offspring."""
        pass
    
    @abstractmethod
    def _mutate(self, offspring: List[np.ndarray]) -> List[np.ndarray]:
        """Perform mutation on offspring."""
        pass
    
    @abstractmethod
    def _survive(self, population: List[np.ndarray],
                 offspring: List[np.ndarray],
                 population_fitnesses: List[Union[float, List[float]]],
                 offspring_fitnesses: List[Union[float, List[float]]]) -> List[np.ndarray]:
        """Select individuals to survive to next generation."""
        pass
    
    def set_generation_callback(self, callback):
        """Set callback function to be called after each generation."""
        self.generation_callback = callback
    
    def set_evaluation_callback(self, callback):
        """Set callback function to be called after each evaluation."""
        self.evaluation_callback = callback
    
    def optimize(self, 
                verbose: bool = True,
                save_intermediate: bool = False,
                intermediate_save_path: str = "intermediate_results") -> OptimizationResult:
        """
        Run the genetic algorithm optimization.
        
        Args:
            verbose: Whether to print progress information
            save_intermediate: Whether to save intermediate results
            intermediate_save_path: Path to save intermediate results
            
        Returns:
            OptimizationResult object
        """
        if verbose:
            print(f"Starting {self.__class__.__name__} optimization")
            print(f"Population size: {self.population_size}")
            print(f"Generations: {self.n_generations}")
            print(f"Parameters: {self.parameter_bounds.n_parameters}")
            print("=" * 50)
        
        start_time = time.time()
        
        # Initialize population
        population = self._initialize_population()
        
        # Evaluate initial population
        if verbose:
            print("Evaluating initial population...")
        
        fitnesses = self.fitness_evaluator.evaluate_population(population)
        
        # Store initial results
        self._store_generation_results(0, population, fitnesses)
        
        # Main optimization loop
        for generation in range(1, self.n_generations + 1):
            if verbose:
                print(f"\n--- Generation {generation}/{self.n_generations} ---")
            
            # Select parents
            parents = self._select_parents(population, fitnesses)
            
            # Create offspring through crossover
            offspring = self._crossover(parents)
            
            # Mutate offspring
            offspring = self._mutate(offspring)
            
            # Evaluate offspring
            if verbose:
                print(f"Evaluating {len(offspring)} offspring...")
            
            offspring_fitnesses = self.fitness_evaluator.evaluate_population(offspring)
            
            # Select survivors; allow _survive to optionally return (population, fitnesses)
            survival_result = self._survive(population, offspring, fitnesses, offspring_fitnesses)
            if isinstance(survival_result, tuple) and len(survival_result) == 2:
                population, fitnesses = survival_result
            else:
                population = survival_result
                # Re-evaluate only if fitnesses were not provided
                fitnesses = self.fitness_evaluator.evaluate_population(population)
            
            # Store generation results
            self._store_generation_results(generation, population, fitnesses)
            
            # Call generation callback
            if self.generation_callback:
                self.generation_callback(generation, population, fitnesses)
            
            # Save intermediate results
            if save_intermediate and generation % 10 == 0:
                self._save_intermediate_results(intermediate_save_path, generation)
            
            # Print progress
            if verbose:
                self._print_generation_progress(generation, fitnesses)
        
        # Store final results
        self._store_final_results(population, fitnesses)
        
        # Record computation time
        self.result.computation_time = time.time() - start_time
        
        if verbose:
            print(f"\nOptimization completed in {self.result.computation_time:.2f} seconds")
            self.result.print_summary()
        
        return self.result
    
    def _store_generation_results(self, 
                                generation: int,
                                population: List[np.ndarray],
                                fitnesses: List[Union[float, List[float]]]):
        """Store results from a single generation."""
        # Find best individual
        if isinstance(fitnesses[0], (list, np.ndarray)):
            # Multi-objective: use first objective for now
            best_idx = np.argmin([f[0] if isinstance(f, (list, np.ndarray)) else f for f in fitnesses])
        else:
            # Single objective
            best_idx = np.argmin(fitnesses)
        
        best_individual = population[best_idx]
        best_fitness = fitnesses[best_idx]
        
        # Store in result object
        self.result.add_generation(generation, population, fitnesses, best_individual, best_fitness)
    
    def _store_final_results(self, 
                           population: List[np.ndarray],
                           fitnesses: List[Union[float, List[float]]]):
        """Store final optimization results."""
        # Determine if this is single or multi-objective
        if isinstance(fitnesses[0], (list, np.ndarray)):
            # Multi-objective: find Pareto front
            pareto_front, pareto_fitnesses = self._find_pareto_front(population, fitnesses)
            self.result.set_final_results(population, fitnesses, pareto_front, pareto_fitnesses)
        else:
            # Single objective
            self.result.set_final_results(population, fitnesses)
    
    def _find_pareto_front(self, 
                          population: List[np.ndarray],
                          fitnesses: List[List[float]]) -> Tuple[List[np.ndarray], List[List[float]]]:
        """Find Pareto optimal solutions (for minimization problems)."""
        pareto_front = []
        pareto_fitnesses = []
        
        for i, (individual, fitness) in enumerate(zip(population, fitnesses)):
            is_pareto_optimal = True
            
            for j, (other_individual, other_fitness) in enumerate(zip(population, fitnesses)):
                if i == j:
                    continue
                
                # Check if other individual dominates this one (for minimization)
                # other dominates if it's better or equal in all objectives and strictly better in at least one
                at_least_one_better = False
                all_better_or_equal = True
                
                for k in range(len(fitness)):
                    if other_fitness[k] < fitness[k]:
                        at_least_one_better = True
                    elif other_fitness[k] > fitness[k]:
                        all_better_or_equal = False
                        break
                
                if all_better_or_equal and at_least_one_better:
                    is_pareto_optimal = False
                    break
            
            if is_pareto_optimal:
                pareto_front.append(individual)
                pareto_fitnesses.append(fitness)
        
        return pareto_front, pareto_fitnesses
    
    def _print_generation_progress(self, generation: int, fitnesses: List[Union[float, List[float]]]):
        """Print progress information for current generation."""
        if isinstance(fitnesses[0], (list, np.ndarray)):
            # Multi-objective
            best_fitness = fitnesses[np.argmin([f[0] for f in fitnesses])]
            print(f"Best fitness: {best_fitness}")
        else:
            # Single objective
            best_fitness = min(fitnesses)
            print(f"Best fitness: {best_fitness:.6f}")
    
    def _save_intermediate_results(self, save_path: str, generation: int):
        """Save intermediate results."""
        import os
        os.makedirs(save_path, exist_ok=True)
        
        filename = f"intermediate_gen_{generation}.pkl"
        filepath = os.path.join(save_path, filename)
        
        # Create a temporary result object with current progress
        temp_result = OptimizationResult(
            algorithm_name=self.__class__.__name__,
            parameter_bounds=self.parameter_bounds
        )
        
        # Copy current progress
        temp_result.population_history = self.result.population_history.copy()
        temp_result.fitness_history = self.result.fitness_history.copy()
        temp_result.best_individuals = self.result.best_individuals.copy()
        temp_result.best_fitnesses = self.result.best_fitnesses.copy()
        temp_result.generation_history = self.result.generation_history.copy()
        
        temp_result.save(filepath)
    
    def get_best_parameters(self) -> Optional[np.ndarray]:
        """Get the best parameters found during optimization."""
        return self.result.get_best_individual()
    
    def get_best_fitness(self) -> Optional[Union[float, np.ndarray]]:
        """Get the fitness of the best individual."""
        return self.result.get_best_fitness()
    
    def save_results(self, filepath: str, format: str = 'pickle'):
        """Save optimization results to file."""
        self.result.save(filepath, format)
    
    def load_results(self, filepath: str, format: str = 'pickle'):
        """Load optimization results from file."""
        self.result = OptimizationResult.load(filepath, format)
