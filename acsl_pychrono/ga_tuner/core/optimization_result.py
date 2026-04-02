"""
Optimization result storage and analysis.
"""

from typing import List, Dict, Any, Optional, Union
import numpy as np
import json
import pickle
import os
from datetime import datetime


class OptimizationResult:
    """
    Class to store and analyze optimization results.
    Supports both single and multi-objective optimization results.
    """
    
    def __init__(self,
                 algorithm_name: str,
                 parameter_bounds,  # ParameterBounds class
                 timestamp: Optional[str] = None):
        """
        Initialize optimization result storage.
        
        Args:
            algorithm_name: Name of the optimization algorithm
            parameter_bounds: Parameter bounds used in optimization
            timestamp: Optional timestamp for the result
        """
        self.algorithm_name = algorithm_name
        self.parameter_bounds = parameter_bounds
        self.timestamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Basic information
        self.optimization_parameters = {}
        self.computation_time = None
        
        # Evolution history
        self.population_history = []
        self.fitness_history = []
        self.best_individuals = []
        self.best_fitnesses = []
        self.generation_history = []
        
        # Final results
        self.final_population = None
        self.final_fitnesses = None
        self.pareto_front = None
        self.pareto_fitnesses = None
        
        # Additional metrics
        self.additional_metrics = {}
        
        # Performance metrics
        self.convergence_history = []
        self.diversity_history = []
    
    def add_generation(self,
                      generation: int,
                      population: List[np.ndarray],
                      fitnesses: List[Union[float, List[float]]],
                      best_individual: np.ndarray,
                      best_fitness: Union[float, List[float]]):
        """
        Add results from a single generation.
        
        Args:
            generation: Generation number
            population: List of individuals in the population
            fitnesses: Fitness values for the population
            best_individual: Best individual in this generation
            best_fitness: Fitness of the best individual
        """
        self.generation_history.append(generation)
        self.population_history.append(population)
        self.fitness_history.append(fitnesses)
        self.best_individuals.append(best_individual)
        self.best_fitnesses.append(best_fitness)
        
        # Update convergence and diversity metrics
        self._update_metrics(population, fitnesses)
    
    def _update_metrics(self,
                       population: List[np.ndarray],
                       fitnesses: List[Union[float, List[float]]]):
        """
        Update performance metrics.
        
        Args:
            population: Current population
            fitnesses: Current fitness values
        """
        # Convergence metric
        if len(fitnesses) > 0 and isinstance(fitnesses[0], (list, np.ndarray)):
            # Multi-objective - handle mixed types
            convergence_values = []
            for f in fitnesses:
                if isinstance(f, (list, np.ndarray)):
                    convergence_values.append(f[0])
                else:
                    # Failed simulation, use a large penalty value
                    convergence_values.append(float('inf'))
            convergence = np.min(convergence_values)
        else:
            # Single objective
            convergence = np.min(fitnesses)
        
        self.convergence_history.append(convergence)
        
        # Diversity metric
        diversity = self._calculate_diversity(population)
        self.diversity_history.append(diversity)
    
    def _calculate_diversity(self, population: List[np.ndarray]) -> float:
        """
        Calculate population diversity using average pairwise distance.
        
        Args:
            population: List of individuals
            
        Returns:
            Diversity metric value
        """
        if len(population) < 2:
            return 0.0
        
        distances = []
        for i in range(len(population)):
            for j in range(i + 1, len(population)):
                distances.append(np.linalg.norm(population[i] - population[j]))
        
        return np.mean(distances)
    
    def set_final_results(self,
                         population: List[np.ndarray],
                         fitnesses: List[Union[float, List[float]]],
                         pareto_front: Optional[List[np.ndarray]] = None,
                         pareto_fitnesses: Optional[List[List[float]]] = None):
        """
        Set final optimization results.
        
        Args:
            population: Final population
            fitnesses: Final fitness values
            pareto_front: Optional Pareto optimal solutions
            pareto_fitnesses: Optional Pareto front fitness values
        """
        self.final_population = population
        self.final_fitnesses = fitnesses
        self.pareto_front = pareto_front
        self.pareto_fitnesses = pareto_fitnesses
    
    def get_best_individual(self) -> Optional[np.ndarray]:
        """
        Get the best individual found during optimization.
        
        Returns:
            Best parameter set found
        """
        if not self.best_individuals:
            return None
        
        if isinstance(self.best_fitnesses[0], (list, np.ndarray)):
            # Multi-objective: return individual with best first objective
            best_idx = np.argmin([f[0] for f in self.best_fitnesses])
        else:
            # Single objective
            best_idx = np.argmin(self.best_fitnesses)
        
        return self.best_individuals[best_idx]
    
    def get_best_fitness(self) -> Optional[Union[float, np.ndarray]]:
        """
        Get the fitness of the best individual.
        
        Returns:
            Best fitness value(s)
        """
        if not self.best_fitnesses:
            return None
        
        if isinstance(self.best_fitnesses[0], (list, np.ndarray)):
            # Multi-objective
            return min(self.best_fitnesses, key=lambda x: x[0])
        else:
            # Single objective
            return min(self.best_fitnesses)
    
    @property
    def best_parameters(self) -> Optional[np.ndarray]:
        """Alias for get_best_individual() for compatibility."""
        return self.get_best_individual()
    
    def get_convergence_metrics(self) -> Dict[str, float]:
        """
        Get convergence metrics.
        
        Returns:
            Dictionary of convergence metrics
        """
        if not self.convergence_history:
            return {}
        
        return {
            'final_convergence': self.convergence_history[-1],
            'best_convergence': min(self.convergence_history),
            'average_convergence': np.mean(self.convergence_history),
            'convergence_std': np.std(self.convergence_history)
        }
    
    def get_diversity_metrics(self) -> Dict[str, float]:
        """
        Get diversity metrics.
        
        Returns:
            Dictionary of diversity metrics
        """
        if not self.diversity_history:
            return {}
        
        return {
            'final_diversity': self.diversity_history[-1],
            'average_diversity': np.mean(self.diversity_history),
            'diversity_std': np.std(self.diversity_history),
            'diversity_trend': np.polyfit(range(len(self.diversity_history)),
                                        self.diversity_history, 1)[0]
        }

    def save(self, filepath: str, format: str = 'pickle'):
        """
        Save optimization results to file.
        
        Args:
            filepath: Path to save file
            format: File format ('pickle' or 'json')
        """
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        if format == 'pickle':
            with open(filepath, 'wb') as f:
                pickle.dump(self, f)
        elif format == 'json':
            best_individual = self.get_best_individual()
            best_fitness = self.get_best_fitness()
            # Convert numpy arrays to lists for JSON serialization
            data = {
                'algorithm_name': self.algorithm_name,
                'timestamp': self.timestamp,
                'optimization_parameters': self.optimization_parameters,
                'computation_time': self.computation_time,
                'best_individual': best_individual.tolist() if best_individual is not None else None,
                'best_fitness': best_fitness.tolist() if hasattr(best_fitness, "tolist") else best_fitness,
                'convergence_metrics': self.get_convergence_metrics(),
                'diversity_metrics': self.get_diversity_metrics(),
                'additional_metrics': self._json_safe(self.additional_metrics)
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
        else:
            raise ValueError(f"Unsupported format: {format}")
    
    @classmethod
    def load(cls, filepath: str, format: str = 'pickle') -> 'OptimizationResult':
        """
        Load optimization results from file.
        
        Args:
            filepath: Path to load file from
            format: File format ('pickle' or 'json')
            
        Returns:
            OptimizationResult object
        """
        if format == 'pickle':
            with open(filepath, 'rb') as f:
                return pickle.load(f)
        elif format == 'json':
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Create new instance and populate from JSON data
            result = cls(
                algorithm_name=data['algorithm_name'],
                parameter_bounds=None,  # Will need to be set separately
                timestamp=data['timestamp']
            )
            
            result.optimization_parameters = data['optimization_parameters']
            result.computation_time = data['computation_time']
            result.additional_metrics = data['additional_metrics']
            
            return result
        else:
            raise ValueError(f"Unsupported format: {format}")

    @staticmethod
    def _json_safe(value):
        """Convert numpy types/arrays to JSON-serializable structures."""
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, list):
            return [OptimizationResult._json_safe(item) for item in value]
        if isinstance(value, tuple):
            return [OptimizationResult._json_safe(item) for item in value]
        if isinstance(value, dict):
            return {str(k): OptimizationResult._json_safe(v) for k, v in value.items()}
        return value
    
    def print_summary(self):
        """Print a summary of optimization results."""
        print(f"\nOptimization Summary for {self.algorithm_name}")
        print("=" * 50)
        
        print("\nConfiguration:")
        for key, value in self.optimization_parameters.items():
            print(f"  {key}: {value}")
        
        print(f"\nComputation time: {self.computation_time:.2f} seconds")
        
        print("\nBest Result:")
        best_individual = self.get_best_individual()
        best_fitness = self.get_best_fitness()
        
        if best_individual is not None:
            print("Parameters:")
            for i, (name, value) in enumerate(zip(self.parameter_bounds.parameter_names,
                                                best_individual)):
                print(f"  {name}: {value:.6f}")
            
            print("\nFitness:")
            if isinstance(best_fitness, (list, np.ndarray)):
                for i, fit in enumerate(best_fitness):
                    print(f"  Objective {i+1}: {fit:.6f}")
            else:
                print(f"  {best_fitness:.6f}")
        
        if self.pareto_front is not None:
            print(f"\nPareto Front Size: {len(self.pareto_front)}")
        
        print("\nPerformance Metrics:")
        conv_metrics = self.get_convergence_metrics()
        div_metrics = self.get_diversity_metrics()
        
        print("Convergence:")
        for key, value in conv_metrics.items():
            print(f"  {key}: {value:.6f}")
        
        print("\nDiversity:")
        for key, value in div_metrics.items():
            print(f"  {key}: {value:.6f}")
        
        if self.additional_metrics:
            print("\nAdditional Metrics:")
            for key, value in self.additional_metrics.items():
                print(f"  {key}: {value}")
    
    def plot_convergence(self, 
                        save_path: Optional[str] = None,
                        show: bool = True):
        """
        Plot convergence history.
        
        Args:
            save_path: Optional path to save plot
            show: Whether to display the plot
        """
        try:
            import matplotlib.pyplot as plt
            
            plt.figure(figsize=(10, 6))
            plt.plot(self.generation_history, self.convergence_history)
            plt.xlabel('Generation')
            plt.ylabel('Best Fitness')
            plt.title('Optimization Convergence')
            plt.grid(True)
            
            if save_path:
                plt.savefig(save_path)
            
            if show:
                plt.show()
            else:
                plt.close()
                
        except ImportError:
            print("matplotlib is required for plotting")
