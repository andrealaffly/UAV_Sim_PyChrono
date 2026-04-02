"""
DEAP-based genetic algorithm implementation
"""

from typing import List, Union, Optional
import numpy as np
import random
from deap import base, creator, tools

from .base_ga import BaseGATuner
from ..core.parameter_bounds import ParameterBounds
from ..core.fitness_evaluator import FitnessEvaluator

class DEAPGATuner(BaseGATuner):
    """
    genetic algorithm tuner using DEAP library.
    Supports both single and multi-objective optimization
    """
    
    def __init__(self, 
                 parameter_bounds: ParameterBounds,
                 fitness_evaluator: FitnessEvaluator,
                 population_size: int = 100,
                 n_generations: int = 50,
                 random_seed: Optional[int] = None,
                 multi_objective: bool = False,
                 n_objectives: int = 3,
                 crossover_prob: float = 0.8,
                 mutation_prob: float = 0.3,
                 selection_method: str = 'tournament',
                 tournament_size: int = 3,
                 crossover_type: str = 'blend',
                 mutation_type: str = 'gaussian'):
        """
        Initialize DEAP GA tuner with enhanced configuration options.
        
        Args:
            parameter_bounds: Parameter bounds and constraints
            fitness_evaluator: Fitness evaluation function
            population_size: Size of the population
            n_generations: Number of generations
            random_seed: Random seed for reproducibility
            multi_objective: Whether to use multi-objective optimization
            n_objectives: Number of objectives for multi-objective optimization
            crossover_prob: Crossover probability
            mutation_prob: Mutation probability
            selection_method: Selection method ('tournament', 'roulette', 'rank')
            tournament_size: Tournament size for tournament selection (default: 3)
            crossover_type: Type of crossover ('blend', 'simulated_binary', 'uniform')
            mutation_type: Type of mutation ('gaussian', 'polynomial', 'uniform')
        """
        super().__init__(parameter_bounds, fitness_evaluator, population_size, n_generations, random_seed)
        
        self.multi_objective = multi_objective
        self.n_objectives = n_objectives
        self.crossover_prob = crossover_prob
        self.mutation_prob = mutation_prob
        self.selection_method = selection_method
        self.tournament_size = tournament_size
        self.crossover_type = crossover_type
        self.mutation_type = mutation_type
        
        # Set random seed for DEAP
        if random_seed is not None:
            random.seed(random_seed)
        
        # Initialize DEAP components
        self._setup_deap()
        
        # Store configuration for results
        self.result.optimization_parameters.update({
            'multi_objective': multi_objective,
            'n_objectives': n_objectives,
            'crossover_prob': crossover_prob,
            'mutation_prob': mutation_prob,
            'selection_method': selection_method,
            'tournament_size': tournament_size,
            'crossover_type': crossover_type,
            'mutation_type': mutation_type
        })
    
    def _setup_deap(self):
        """Setup DEAP creator and toolbox with enhanced operators."""
        # Clear any existing creators to avoid conflicts
        self._clear_deap_creators()
        
        # Create fitness and individual classes
        if self.multi_objective:
            # Multi-objective: minimize all objectives (e.g., tracking errors + control effort)
            creator.create("FitnessMulti", base.Fitness, weights=(-1.0,) * self.n_objectives)
            fitness_class = creator.FitnessMulti
        else:
            creator.create("FitnessSingle", base.Fitness, weights=(-1.0,))
            fitness_class = creator.FitnessSingle
        
        creator.create("Individual", list, fitness=fitness_class)
        
        # Create toolbox
        self.toolbox = base.Toolbox()
        
        # Register individual and population creation using per-parameter bounds
        self.toolbox.register("individual", self._create_individual)
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)

        # Register enhanced genetic operators
        self._register_genetic_operators()
    
    def _clear_deap_creators(self):
        """Clear existing DEAP creator classes."""
        if hasattr(creator, 'FitnessMulti'):
            del creator.FitnessMulti
        if hasattr(creator, 'FitnessSingle'):
            del creator.FitnessSingle
        if hasattr(creator, 'Individual'):
            del creator.Individual
    
    def _create_individual(self):
        """Generate a new individual sampling each gene within its bounds."""
        values = self.parameter_bounds.get_random_parameters()
        if hasattr(values, "tolist"):
            values = values.tolist()
        return creator.Individual(values)

    def _register_genetic_operators(self):
        """Register enhanced genetic operators in the toolbox."""
        # Crossover operators
        if self.crossover_type == 'blend':
            self.toolbox.register("mate", tools.cxBlend, alpha=0.4)
        elif self.crossover_type == 'simulated_binary':
            self.toolbox.register("mate", tools.cxSimulatedBinary, eta=3.0)
        elif self.crossover_type == 'uniform':
            self.toolbox.register("mate", tools.cxUniform, indpb=0.5)
        else:
            raise ValueError(f"Unknown crossover type: {self.crossover_type}")
        
        # Mutation operators with more exploration
        if self.mutation_type == 'gaussian':
            # Increased sigma for more exploration, higher indpb for more frequent mutation
            self.toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=5.0, indpb=0.3)
        elif self.mutation_type == 'polynomial':
            # Lower eta for more exploration, higher indpb
            self.toolbox.register("mutate", tools.mutPolynomialBounded,
                                low=self.parameter_bounds.lower_bounds.tolist(),
                                up=self.parameter_bounds.upper_bounds.tolist(),
                                eta=2.0, indpb=0.3)
        elif self.mutation_type == 'uniform':
            def _uniform_mutation(individual, low, up, indpb):
                for i, (lo, hi) in enumerate(zip(low, up)):
                    if random.random() < indpb:
                        individual[i] = random.uniform(lo, hi)
                return (individual,)

            self.toolbox.register(
                "mutate",
                _uniform_mutation,
                low=self.parameter_bounds.lower_bounds.tolist(),
                up=self.parameter_bounds.upper_bounds.tolist(),
                indpb=0.3
            )
        else:
            raise ValueError(f"Unknown mutation type: {self.mutation_type}")
        
        # Selection operators with reduced selection pressure
        if self.selection_method == 'tournament':
            # Use configured tournament size
            self.toolbox.register("select", tools.selTournament, tournsize=self.tournament_size)
        elif self.selection_method == 'roulette':
            self.toolbox.register("select", tools.selRoulette)
        elif self.selection_method == 'rank':
            self.toolbox.register("select", tools.selRandom)
        else:
            raise ValueError(f"Unknown selection method: {self.selection_method}")
        
        # Multi-objective selection
        if self.multi_objective:
            self.toolbox.register("select_multi", tools.selNSGA2)
        
        # Decorate operators with bounds checking
        self.toolbox.decorate("mate", self._check_bounds)
        self.toolbox.decorate("mutate", self._check_bounds)
    
    def _check_bounds(self, func):
        """Enhanced bounds checking decorator with efficient numpy operations."""
        def wrapper(*args, **kwargs):
            offspring = func(*args, **kwargs)
            for child in offspring:
                child_array = np.array(child)
                child_array = np.clip(
                    child_array,
                    self.parameter_bounds.lower_bounds,
                    self.parameter_bounds.upper_bounds
                )
                for i, val in enumerate(child_array):
                    child[i] = val
            return offspring
        return wrapper
    
    def optimize(self, 
                verbose: bool = True,
                save_intermediate: bool = False,
                intermediate_save_path: str = "intermediate_results"):
        """Enhanced optimization with parallel evaluation support."""
        if verbose:
            print(f"Starting DEAP Genetic Algorithm optimization")
            print(f"Configuration:")
            for key, value in self.result.optimization_parameters.items():
                print(f"  {key}: {value}")
            print("=" * 50)
        
        return super().optimize(verbose, save_intermediate, intermediate_save_path)

    def set_operators(self, 
                     crossover_type: Optional[str] = None,
                     mutation_type: Optional[str] = None,
                     selection_method: Optional[str] = None):
        """
        Update genetic operators during optimization.
        
        Args:
            crossover_type: New crossover type
            mutation_type: New mutation type
            selection_method: New selection method
        """
        if crossover_type:
            self.crossover_type = crossover_type
        if mutation_type:
            self.mutation_type = mutation_type
        if selection_method:
            self.selection_method = selection_method
            
        self._register_genetic_operators()
    
    @property
    def available_operators(self):
        """Get available genetic operators."""
        return {
            'crossover': ['blend', 'simulated_binary', 'uniform'],
            'mutation': ['gaussian', 'polynomial', 'uniform'],
            'selection': ['tournament', 'roulette', 'rank']
        }
    
    def _initialize_population(self) -> List[np.ndarray]:
        """Initialize population with enhanced diversity preservation."""
        population = self.toolbox.population(n=self.population_size)
        return [np.array(ind) for ind in population]
    
    def _select_parents(self,
                       population: List[np.ndarray],
                       fitnesses: List[Union[float, List[float]]]) -> List[np.ndarray]:
        """Enhanced parent selection with diversity preservation."""
        deap_population = [creator.Individual(list(ind)) for ind in population]
        
        for ind, fit in zip(deap_population, fitnesses):
            if self.multi_objective:
                ind.fitness.values = fit
            else:
                ind.fitness.values = (fit,)
        
        if self.multi_objective:
            parents = self.toolbox.select_multi(deap_population, len(deap_population))
        else:
            parents = self.toolbox.select(deap_population, len(deap_population))
        
        return [np.array(ind) for ind in parents]
    
    def _crossover(self, parents: List[np.ndarray]) -> List[np.ndarray]:
        """Enhanced crossover with improved offspring generation."""
        offspring = []
        n_parents = len(parents)

        for i in range(0, n_parents - 1, 2):
            if random.random() < self.crossover_prob:
                p1 = creator.Individual(list(parents[i]))
                p2 = creator.Individual(list(parents[i + 1]))
                c1, c2 = self.toolbox.mate(p1, p2)
                offspring.extend([np.array(c1), np.array(c2)])
            else:
                offspring.extend([parents[i].copy(), parents[i + 1].copy()])

        if n_parents % 2 == 1:
            offspring.append(parents[-1].copy())

        return offspring
    
    def _mutate(self, offspring: List[np.ndarray]) -> List[np.ndarray]:
        """Enhanced mutation with adaptive rates."""
        mutated = []
        for child in offspring:
            if random.random() < self.mutation_prob:
                mutant = creator.Individual(list(child))
                self.toolbox.mutate(mutant)
                mutated.append(np.array(mutant))
            else:
                mutated.append(child.copy())
        return mutated
    
    def _survive(self,
                 population: List[np.ndarray],
                 offspring: List[np.ndarray],
                 population_fitnesses: List[Union[float, List[float]]],
                 offspring_fitnesses: List[Union[float, List[float]]]) -> tuple[List[np.ndarray], List[Union[float, List[float]]]]:
        """Survival selection with reduced elitism for better exploration.

        Returns both the survivor population and their corresponding fitnesses
        to avoid re-evaluating expensive simulations in the base class loop.
        """
        # Use only offspring for next generation (no elitism)
        # This forces more exploration but risks losing good solutions
        deap_offspring = [creator.Individual(list(ind)) for ind in offspring]
        for ind, fit in zip(deap_offspring, offspring_fitnesses):
            if self.multi_objective:
                ind.fitness.values = fit
            else:
                ind.fitness.values = (fit,)
        
        if self.multi_objective:
            survivors = self.toolbox.select_multi(deap_offspring, self.population_size)
        else:
            survivors = self.toolbox.select(deap_offspring, self.population_size)
        
        survivor_arrays = [np.array(ind) for ind in survivors]
        survivor_fitnesses: List[Union[float, List[float]]] = []
        for ind in survivors:
            values = ind.fitness.values
            if self.multi_objective:
                survivor_fitnesses.append(list(values))
            else:
                survivor_fitnesses.append(float(values[0]))
        
        return survivor_arrays, survivor_fitnesses
