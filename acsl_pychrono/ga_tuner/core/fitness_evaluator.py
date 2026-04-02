"""
Core fitness evaluator for genetic algorithm optimization.
"""

from typing import List, Union, Optional, Callable, Any
import numpy as np
import threading
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

class FitnessEvaluator:
    """
    Class to handle fitness evaluation for optimization.
    Supports single and multi-objective evaluation
    """
    
    def __init__(self, 
                 evaluation_function: Callable,
                 n_objectives: int = 1,
                 parallel: bool = False,
                 n_workers: Optional[int] = None,
                 use_processes: bool = False,
                 cache_size: int = 1000):
        """
        Initialize fitness evaluator.
        
        Args:
            evaluation_function: Function that evaluates fitness for parameters
            n_objectives: Number of objectives (1 for single-objective)
            parallel: Whether to use parallel evaluation
            n_workers: Number of parallel workers (None = auto)
            use_processes: Whether to use processes instead of threads
            cache_size: Size of fitness cache
        """
        self.evaluation_function = evaluation_function
        self.n_objectives = n_objectives
        self.parallel = parallel
        self.n_workers = n_workers
        self.use_processes = use_processes
        
        # Initialize cache with thread lock for thread-safety
        self.cache = {}
        self.cache_size = cache_size
        self.cache_hits = 0
        self.total_evaluations = 0
        self._cache_lock = threading.Lock()

        # Initialize parallel executor
        self._init_parallel_executor()
    
    def __getstate__(self):
        """Prepare object for pickling - exclude unpicklable thread lock and executor."""
        state = self.__dict__.copy()
        # Remove the unpicklable thread lock
        state['_cache_lock'] = None
        # Remove executor (will be recreated in subprocess)
        state['executor'] = None
        return state
    
    def __setstate__(self, state):
        """Restore object after unpickling - recreate thread lock."""
        self.__dict__.update(state)
        # Recreate the thread lock
        self._cache_lock = threading.Lock()
        # Don't recreate executor - it's only needed in the parent process
    
    def _init_parallel_executor(self):
        """Initialize parallel execution environment."""
        if self.parallel:
            try:
                if self.use_processes:
                    self.executor = ProcessPoolExecutor(max_workers=self.n_workers)
                    actual_workers = self.executor._max_workers
                    print(f"[PARALLEL INIT] ProcessPoolExecutor created: {actual_workers} workers")
                else:
                    self.executor = ThreadPoolExecutor(max_workers=self.n_workers)
                    actual_workers = self.executor._max_workers
                    print(f"[PARALLEL INIT] ThreadPoolExecutor created: {actual_workers} workers")
            except Exception as e:
                print(f"[PARALLEL INIT] ERROR: Failed to initialize parallel executor: {e}")
                self.parallel = False
                self.executor = None
    
    def evaluate_individual(self, 
                          parameters: Union[List[float], np.ndarray],
                          use_cache: bool = True) -> Union[float, List[float]]:
        """
        Evaluate fitness for a single individual.
        
        Args:
            parameters: Parameters to evaluate
            use_cache: Whether to use cached results
            
        Returns:
            Fitness value(s)
        """
        with self._cache_lock:
            self.total_evaluations += 1
        
        # Convert parameters to a hashable tuple for caching
        param_tuple = self._make_hashable(parameters)
        
        # Check cache (thread-safe)
        if use_cache:
            with self._cache_lock:
                if param_tuple in self.cache:
                    self.cache_hits += 1
                    return self.cache[param_tuple]
        
        try:
            fitness = self.evaluation_function(parameters)
            
            # Handle None values from evaluation function
            if fitness is None:
                print(f"Warning: Evaluation function returned None for parameters {parameters}")
                fitness = float('inf')
            elif isinstance(fitness, list) and any(f is None for f in fitness):
                print(f"Warning: Evaluation function returned None values in fitness list for parameters {parameters}")
                fitness = [float('inf') if f is None else f for f in fitness]
                
        except Exception as e:
            print(f"Error evaluating parameters {parameters}: {str(e)}")
            # Return worst possible fitness
            return [float('inf')] * self.n_objectives if self.n_objectives > 1 else float('inf')
        
        # Cache result
        if use_cache:
            self._cache_result(param_tuple, fitness)
        
        return fitness

    def _make_hashable(self, value: Any):
        """Convert nested parameter structures to hashable tuples."""
        if isinstance(value, np.ndarray):
            return tuple(self._make_hashable(elem) for elem in value.tolist())
        if isinstance(value, (list, tuple)):
            return tuple(self._make_hashable(elem) for elem in value)
        if isinstance(value, (np.generic,)):  # numpy scalar
            return value.item()
        return value
    
    def evaluate_population(self,
                          population: List[Union[List[float], np.ndarray]],
                          use_cache: bool = True) -> List[Union[float, List[float]]]:
        """
        Evaluate fitness for a population of individuals.
        
        Args:
            population: List of parameter sets to evaluate
            use_cache: Whether to use cached results
            
        Returns:
            List of fitness values
        """
        if not self.parallel:
            print(f"[EVAL] Evaluating {len(population)} individuals SEQUENTIALLY")
            return [self.evaluate_individual(ind, use_cache) for ind in population]
        
        # Parallel evaluation
        print(f"[EVAL] Evaluating {len(population)} individuals in PARALLEL")
        print(f"[EVAL] Parallel flag: {self.parallel}, Executor exists: {hasattr(self, 'executor')}")
        return self._parallel_evaluate(population, use_cache)
    
    def _parallel_evaluate(self,
                         population: List[Union[List[float], np.ndarray]],
                         use_cache: bool) -> List[Union[float, List[float]]]:
        """
        Evaluate population in parallel.
        
        Args:
            population: List of parameter sets to evaluate
            use_cache: Whether to use cached results
            
        Returns:
            List of fitness values
        """
        # Check if parallel execution is available
        if not hasattr(self, 'executor') or self.executor is None:
            print("[PARALLEL] WARNING: Parallel executor not available, falling back to sequential")
            return [self.evaluate_individual(ind, use_cache) for ind in population]
        
        print(f"[PARALLEL] Executor type: {type(self.executor).__name__}")
        
        # Filter out cached results
        to_evaluate = []
        cached_results = {}
        
        if use_cache:
            for ind in population:
                param_tuple = self._make_hashable(ind)
                if param_tuple in self.cache:
                    self.cache_hits += 1
                    cached_results[param_tuple] = self.cache[param_tuple]
                else:
                    to_evaluate.append(ind)
        else:
            to_evaluate = population
        
        print(f"[PARALLEL] Pop={len(population)}, Cached={len(cached_results)}, ToEval={len(to_evaluate)}")
        
        # Evaluate remaining individuals in parallel
        results = []
        if to_evaluate:
            try:
                import time
                print(f"[PARALLEL] Submitting {len(to_evaluate)} tasks to executor...")
                start_time = time.time()
                executor = self.executor
                # Use evaluate_individual instead of evaluation_function directly
                # This ensures that any overridden evaluate_individual (like _PartialEvaluatorMixin) is called
                futures = [executor.submit(self.evaluate_individual, ind, False) for ind in to_evaluate]
                print(f"[PARALLEL] Tasks submitted, waiting for results...")
                results = [future.result() for future in futures]
                elapsed = time.time() - start_time
                print(f"[PARALLEL] Completed {len(results)} evaluations in {elapsed:.2f}s")

                # Cache new results
                if use_cache:
                    for ind, result in zip(to_evaluate, results):
                        self._cache_result(self._make_hashable(ind), result)
                
            except Exception as e:
                import traceback
                print(f"[PARALLEL] ERROR during parallel evaluation: {e}")
                print("[PARALLEL] Full traceback:")
                traceback.print_exc()
                print("[PARALLEL] Falling back to sequential evaluation")
                # Fallback to sequential evaluation
                results = [self.evaluate_individual(ind, use_cache) for ind in to_evaluate]
        
        # Combine cached and new results
        if use_cache:
            final_results = []
            for ind in population:
                param_tuple = self._make_hashable(ind)
                if param_tuple in cached_results:
                    final_results.append(cached_results[param_tuple])
                else:
                    final_results.append(results.pop(0))
            return final_results
        
        return results
    
    def _cache_result(self, param_tuple: tuple, fitness: Union[float, List[float]]):
        """
        Cache a fitness result (thread-safe).
        
        Args:
            param_tuple: Parameters as tuple
            fitness: Fitness value(s)
        """
        with self._cache_lock:
            # Implement LRU cache
            if len(self.cache) >= self.cache_size:
                # Remove oldest entry
                self.cache.pop(next(iter(self.cache)))
            
            self.cache[param_tuple] = fitness
    
    def __del__(self):
        """Cleanup parallel executor."""
        executor = getattr(self, 'executor', None)
        if executor is not None:
            executor.shutdown()
    
