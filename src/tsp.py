import random
import time
import datetime
import sys
from typing import List, Tuple, Dict, Callable, Set, Any

from src.algorithms import dijkstra, astar


class TabuSearch:
    def __init__(self, graph, start_stop, stops_to_visit, start_time, cost_function,
                 use_dynamic_tabu_size=True, use_aspiration=True, use_advanced_sampling=True):
        """
        Initialize Tabu Search algorithm for solving multi-stop routing problems
        
        Args:
            graph: The transportation network graph
            start_stop: Starting stop name
            stops_to_visit: List of stops that must be visited
            start_time: Departure time from the first stop
            cost_function: Function used to evaluate route cost
            use_dynamic_tabu_size: Whether to adjust tabu list size dynamically
            use_aspiration: Whether to use aspiration criteria
            use_advanced_sampling: Whether to use advanced neighborhood sampling
        """
        self.graph = graph
        self.start_stop = start_stop.lower()
        self.stops_to_visit = [stop.lower() for stop in stops_to_visit]
        self.start_time = start_time
        self.cost_function = cost_function

        # Tabu search parameters
        self.use_dynamic_tabu_size = use_dynamic_tabu_size
        self.use_aspiration = use_aspiration
        self.use_advanced_sampling = use_advanced_sampling

        # Route cache
        self.route_cache = {}
        
        # Distance estimation cache for heuristic evaluation
        self.distance_cache = {}

        # Statistics
        self.iterations = 0
        self.aspiration_accepted = 0
        self.cache_hits = 0
        self.cache_misses = 0
        
        # For small problems, precompute only what's necessary
        if len(stops_to_visit) <= 5:
            self._selective_precompute()
        else:
            # For larger problems, just precompute direct paths from start
            self._precompute_from_start()
        
    def _selective_precompute(self):
        """Precompute paths only between frequently used stop pairs"""
        print(f"Selective precomputation for {len(self.stops_to_visit)} stops...", file=sys.stderr)
        all_stops = [self.start_stop] + self.stops_to_visit
        
        # Base time for precomputation
        base_time = self.start_time
        
        # For small problems, precompute everything
        if len(all_stops) <= 5:
            for from_stop in all_stops:
                for to_stop in all_stops:
                    if from_stop != to_stop:
                        key = (from_stop, to_stop, base_time.strftime('%H:%M:%S'))
                        path, costs, segment_time, line_changes = dijkstra(
                            self.graph, from_stop, to_stop, base_time, self.cost_function
                        )
                        self.route_cache[key] = (path, costs, segment_time, line_changes)
                        
                        # Also cache distance for heuristic evaluation
                        self.distance_cache[(from_stop, to_stop)] = (
                            segment_time.total_seconds() / 60.0, line_changes
                        )
        else:
            # For larger problems, be more selective
            self._precompute_from_start()
            
        print(f"Precomputed {len(self.route_cache)} paths", file=sys.stderr)

    def _precompute_from_start(self):
        """Precompute paths from start to all stops and between closest pairs"""
        print("Precomputing paths from start...", file=sys.stderr)
        base_time = self.start_time
        
        # Precompute paths from start to all stops
        for stop in self.stops_to_visit:
            key = (self.start_stop, stop, base_time.strftime('%H:%M:%S'))
            path, costs, segment_time, line_changes = dijkstra(
                self.graph, self.start_stop, stop, base_time, self.cost_function
            )
            self.route_cache[key] = (path, costs, segment_time, line_changes)
            
            # Cache estimated travel time and line changes
            self.distance_cache[(self.start_stop, stop)] = (
                segment_time.total_seconds() / 60.0, line_changes
            )
        
        print(f"Precomputed {len(self.route_cache)} paths from start", file=sys.stderr)

    def _get_route_segment(self, from_stop, to_stop, current_time):
        """Get route segment from cache or calculate if not available"""
        # Format time consistently for caching
        time_key = current_time.strftime('%H:%M:%S')
        key = (from_stop, to_stop, time_key)
        
        # Try exact match first
        if key in self.route_cache:
            self.cache_hits += 1
            return self.route_cache[key]
        
        # If no exact match, try to find a close time match (within 15 minutes)
        if len(self.stops_to_visit) <= 3:  # Only for small problems to save time
            for cached_key in list(self.route_cache.keys()):
                cached_from, cached_to, cached_time_str = cached_key
                if cached_from == from_stop and cached_to == to_stop:
                    cached_time = datetime.datetime.strptime(cached_time_str, '%H:%M:%S')
                    time_diff = abs((cached_time - current_time).total_seconds() / 60.0)
                    # If within 15 minutes, use this cached result
                    if time_diff <= 15:
                        self.cache_hits += 1
                        return self.route_cache[cached_key]
        
        # Calculate new path
        self.cache_misses += 1
        path, costs, segment_time, line_changes = dijkstra(
            self.graph, from_stop, to_stop, current_time, self.cost_function
        )
        
        # Cache the result
        self.route_cache[key] = (path, costs, segment_time, line_changes)
        
        # Also update distance cache
        self.distance_cache[(from_stop, to_stop)] = (
            segment_time.total_seconds() / 60.0 if segment_time else float('inf'), 
            line_changes
        )
        
        return path, costs, segment_time, line_changes

    def _calculate_route_cost(self, route: List[str], best_known_cost=float('inf')) -> Tuple[float, List[str], datetime.timedelta, int]:
        """Calculate the total cost of a route through all stops with early termination"""
        total_cost = 0
        total_time = datetime.timedelta()
        total_line_changes = 0
        detailed_path = []

        current_time = self.start_time
        prev_line = None

        for i in range(len(route) - 1):
            from_stop = route[i]
            to_stop = route[i + 1]

            # Early termination if current cost exceeds best known
            if self.cost_function == self.graph.get_time_cost and total_time.total_seconds() / 60.0 >= best_known_cost:
                return float('inf'), [], datetime.timedelta(0), 0
            elif self.cost_function != self.graph.get_time_cost and total_line_changes >= best_known_cost:
                return float('inf'), [], datetime.timedelta(0), 0
                
            # Estimate remaining cost using distance cache
            if self.cost_function == self.graph.get_time_cost and i < len(route) - 2:
                remaining_time_estimate = 0
                for j in range(i + 1, len(route) - 1):
                    estimate_key = (route[j], route[j + 1])
                    if estimate_key in self.distance_cache:
                        remaining_time_estimate += self.distance_cache[estimate_key][0]
                    else:
                        # Use a very rough estimate if not cached
                        remaining_time_estimate += 20  # 20 minutes per segment guess
                        
                # If current + estimated remaining exceeds best, terminate early
                if (total_time.total_seconds() / 60.0) + remaining_time_estimate >= best_known_cost:
                    return float('inf'), [], datetime.timedelta(0), 0

            # Get route segment
            path, costs, segment_time, line_changes = self._get_route_segment(
                from_stop, to_stop, current_time
            )

            if not path:  # No path found
                return float('inf'), [], datetime.timedelta(0), 0

            # Update totals
            detailed_path.extend(path)
            total_time += segment_time
            total_line_changes += line_changes

            # Update current time for next segment
            current_time += segment_time

        # Calculate final cost based on optimization criterion
        if self.cost_function == self.graph.get_time_cost:
            final_cost = total_time.total_seconds() / 60.0  # Convert to minutes
        else:
            final_cost = total_line_changes

        return final_cost, detailed_path, total_time, total_line_changes

    def _calculate_initial_solution(self) -> List[str]:
        """Generate initial solution using a greedy approach"""
        # For small problems, try all permutations
        if len(self.stops_to_visit) <= 3:
            return self._get_best_permutation()
            
        # For larger problems, use greedy approach
        route = [self.start_stop]
        unvisited = set(self.stops_to_visit)
        current = self.start_stop
        current_time = self.start_time

        while unvisited:
            best_next = None
            best_cost = float('inf')
            best_time = None

            for next_stop in unvisited:
                # Use cached path if available
                path, costs, segment_time, _ = self._get_route_segment(
                    current, next_stop, current_time
                )
                
                if segment_time:
                    cost = segment_time.total_seconds() / 60.0
                    if cost < best_cost:
                        best_cost = cost
                        best_next = next_stop
                        best_time = segment_time

            if best_next is None:  # No path found to any remaining stop
                break

            route.append(best_next)
            unvisited.remove(best_next)
            current = best_next
            current_time += best_time

        return route
    
    def _get_best_permutation(self) -> List[str]:
        """For small problems, evaluate all permutations"""
        print("Evaluating all permutations for small problem", file=sys.stderr)
        import itertools
        
        best_route = None
        best_cost = float('inf')
        
        # Generate all permutations of stops
        for perm in itertools.permutations(self.stops_to_visit):
            route = [self.start_stop] + list(perm)
            cost, _, _, _ = self._calculate_route_cost(route)
            if cost < best_cost:
                best_cost = cost
                best_route = route
                
        print(f"Best permutation has cost {best_cost}", file=sys.stderr)
        return best_route

    def _get_tabu_list_size(self) -> int:
        """Determine appropriate tabu list size"""
        if not self.use_dynamic_tabu_size:
            return 10  # Default fixed size

        # Dynamically adjust based on problem size
        n = len(self.stops_to_visit)
        return max(3, min(20, n))

    def _generate_neighbors(self, solution: List[str], num_neighbors: int) -> List[List[str]]:
        """Generate neighbor solutions by swapping stops"""
        neighbors = []
        n = len(solution)
        
        # For very small problems, generate all possible swaps
        if n <= 4:  # Including start stop
            for i in range(1, n - 1):  # Skip start position
                for j in range(i + 1, n - 1):  # Skip end position if returning to start
                    neighbor = solution.copy()
                    neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
                    neighbors.append(neighbor)
            return neighbors

        # For larger problems, use sampling
        positions = []
        max_swaps = min(num_neighbors, (n - 2) * (n - 3) // 2)
        
        while len(positions) < max_swaps:
            i = random.randint(1, n - 2)
            j = random.randint(1, n - 2)
            while i == j:
                j = random.randint(1, n - 2)
            
            if i > j:
                i, j = j, i
                
            if (i, j) not in positions:
                positions.append((i, j))
                neighbor = solution.copy()
                neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
                neighbors.append(neighbor)

        return neighbors

    def solve(self, max_iterations=100, max_time=30) -> Tuple[List[str], float, datetime.timedelta, int]:
        """
        Run Tabu Search algorithm to find optimal tour
        
        Args:
            max_iterations: Maximum number of iterations
            max_time: Maximum runtime in seconds
            
        Returns:
            Tuple of (detailed path, cost, total time, line changes)
        """
        start_time = time.time()
        print(f"Starting tabu search with {len(self.stops_to_visit)} stops to visit", file=sys.stderr)

        # Adjust parameters based on problem size
        if len(self.stops_to_visit) <= 3:
            max_iterations = min(max_iterations, 50)  # Small problems need fewer iterations
            
        # Generate initial solution
        current_solution = self._calculate_initial_solution()
        current_cost, current_path, current_total_time, current_line_changes = self._calculate_route_cost(
            current_solution)

        best_solution = current_solution
        best_cost = current_cost
        best_path = current_path
        best_total_time = current_total_time
        best_line_changes = current_line_changes

        # Initialize tabu list as a dictionary of moves with expiration
        tabu_list = {}
        tabu_tenure = self._get_tabu_list_size()

        # Main Tabu Search loop
        iteration = 0
        non_improving_iterations = 0
        
        # For very small problems, we can terminate early
        early_termination_count = 5 if len(self.stops_to_visit) <= 3 else 15

        while iteration < max_iterations and time.time() - start_time < max_time:
            # Generate fewer neighbors for small problems
            num_neighbors = min(5, (len(self.stops_to_visit) * 2))
            neighbors = self._generate_neighbors(current_solution, num_neighbors)

            # Evaluate neighbors
            best_neighbor = None
            best_neighbor_cost = float('inf')
            best_neighbor_path = []
            best_neighbor_time = datetime.timedelta(0)
            best_neighbor_changes = 0
            best_move = None

            for neighbor in neighbors:
                # Identify the move - simplified to just the positions that were swapped
                # This reduces memory usage and improves lookup speed
                move = None
                for i in range(len(neighbor)):
                    if i < len(current_solution) and neighbor[i] != current_solution[i]:
                        for j in range(i + 1, len(neighbor)):
                            if j < len(current_solution) and neighbor[j] != current_solution[j]:
                                move = (i, j)
                                break
                        if move:
                            break

                # Check if move is tabu
                is_tabu = move in tabu_list and tabu_list[move] > iteration

                # Skip evaluation if tabu and not using aspiration
                if is_tabu and not self.use_aspiration:
                    continue

                # Evaluate solution with early termination against best known
                neighbor_cost, neighbor_path, neighbor_time, neighbor_changes = self._calculate_route_cost(
                    neighbor, best_known_cost=best_cost if self.use_aspiration else best_neighbor_cost
                )

                # Only update if better and not tabu, or passes aspiration
                if neighbor_cost < float('inf') and (not is_tabu or (self.use_aspiration and neighbor_cost < best_cost)):
                    if neighbor_cost < best_neighbor_cost:
                        best_neighbor = neighbor
                        best_neighbor_cost = neighbor_cost
                        best_neighbor_path = neighbor_path
                        best_neighbor_time = neighbor_time
                        best_neighbor_changes = neighbor_changes
                        best_move = move
                        
                        # If using aspiration and found better than global best, note it
                        if is_tabu and self.use_aspiration and neighbor_cost < best_cost:
                            self.aspiration_accepted += 1

            # No valid neighbor found
            if best_neighbor is None:
                print("No valid neighbors found, terminating search", file=sys.stderr)
                break

            # Update current solution
            current_solution = best_neighbor
            current_cost = best_neighbor_cost
            current_path = best_neighbor_path
            current_total_time = best_neighbor_time
            current_line_changes = best_neighbor_changes

            # Update best solution if improved
            if current_cost < best_cost:
                best_solution = current_solution
                best_cost = current_cost
                best_path = current_path
                best_total_time = current_total_time
                best_line_changes = current_line_changes
                non_improving_iterations = 0
                print(f"New best solution found: {best_cost:.2f}", file=sys.stderr)
            else:
                non_improving_iterations += 1

            # Add current move to tabu list
            if best_move:
                tabu_list[best_move] = iteration + tabu_tenure

            # Update iteration counter
            iteration += 1
            self.iterations = iteration

            # Early termination if no improvement
            if non_improving_iterations >= early_termination_count:
                print(f"Early termination after {non_improving_iterations} non-improving iterations", file=sys.stderr)
                break

        # Print statistics
        elapsed = time.time() - start_time
        print(f"Tabu search completed in {iteration} iterations, {elapsed:.2f}s", file=sys.stderr)
        print(f"Cache stats: {self.cache_hits} hits, {self.cache_misses} misses", file=sys.stderr)

        return best_path, best_cost, best_total_time, best_line_changes


def solve_multi_stop_problem(graph, start_stop, stops_to_visit, start_time, cost_function,
                             use_dynamic_tabu=True, use_aspiration=True, use_advanced_sampling=True):
    """
    Solve the multi-stop routing problem using Tabu Search
    
    Args:
        graph: The transportation network graph
        start_stop: Starting stop name
        stops_to_visit: List of stops that must be visited
        start_time: Departure time from the first stop
        cost_function: Function used to evaluate route cost
        use_dynamic_tabu: Whether to use dynamic tabu list sizing
        use_aspiration: Whether to use aspiration criteria
        use_advanced_sampling: Whether to use advanced neighborhood sampling
        
    Returns:
        Tuple of (detailed path, cost, total time, line changes)
    """
    # For very small problems (3 or fewer stops), use exhaustive search
    if len(stops_to_visit) <= 3:
        print(f"Small problem with {len(stops_to_visit)} stops - using optimized approach", file=sys.stderr)
        max_iterations = 50  # Fewer iterations needed
        max_time = 15  # Shorter timeout
    else:
        max_iterations = 1000
        max_time = 60

    tabu_search = TabuSearch(
        graph,
        start_stop,
        stops_to_visit,
        start_time,
        cost_function,
        use_dynamic_tabu,
        use_aspiration,
        use_advanced_sampling
    )

    return tabu_search.solve(max_iterations=max_iterations, max_time=max_time)
