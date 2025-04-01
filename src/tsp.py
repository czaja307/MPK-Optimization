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

        # Pre-calculate distances between all stops
        self.distances = {}
        self.paths = {}

        # Statistics
        self.iterations = 0
        self.aspiration_accepted = 0

    def _calculate_route_cost(self, route: List[str]) -> Tuple[float, List[str], datetime.timedelta, int]:
        """Calculate the total cost of a route through all stops"""
        total_cost = 0
        total_time = datetime.timedelta()
        total_line_changes = 0
        detailed_path = []

        current_time = self.start_time
        prev_line = None

        for i in range(len(route) - 1):
            from_stop = route[i]
            to_stop = route[i + 1]

            # Get route segment from current stop to next
            path, costs, segment_time, line_changes = dijkstra(
                self.graph, from_stop, to_stop, current_time, self.cost_function
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
        route = [self.start_stop]
        unvisited = set(self.stops_to_visit)
        current = self.start_stop
        current_time = self.start_time

        while unvisited:
            best_next = None
            best_cost = float('inf')
            best_time = None

            for next_stop in unvisited:
                _, costs, segment_time, _ = dijkstra(
                    self.graph, current, next_stop, current_time, self.cost_function
                )

                if costs[next_stop] < best_cost:
                    best_cost = costs[next_stop]
                    best_next = next_stop
                    best_time = segment_time

            if best_next is None:  # No path found to any remaining stop
                break

            route.append(best_next)
            unvisited.remove(best_next)
            current = best_next
            current_time += best_time

        # Complete the tour by returning to start
        route.append(self.start_stop)
        return route

    def _get_tabu_list_size(self) -> int:
        """Determine appropriate tabu list size"""
        if not self.use_dynamic_tabu_size:
            return 20  # Default fixed size

        # Dynamically adjust based on problem size
        n = len(self.stops_to_visit)
        return max(5, min(50, n * 2))

    def _generate_neighbors(self, solution: List[str], num_neighbors: int) -> List[List[str]]:
        """Generate neighbor solutions by swapping stops"""
        neighbors = []

        # Fixed positions (start and end positions are the same stop)
        n = len(solution) - 1  # -1 because we don't swap the final return to start

        if self.use_advanced_sampling:
            # Use more intelligent sampling based on current solution quality
            # Focus on positions with higher potential for improvement
            if n <= 3:
                # If we have only a few stops, generate all possible swaps
                positions = [(i, j) for i in range(1, n) for j in range(i + 1, n)]
                random.shuffle(positions)
                positions = positions[:min(len(positions), num_neighbors)]
            else:
                # Use a distribution that favors swaps of nearby positions
                positions = []
                while len(positions) < min(num_neighbors, n * (n - 1) // 2):
                    i = random.randint(1, n - 1)
                    # Bias toward nearby positions with a probability distribution
                    max_dist = min(n - i, 5)  # Limit distance to avoid out-of-bounds
                    dist = min(1 + int(random.expovariate(1.0)), max_dist)
                    j = i + dist
                    if j < n and (i, j) not in positions:
                        positions.append((i, j))
        else:
            # Basic random sampling
            positions = []
            for _ in range(min(num_neighbors, n * (n - 1) // 2)):
                i = random.randint(1, n - 1)
                j = random.randint(1, n - 1)
                while i == j:
                    j = random.randint(1, n - 1)
                if i > j:
                    i, j = j, i
                if (i, j) not in positions:
                    positions.append((i, j))

        # Generate neighbors by swapping positions
        for i, j in positions:
            neighbor = solution.copy()
            neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
            neighbors.append(neighbor)

        return neighbors

    def solve(self, max_iterations=1000, max_time=60) -> Tuple[List[str], float, datetime.timedelta, int]:
        """
        Run Tabu Search algorithm to find optimal tour
        
        Args:
            max_iterations: Maximum number of iterations
            max_time: Maximum runtime in seconds
            
        Returns:
            Tuple of (detailed path, cost, total time, line changes)
        """
        start_time = time.time()

        # Generate initial solution
        current_solution = self._calculate_initial_solution()
        current_cost, current_path, current_total_time, current_line_changes = self._calculate_route_cost(
            current_solution)

        best_solution = current_solution
        best_cost = current_cost
        best_path = current_path
        best_total_time = current_total_time
        best_line_changes = current_line_changes

        # Initialize tabu list as a set of moves (i, j) that are forbidden
        tabu_list = {}
        tabu_tenure = self._get_tabu_list_size()

        # Main Tabu Search loop
        iteration = 0
        non_improving_iterations = 0

        while iteration < max_iterations and time.time() - start_time < max_time:
            # Generate neighbors
            num_neighbors = max(5, min(20, len(self.stops_to_visit) * 2))
            neighbors = self._generate_neighbors(current_solution, num_neighbors)

            # Evaluate neighbors
            best_neighbor = None
            best_neighbor_cost = float('inf')
            best_neighbor_path = []
            best_neighbor_time = datetime.timedelta(0)
            best_neighbor_changes = 0

            for neighbor in neighbors:
                # Convert solution to a move (swapped positions)
                move = None
                for i in range(len(neighbor)):
                    if i > 0 and neighbor[i] != current_solution[i]:
                        for j in range(i + 1, len(neighbor)):
                            if neighbor[j] == current_solution[i]:
                                move = (neighbor[i], neighbor[j])
                                break
                        if move:
                            break

                # Check if move is tabu
                is_tabu = move in tabu_list and tabu_list[move] > iteration

                # Calculate solution cost
                neighbor_cost, neighbor_path, neighbor_time, neighbor_changes = self._calculate_route_cost(neighbor)

                # Check aspiration criterion - accept tabu move if it's better than the best
                if is_tabu and self.use_aspiration and neighbor_cost < best_cost:
                    is_tabu = False
                    self.aspiration_accepted += 1

                # Update best neighbor if not tabu or passes aspiration
                if not is_tabu and neighbor_cost < best_neighbor_cost:
                    best_neighbor = neighbor
                    best_neighbor_cost = neighbor_cost
                    best_neighbor_path = neighbor_path
                    best_neighbor_time = neighbor_time
                    best_neighbor_changes = neighbor_changes
                    best_neighbor_move = move

            # No valid neighbor found
            if best_neighbor is None:
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
            else:
                non_improving_iterations += 1

            # Add current move to tabu list
            if best_neighbor_move:
                tabu_list[best_neighbor_move] = iteration + tabu_tenure
                # Also add the reverse move
                reverse_move = (best_neighbor_move[1], best_neighbor_move[0])
                tabu_list[reverse_move] = iteration + tabu_tenure

            # Dynamically adjust tabu tenure if we're stuck
            if self.use_dynamic_tabu_size and non_improving_iterations > 10:
                tabu_tenure = self._get_tabu_list_size() * (1 + non_improving_iterations // 10)
                non_improving_iterations = 0

            # Clean up expired tabu moves
            tabu_list = {k: v for k, v in tabu_list.items() if v > iteration}

            iteration += 1
            self.iterations = iteration

            # Early termination if we're not making progress
            if non_improving_iterations > 30:
                break

        print(f"Tabu search completed in {iteration} iterations, "
              f"aspiration accepted: {self.aspiration_accepted} times",
              file=sys.stderr)

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

    return tabu_search.solve()
