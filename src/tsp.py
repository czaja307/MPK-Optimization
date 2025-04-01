import random
import time
import datetime
import sys

from src.algorithms import dijkstra


class TabuSearch:
    def __init__(self, graph, start_stop, stops_to_visit, start_time, cost_function,
                 use_dynamic_tabu_size=True, use_aspiration=True, use_advanced_sampling=True):
        self.graph = graph
        self.start_stop = start_stop.lower()
        self.stops_to_visit = [stop.lower() for stop in stops_to_visit]
        self.start_time = start_time
        self.cost_function = cost_function
        self.use_dynamic_tabu_size = use_dynamic_tabu_size
        self.use_aspiration = use_aspiration
        self.use_advanced_sampling = use_advanced_sampling
        self.route_cache = {}
        self.distance_cache = {}
        self.iterations = 0
        self.aspiration_accepted = 0
        self.cache_hits = 0
        self.cache_misses = 0
        if len(stops_to_visit) <= 5:
            self._selective_precompute()
        else:
            self._precompute_from_start()

    def _selective_precompute(self):
        print(f"Selective precomputation for {len(self.stops_to_visit)} stops...", file=sys.stderr)
        all_stops = [self.start_stop] + self.stops_to_visit
        base_time = self.start_time
        if len(all_stops) <= 5:
            for from_stop in all_stops:
                for to_stop in all_stops:
                    if from_stop != to_stop:
                        key = (from_stop, to_stop, base_time.strftime('%H:%M:%S'))
                        path, costs, segment_time, line_changes = dijkstra(
                            self.graph, from_stop, to_stop, base_time, self.cost_function
                        )
                        self.route_cache[key] = (path, costs, segment_time, line_changes)
                        self.distance_cache[(from_stop, to_stop)] = (
                            segment_time.total_seconds() / 60.0, line_changes
                        )
        else:
            self._precompute_from_start()
        print(f"Precomputed {len(self.route_cache)} paths", file=sys.stderr)

    def _precompute_from_start(self):
        print("Precomputing paths from start...", file=sys.stderr)
        base_time = self.start_time
        for stop in self.stops_to_visit:
            key = (self.start_stop, stop, base_time.strftime('%H:%M:%S'))
            path, costs, segment_time, line_changes = dijkstra(
                self.graph, self.start_stop, stop, base_time, self.cost_function
            )
            self.route_cache[key] = (path, costs, segment_time, line_changes)
            self.distance_cache[(self.start_stop, stop)] = (
                segment_time.total_seconds() / 60.0, line_changes
            )
        print(f"Precomputed {len(self.route_cache)} paths from start", file=sys.stderr)

    def _get_route_segment(self, from_stop, to_stop, current_time):
        time_key = current_time.strftime('%H:%M:%S')
        key = (from_stop, to_stop, time_key)
        if key in self.route_cache:
            self.cache_hits += 1
            return self.route_cache[key]
        if len(self.stops_to_visit) <= 3:
            for cached_key in list(self.route_cache.keys()):
                cached_from, cached_to, cached_time_str = cached_key
                if cached_from == from_stop and cached_to == to_stop:
                    cached_time = datetime.datetime.strptime(cached_time_str, '%H:%M:%S')
                    time_diff = abs((cached_time - current_time).total_seconds() / 60.0)
                    if time_diff <= 15:
                        self.cache_hits += 1
                        return self.route_cache[cached_key]
        self.cache_misses += 1
        path, costs, segment_time, line_changes = dijkstra(
            self.graph, from_stop, to_stop, current_time, self.cost_function
        )
        self.route_cache[key] = (path, costs, segment_time, line_changes)
        self.distance_cache[(from_stop, to_stop)] = (
            segment_time.total_seconds() / 60.0 if segment_time else float('inf'),
            line_changes
        )
        return path, costs, segment_time, line_changes

    def calculate_route_cost(self, route, best_known_cost=float('inf')):
        total_cost = 0
        total_time = datetime.timedelta()
        total_line_changes = 0
        detailed_path = []
        current_time = self.start_time
        prev_line = None
        for i in range(len(route) - 1):
            from_stop = route[i]
            to_stop = route[i + 1]
            if self.cost_function == self.graph.get_time_cost and total_time.total_seconds() / 60.0 >= best_known_cost:
                return float('inf'), [], datetime.timedelta(0), 0
            elif self.cost_function != self.graph.get_time_cost and total_line_changes >= best_known_cost:
                return float('inf'), [], datetime.timedelta(0), 0
            if self.cost_function == self.graph.get_time_cost and i < len(route) - 2:
                remaining_time_estimate = 0
                for j in range(i + 1, len(route) - 1):
                    estimate_key = (route[j], route[j + 1])
                    if estimate_key in self.distance_cache:
                        remaining_time_estimate += self.distance_cache[estimate_key][0]
                    else:
                        remaining_time_estimate += 20
                if (total_time.total_seconds() / 60.0) + remaining_time_estimate >= best_known_cost:
                    return float('inf'), [], datetime.timedelta(0), 0
            path, costs, segment_time, line_changes = self._get_route_segment(
                from_stop, to_stop, current_time
            )
            if not path:
                return float('inf'), [], datetime.timedelta(0), 0
            detailed_path.extend(path)
            total_time += segment_time
            total_line_changes += line_changes
            current_time += segment_time
        if self.cost_function == self.graph.get_time_cost:
            final_cost = total_time.total_seconds() / 60.0
        else:
            final_cost = total_line_changes
        return final_cost, detailed_path, total_time, total_line_changes

    def calculate_initial_solution(self):
        if len(self.stops_to_visit) <= 3:
            return self.get_best_permutation()
        route = [self.start_stop]
        unvisited = set(self.stops_to_visit)
        current = self.start_stop
        current_time = self.start_time
        while unvisited:
            best_next = None
            best_cost = float('inf')
            best_time = None
            for next_stop in unvisited:
                path, costs, segment_time, _ = self._get_route_segment(
                    current, next_stop, current_time
                )
                if segment_time:
                    cost = segment_time.total_seconds() / 60.0
                    if cost < best_cost:
                        best_cost = cost
                        best_next = next_stop
                        best_time = segment_time
            if best_next is None:
                break
            route.append(best_next)
            unvisited.remove(best_next)
            current = best_next
            current_time += best_time
        return route

    def get_best_permutation(self):
        print("Evaluating all permutations for small problem", file=sys.stderr)
        import itertools
        best_route = None
        best_cost = float('inf')
        for perm in itertools.permutations(self.stops_to_visit):
            route = [self.start_stop] + list(perm)
            cost, _, _, _ = self.calculate_route_cost(route)
            if cost < best_cost:
                best_cost = cost
                best_route = route
        print(f"Best permutation has cost {best_cost}", file=sys.stderr)
        return best_route

    def get_tabu_list_size(self):
        if not self.use_dynamic_tabu_size:
            return 10
        n = len(self.stops_to_visit)
        return max(3, min(20, n))

    def generate_neighbors(self, solution, num_neighbors):
        neighbors = []
        n = len(solution)
        if n <= 4:
            for i in range(1, n - 1):
                for j in range(i + 1, n - 1):
                    neighbor = solution.copy()
                    neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
                    neighbors.append(neighbor)
            return neighbors
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

    def solve(self, max_iterations=100, max_time=30):
        start_time = time.time()
        print(f"Starting tabu search with {len(self.stops_to_visit)} stops to visit", file=sys.stderr)
        if len(self.stops_to_visit) <= 3:
            max_iterations = min(max_iterations, 50)
        current_solution = self.calculate_initial_solution()
        current_cost, current_path, current_total_time, current_line_changes = self.calculate_route_cost(
            current_solution)
        best_solution = current_solution
        best_cost = current_cost
        best_path = current_path
        best_total_time = current_total_time
        best_line_changes = current_line_changes
        tabu_list = {}
        tabu_tenure = self.get_tabu_list_size()
        iteration = 0
        non_improving_iterations = 0
        early_termination_count = 5 if len(self.stops_to_visit) <= 3 else 15
        while iteration < max_iterations and time.time() - start_time < max_time:
            num_neighbors = min(5, (len(self.stops_to_visit) * 2))
            neighbors = self.generate_neighbors(current_solution, num_neighbors)
            best_neighbor = None
            best_neighbor_cost = float('inf')
            best_neighbor_path = []
            best_neighbor_time = datetime.timedelta(0)
            best_neighbor_changes = 0
            best_move = None
            for neighbor in neighbors:
                move = None
                for i in range(len(neighbor)):
                    if i < len(current_solution) and neighbor[i] != current_solution[i]:
                        for j in range(i + 1, len(neighbor)):
                            if j < len(current_solution) and neighbor[j] != current_solution[j]:
                                move = (i, j)
                                break
                        if move:
                            break
                is_tabu = move in tabu_list and tabu_list[move] > iteration
                if is_tabu and not self.use_aspiration:
                    continue
                neighbor_cost, neighbor_path, neighbor_time, neighbor_changes = self.calculate_route_cost(
                    neighbor, best_known_cost=best_cost if self.use_aspiration else best_neighbor_cost
                )
                if neighbor_cost < float('inf') and (
                        not is_tabu or (self.use_aspiration and neighbor_cost < best_cost)):
                    if neighbor_cost < best_neighbor_cost:
                        best_neighbor = neighbor
                        best_neighbor_cost = neighbor_cost
                        best_neighbor_path = neighbor_path
                        best_neighbor_time = neighbor_time
                        best_neighbor_changes = neighbor_changes
                        best_move = move
                        if is_tabu and self.use_aspiration and neighbor_cost < best_cost:
                            self.aspiration_accepted += 1
            if best_neighbor is None:
                print("No valid neighbors found, terminating search", file=sys.stderr)
                break
            current_solution = best_neighbor
            current_cost = best_neighbor_cost
            current_path = best_neighbor_path
            current_total_time = best_neighbor_time
            current_line_changes = best_neighbor_changes
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
            if best_move:
                tabu_list[best_move] = iteration + tabu_tenure
            iteration += 1
            self.iterations = iteration
            if non_improving_iterations >= early_termination_count:
                print(f"Early termination after {non_improving_iterations} non-improving iterations", file=sys.stderr)
                break
        elapsed = time.time() - start_time
        print(f"Tabu search completed in {iteration} iterations, {elapsed:.2f}s", file=sys.stderr)
        print(f"Cache stats: {self.cache_hits} hits, {self.cache_misses} misses", file=sys.stderr)
        return best_path, best_cost, best_total_time, best_line_changes


def solve_multi_stop_problem(graph, start_stop, stops_to_visit, start_time, cost_function,
                             use_dynamic_tabu=True, use_aspiration=True, use_advanced_sampling=True):
    if len(stops_to_visit) <= 3:
        print(f"Small problem with {len(stops_to_visit)} stops - using optimized approach", file=sys.stderr)
        max_iterations = 50
        max_time = 15
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
