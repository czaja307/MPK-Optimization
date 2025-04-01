import heapq
import math


def dijkstra(graph, start, end, start_time, cost_function):
    start_node = None
    end_node = None
    for node in graph.nodes:
        if node == start.lower():
            start_node = node
        if node == end.lower():
            end_node = node

    if start_node is None or end_node is None:
        raise ValueError("Start or end node not found in the graph")

    # Initialize costs and priority queue
    costs = {node: float('infinity') for node in graph.nodes}
    costs[start] = 0
    priority_queue = [(0, start, start_time, None, 0)]  # (cost, node, current_time, last_line, used_lines)
    previous_nodes = {node: None for node in graph.nodes}
    previous_edges = {node: None for node in graph.nodes}

    while priority_queue:
        current_cost, current_node, current_time, last_line, line_changes = heapq.heappop(priority_queue)

        if current_cost > costs[current_node]:
            continue

        for edge in graph.edges.get(current_node, []):
            neighbor = edge.end_stop

            # Only consider edges with departure time after current time
            if edge.departure_time < current_time:
                continue

            weight, edge_used = cost_function(current_node, neighbor, current_time, last_line, line_changes)

            if edge_used is None:
                continue

            distance = current_cost + weight
            new_time = edge_used.arrival_time  # Update the time to the arrival time of this edge

            if distance < costs[neighbor]:
                costs[neighbor] = distance
                previous_nodes[neighbor] = current_node
                previous_edges[neighbor] = edge_used
                new_line_changes = line_changes
                if last_line is not None and edge_used.line != last_line:
                    new_line_changes += 1
                heapq.heappush(priority_queue, (distance, neighbor, new_time, edge_used.line, new_line_changes))

    # Reconstruct path
    path, current_node = [], end
    while previous_nodes[current_node] is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    if path:
        path.insert(0, current_node)

    # Format the path using the stored edges
    formatted_path = []
    for i in range(len(path) - 1):
        edge = previous_edges[path[i + 1]]
        formatted_path.append(
            f"{path[i]} -> {path[i + 1]} (Line: {edge.line},"
            f" Travel Time: {edge.get_travel_time()} mins,"
            f" Departure Time: {edge.departure_time.strftime('%H:%M:%S')},"
            f" Arrival Time: {edge.arrival_time.strftime('%H:%M:%S')})")

    total_time = previous_edges[path[-1]].arrival_time - start_time
    line_changes = 0
    prev_line = None
    for i in range(len(path) - 1):
        edge = previous_edges[path[i + 1]]
        if prev_line is not None and edge.line != prev_line:
            line_changes += 1
        prev_line = edge.line

    return formatted_path, costs, total_time, line_changes


def astar(graph, start, end, start_time, cost_function):
    costs = {node: float('inf') for node in graph.nodes}
    costs[start] = 0
    previous_nodes = {node: None for node in graph.nodes}
    previous_edges = {node: None for node in graph.nodes}
    priority_queue = []

    # We can use a distance heuristic if cost_function is time-based:
    base_heuristic = graph.get_distance(start.lower(), end.lower()) if cost_function == graph.get_time_cost else 0
    heapq.heappush(priority_queue, (base_heuristic, 0, start, start_time, None, 0))

    while priority_queue:
        _, current_cost, current_node, current_time, last_line, line_changes = heapq.heappop(priority_queue)
        if current_node == end.lower():
            path, total_time, line_changes = _reconstruct_with_details(
                previous_nodes, previous_edges, current_node, start_time
            )
            return path, current_cost

        if current_cost > costs[current_node]:
            continue

        for edge in graph.edges.get(current_node, []):
            if edge.departure_time < current_time:
                continue
            travel_cost, edge_used = cost_function(
                current_node, edge.end_stop, current_time, last_line, line_changes
            )
            if edge_used is None:
                continue
            distance = current_cost + travel_cost
            if distance < costs[edge.end_stop]:
                costs[edge.end_stop] = distance
                previous_nodes[edge.end_stop] = current_node
                previous_edges[edge.end_stop] = edge_used
                # Heuristic is distance if time cost is chosen, else 0
                heuristic = graph.get_distance(edge.end_stop, end.lower()) \
                    if cost_function == graph.get_time_cost else 0
                heapq.heappush(priority_queue, (distance + heuristic, distance,
                                                edge.end_stop, edge_used.arrival_time,
                                                edge_used.line, line_changes))

    return [], float('inf')


def _reconstruct_with_details(previous_nodes, previous_edges, current_node, start_time):
    # Helper for path reconstruction, almost identical to dijkstra
    path = []
    while previous_nodes[current_node] is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    if path:
        path.insert(0, current_node)

    formatted_path = []
    for i in range(len(path) - 1):
        edge = previous_edges[path[i + 1]]
        formatted_path.append(
            f"{path[i]} -> {path[i + 1]} (Line: {edge.line}, "
            f"Travel Time: {edge.get_travel_time()} mins, "
            f"Departure: {edge.departure_time.strftime('%H:%M:%S')}, "
            f"Arrival: {edge.arrival_time.strftime('%H:%M:%S')})"
        )
    total_time = previous_edges[path[-1]].arrival_time - start_time if path else 0
    line_changes = 0
    prev_line = None
    for i in range(len(path) - 1):
        edge = previous_edges[path[i + 1]]
        if prev_line is not None and edge.line != prev_line:
            line_changes += 1
        prev_line = edge.line

    return formatted_path, total_time, line_changes


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.insert(0, current)
    return path
