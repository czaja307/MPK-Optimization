import heapq


def dijkstra(graph, start, end):
    start_node = None
    end_node = None
    for node in graph.nodes:
        if node == start.lower():
            start_node = node
        if node == end.lower():
            end_node = node

    if start_node is None or end_node is None:
        raise ValueError("Start or end node not found in the graph")

    # Initialize distances and priority queue
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {node: None for node in graph.nodes}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        for edge in graph.edges.get(current_node, []):
            neighbor = edge.end_stop
            weight = edge.get_travel_time()
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    path, current_node = [], end
    while previous_nodes[current_node] is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    if path:
        path.insert(0, current_node)

    formatted_path = []
    for i in range(len(path) - 1):
        for edge in graph.edges[path[i]]:
            if edge.end_stop == path[i + 1]:
                formatted_path.append(
                    f"{path[i]} -> {path[i + 1]} (Line: {edge.line},"
                    f" Travel Time: {edge.get_travel_time()} mins,"
                    f" Departure Time: {edge.departure_time.strftime('%H:%M:%S')})")
                break

    return formatted_path, distances
