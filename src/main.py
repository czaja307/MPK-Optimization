import datetime
import sys
import time

from utils import load_data, load_graph
from src.algorithms import dijkstra


def get_user_input_and_run_dijkstra(graph):
    start = input("Enter the start stop: ")
    end = input("Enter the end stop: ")
    optimization_criterion = input("Enter the optimization criterion (t - time, p - line changes): ")
    start_time = input("Enter the start time (HH:MM:SS): ")
    start_time = datetime.datetime.strptime(start_time, '%H:%M:%S')

    if optimization_criterion == 't':
        cost_function = graph.get_time_cost
    elif optimization_criterion == 'p':
        cost_function = graph.get_line_change_cost
    else:
        print("Invalid optimization criterion. Defaulting to time.")
        cost_function = graph.get_time_cost

    alg_start_time = time.time()
    path, distances = dijkstra(graph, start, end, start_time, cost_function)
    alg_end_time = time.time()
    execution_time = alg_end_time - alg_start_time
    print(f"Dijkstra execution time: {execution_time:.4f} seconds", file=sys.stderr)

    print("Shortest path:")
    for step in path:
        print(step)


def main():
    graph = load_graph()

    # print(len(graph.nodes))
    # print(sum(len(edges) for edges in graph.edges.values()))

    # path, distances = dijkstra(graph, "poprzeczna", "pl. grunwaldzki")
    path, distances, total_time, line_changes = dijkstra(graph, "pl. grunwaldzki", "arkady (capitol)",
                                                         datetime.datetime.strptime("12:00:00", '%H:%M:%S'),
                                                         graph.get_time_cost)
    path, distances, total_time, line_changes = dijkstra(graph, "poprzeczna", "rondo",
                                                         datetime.datetime.strptime("12:00:00", '%H:%M:%S'),
                                                         graph.get_line_cost)

    print("Shortest path:")
    for step in path:
        print(step)

    print("Time", total_time)
    print("Line changes:", line_changes)
    # get_user_input_and_run_dijkstra(graph)


if __name__ == "__main__":
    main()
