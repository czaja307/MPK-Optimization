import datetime
import sys
import time

from utils import load_data, load_graph
from src.algorithms import dijkstra, astar


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
    start_stop = "poprzeczna"
    end_stop = "rondo"
    start_time = datetime.datetime.strptime("12:00:00", '%H:%M:%S')
    
    print("\n=== Dijkstra's Algorithm ===")
    dijkstra_start = time.time()
    path_time, costs_time, total_time, line_changes = dijkstra(
        graph, start_stop, end_stop, start_time, graph.get_time_cost
    )
    dijkstra_end = time.time()
    print(f"Execution time: {dijkstra_end - dijkstra_start:.4f} seconds")
    print("Shortest path (time):")
    for step in path_time:
        print(step)
    print(f"Time: {total_time}")
    print(f"Line changes: {line_changes}")
    
    print("\n=== A* Algorithm ===")
    astar_start = time.time()
    path_time_astar, costs_time_astar, total_time_astar, line_changes_astar = astar(
        graph, start_stop, end_stop, start_time, graph.get_time_cost
    )
    astar_end = time.time()
    print(f"Execution time: {astar_end - astar_start:.4f} seconds")
    print("A* shortest path (time):")
    for step in path_time_astar:
        print(step)
    print(f"Time: {total_time_astar}")
    print(f"Line changes: {line_changes_astar}")
    
    print("\n=== Comparing line change optimization ===")
    path_line, costs_line, total_time_line, line_changes_line = dijkstra(
        graph, start_stop, end_stop, start_time, graph.get_line_cost
    )
    print("Dijkstra shortest path (line changes):")
    for step in path_line:
        print(step)
    print(f"Time: {total_time_line}")
    print(f"Line changes: {line_changes_line}")
    
    path_line_astar, costs_line_astar, total_time_astar_line, line_changes_astar_line = astar(
        graph, start_stop, end_stop, start_time, graph.get_line_cost
    )
    print("A* shortest path (line changes):")
    for step in path_line_astar:
        print(step)
    print(f"Time: {total_time_astar_line}")
    print(f"Line changes: {line_changes_astar_line}")

if __name__ == "__main__":
    main()

