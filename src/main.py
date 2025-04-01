import datetime
import sys
import time

from utils import load_data, load_graph
from src.algorithms import dijkstra, astar
from src.tsp import solve_multi_stop_problem


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


def get_user_input_and_run_multi_stop(graph):
    """Handle user input for multi-stop route problem and run the solver"""
    start_stop = input()
    stops_to_visit = input().strip().split(';')
    optimization_criterion = input().strip()
    start_time_str = input().strip()

    start_time = datetime.datetime.strptime(start_time_str, '%H:%M:%S')

    if optimization_criterion == 't':
        cost_function = graph.get_time_cost
    elif optimization_criterion == 'p':
        cost_function = graph.get_line_cost
    else:
        print("Invalid optimization criterion. Defaulting to time.")
        cost_function = graph.get_time_cost

    alg_start_time = time.time()

    path, cost, total_time, line_changes = solve_multi_stop_problem(
        graph, start_stop, stops_to_visit, start_time, cost_function,
        use_dynamic_tabu=True, use_aspiration=True, use_advanced_sampling=True
    )

    alg_end_time = time.time()
    execution_time = alg_end_time - alg_start_time

    # Print the detailed path
    for step in path:
        print(step)

    # Print cost and execution time to stderr
    if cost_function == graph.get_time_cost:
        print(f"Total travel time: {total_time}", file=sys.stderr)
    else:
        print(f"Total line changes: {line_changes}", file=sys.stderr)
    print(f"Execution time: {execution_time:.4f} seconds", file=sys.stderr)


def main():
    graph = load_graph()
    start_stop = "pl. grunwaldzki"
    end_stop = "wrocławski park przemysłowy"
    start_time = datetime.datetime.strptime("14:30:00", '%H:%M:%S')

    # Check if the user wants to run the multi-stop routing problem
    if len(sys.argv) > 1 and sys.argv[1] == '--multi':
        get_user_input_and_run_multi_stop(graph)
        return

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
    dijkstra_line_start = time.time()
    path_line, costs_line, total_time_line, line_changes_line = dijkstra(
        graph, start_stop, end_stop, start_time, graph.get_line_cost
    )
    dijkstra_line_end = time.time()
    print(f"Execution time: {dijkstra_line_end - dijkstra_line_start:.4f} seconds")
    print("Dijkstra shortest path (line changes):")
    for step in path_line:
        print(step)
    print(f"Time: {total_time_line}")
    print(f"Line changes: {line_changes_line}")

    astar_line_start = time.time()
    path_line_astar, costs_line_astar, total_time_astar_line, line_changes_astar_line = astar(
        graph, start_stop, end_stop, start_time, graph.get_line_cost
    )
    astar_line_end = time.time()
    print(f"Execution time: {astar_line_end - astar_line_start:.4f} seconds")
    print("A* shortest path (line changes):")
    for step in path_line_astar:
        print(step)
    print(f"Time: {total_time_astar_line}")
    print(f"Line changes: {line_changes_astar_line}")

    print("\n=== Tabu Search for multi-stop routing ===")
    # Define some stops to visit
    multi_stops = ["wrocławski park przemysłowy", "poprzeczna", "galeria dominikańska"]
    tbs_start = time.time()
    tbs_path, tbs_cost, tbs_time, tbs_line_changes = solve_multi_stop_problem(
        graph, start_stop, multi_stops, start_time, graph.get_time_cost,
        use_dynamic_tabu=True, use_aspiration=True, use_advanced_sampling=True
    )
    tbs_end = time.time()
    print(f"Execution time: {tbs_end - tbs_start:.4f} seconds")
    print("Tabu Search multi-stop path:")
    for step in tbs_path:
        print(step)
    print(f"Total travel time: {tbs_time}")
    print(f"Line changes: {tbs_line_changes}")


if __name__ == "__main__":
    main()
