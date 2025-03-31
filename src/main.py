import datetime

from file_utility import load_data, load_graph
from src.optimizers import dijkstra


def get_user_input_and_run_dijkstra(graph):
    start = input("Enter the start stop: ")
    end = input("Enter the end stop: ")

    path, distances = dijkstra(graph, start, end)

    print("Shortest path:")
    for step in path:
        print(step)


def main():
    graph = load_graph()

    # print(len(graph.nodes))
    # print(sum(len(edges) for edges in graph.edges.values()))

    # path, distances = dijkstra(graph, "poprzeczna", "pl. grunwaldzki")
    path, distances = dijkstra(graph, "poprzeczna", "pl. grunwaldzki", datetime.datetime.strptime("12:00:00", '%H:%M:%S'))

    print("Shortest path:")
    for step in path:
        print(step)

    # get_user_input_and_run_dijkstra(graph)


if __name__ == "__main__":
    main()
