import os.path
import pickle

import pandas as pd

from constants import csv_columns, data_file_path, graph_file
from graph import Graph


def load_data():
    data = pd.read_csv(data_file_path, dtype={2: str}, usecols=csv_columns).drop_duplicates()
    data["start_stop"] = data["start_stop"].str.lower()
    data["end_stop"] = data["end_stop"].str.lower()
    return data


def load_graph():
    if os.path.exists(graph_file):
        try:
            with open(graph_file, 'rb') as f:
                graph = pickle.load(f)
            print("Graph loaded from file.")
        except (pickle.UnpicklingError, EOFError, FileNotFoundError):
            print("Failed to load graph from file, creating a new one.")
            graph = create_and_pickle_graph()
    else:
        print("Graph file not found, creating a new one.")
        graph = create_and_pickle_graph()

    return graph


def create_and_pickle_graph():
    data = load_data()
    print(data)

    graph = Graph()
    graph.create_from_dataframe(data)

    with open(graph_file, 'wb') as f:
        pickle.dump(graph, f)

    return graph
