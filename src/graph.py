import datetime
from bisect import bisect
import math
from constants import km_per_deg, mins_per_km


class Graph:
    def __init__(self):
        self.nodes = dict()
        self.edges = dict()

    def create_from_dataframe(self, df):
        for _, row in df.iterrows():
            self.add_node(row["start_stop"], row["start_stop_lat"], row["start_stop_lon"])
            self.add_node(row["end_stop"], row["end_stop_lat"], row["end_stop_lon"])
            edge = Edge(
                row['line'], row['departure_time'], row['arrival_time'], row['start_stop'], row['end_stop'],
                row['start_stop_lat'], row['start_stop_lon'], row['end_stop_lat'], row['end_stop_lon']
            )
            self.add_edge(edge)

    def add_node(self, name, lat, long):
        if name not in self.nodes:
            self.nodes[name] = Node(name)
        self.nodes[name].add_location(lat, long)

    def add_edge(self, edge):
        # Add the edge to the graph's edge collection
        if edge.start_stop not in self.edges:
            self.edges[edge.start_stop] = []
        self.edges[edge.start_stop].append(edge)

        # Also add the edge to the start node's outgoing edges
        self.nodes[edge.start_stop].add_edge(edge)

    def get_time_cost(self, start, end, time: datetime.datetime, last_line, used_lines):
        possible_paths = self.nodes[start].get_sorted_edges(end)
        if not possible_paths:
            return float('inf'), None

        soonest_id = self.find_first_id_after(possible_paths, time)
        if soonest_id is None:
            soonest_id = 0
        cost = possible_paths[soonest_id].get_travel_time() + abs(
            (possible_paths[soonest_id].arrival_time - time).seconds / 60.0)
        if possible_paths[soonest_id].line != last_line:
            cost += 0
        return cost, possible_paths[soonest_id]

    def get_line_cost(self, start, end, time, last_line, used_lines):
        possible_paths = self.nodes[start].get_sorted_edges(end)
        if not possible_paths:
            return float('inf'), None

        soonest_id = self.find_first_id_after(possible_paths, time)
        if soonest_id is None:
            soonest_id = 0

        travel_time = possible_paths[soonest_id].get_travel_time()
        waiting_time = abs((possible_paths[soonest_id].departure_time - time).seconds / 60.0)

        cost = travel_time + waiting_time
        if possible_paths[soonest_id].line != last_line and last_line is not None:
            cost += 100 * used_lines

        return cost, possible_paths[soonest_id]

    def find_first_id_after(self, paths, time: datetime.datetime, line=None):
        if not paths:
            return None

        path_i = bisect(paths, time, key=lambda path: path.departure_time)
        i = 0
        while path_i - i - 1 >= 0 and paths[path_i - i - 1].arrival_time == time:
            if line is not None:
                if paths[path_i - i - 1].line == line:
                    return path_i - i - 1
            i += 1
        if 0 <= path_i - i < len(paths):
            return path_i - i
        return None

    def get_distance(self, from_stop, to_stop):
        if from_stop not in self.nodes or to_stop not in self.nodes:
            return 0.0
        lat1, lon1 = self.nodes[from_stop].get_avg_location()
        lat2, lon2 = self.nodes[to_stop].get_avg_location()
        # Use Manhattan distance heuristic instead of Haversine formula
        return (abs(lon2 - lon1) + abs(lat2 - lat1)) * km_per_deg * mins_per_km

    def __str__(self):
        total_edges = sum(len(edges) for edges in self.edges.values())
        return f"Graph with {len(self.nodes)} nodes and {total_edges} edges"


class Edge:
    def __init__(self, line, departure_time, arrival_time, start_stop, end_stop, start_stop_lat, start_stop_lon,
                 end_stop_lat, end_stop_lon):
        self.line = line
        dep_time = str(int(departure_time[:2]) % 24) + departure_time[2:]
        arr_time = str(int(arrival_time[:2]) % 24) + arrival_time[2:]
        self.departure_time = datetime.datetime.strptime(dep_time, '%H:%M:%S')
        self.arrival_time = datetime.datetime.strptime(arr_time, '%H:%M:%S')
        self.start_stop = start_stop
        self.end_stop = end_stop
        self.start_stop_lat = start_stop_lat
        self.start_stop_lon = start_stop_lon
        self.end_stop_lat = end_stop_lat
        self.end_stop_lon = end_stop_lon

    def get_travel_time(self):
        return (self.arrival_time - self.departure_time).seconds / 60

    def __str__(self):
        return f"Edge {self.line} from {self.start_stop} to {self.end_stop} at {self.departure_time}"


class Node:
    def __init__(self, name):
        self.name = name
        self.locations = set()
        self.outgoing_edges = {}  # Changed from list to dictionary
        self.was_sorted = False

    def add_location(self, lat, lon):
        self.locations.add((lat, lon))
        self.was_sorted = False

    def add_edge(self, edge):
        # Add the edge to the dictionary using end_stop as key
        if edge.end_stop not in self.outgoing_edges:
            self.outgoing_edges[edge.end_stop] = []
        self.outgoing_edges[edge.end_stop].append(edge)
        self.was_sorted = False

    def get_avg_location(self):
        lat_sum = 0
        lon_sum = 0
        for lat, lon in self.locations:
            lat_sum += lat
            lon_sum += lon
        return lat_sum / len(self.locations), lon_sum / len(self.locations)

    def get_sorted_edges(self, end_stop):
        # Get sorted edges for a specific end_stop
        if end_stop not in self.outgoing_edges:
            return []

        if not self.was_sorted:
            for dest in self.outgoing_edges:
                self.outgoing_edges[dest] = sorted(self.outgoing_edges[dest], key=lambda path: path.arrival_time)
            self.was_sorted = True

        return self.outgoing_edges[end_stop]

    def __str__(self):
        total_edges = sum(len(edges) for edges in self.outgoing_edges.values())
        return f"Node {self.name} with {len(self.locations)} locations and {total_edges} outgoing edges"

