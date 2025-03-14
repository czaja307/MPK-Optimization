import datetime


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
        self.edges[edge.start_stop] = edge

    def __str__(self):
        return f"Graph with {len(self.nodes)} nodes and {len(self.edges)} edges"


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
        return (self.arrival_time - self.departure_time) / 60

    def __str__(self):
        return f"Edge {self.line} from {self.start_stop} to {self.end_stop} at {self.departure_time}"


class Node:
    def __init__(self, name):
        self.name = name
        self.locations = set()

    def add_location(self, lat, lon):
        self.locations.add((lat, lon))

    def get_avg_location(self):
        lat_sum = 0
        lon_sum = 0
        for lat, lon in self.locations:
            lat_sum += lat
            lon_sum += lon
        return lat_sum / len(self.locations), lon_sum / len(self.locations)

    def __str__(self):
        return f"Node {self.name} with {len(self.locations)} locations"
