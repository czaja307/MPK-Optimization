data_file_path = "data.csv"

csv_columns = ["line", "departure_time", "arrival_time", "start_stop", "end_stop", "start_stop_lat",
               "start_stop_lon", "end_stop_lat", "end_stop_lon"]

graph_file = "graph.pkl"

# Constants for Manhattan distance heuristic
km_per_deg = 111.32  # Approximate kilometers per degree of latitude/longitude
mins_per_km = 2  # Approximate minutes per kilometer for public transport
