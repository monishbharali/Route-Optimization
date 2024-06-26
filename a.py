import folium
import openrouteservice as ors
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Initialize ORS client
client = ors.Client(key='5b3ce3597851110001cf62482f14c4be0a8c405a9d7316d5cf55a965')

# Function to get coordinates from place names
def get_coordinates(place_names, client):
    coordinates = []
    for place in place_names:
        geocode = client.pelias_search(text=place)
        if geocode and 'features' in geocode and len(geocode['features']) > 0:
            coords = geocode['features'][0]['geometry']['coordinates']
            coordinates.append(coords)
        else:
            print(f"Could not find coordinates for {place}")
    return coordinates

# Function to calculate Euclidean distance matrix
def compute_euclidean_distance_matrix(locations):
    distances = np.zeros((len(locations), len(locations)))
    for i, from_node in enumerate(locations):
        for j, to_node in enumerate(locations):
            if i != j:
                distances[i][j] = np.linalg.norm(np.array(from_node) - np.array(to_node))
    return distances

# Function to create data model for OR-Tools
def create_data_model(distance_matrix):
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

# Function to solve TSP using OR-Tools
def solve_tsp(distance_matrix):
    data = create_data_model(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        index = routing.Start(0)
        plan_output = []
        while not routing.IsEnd(index):
            plan_output.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        return plan_output
    else:
        return None

# Get place names from the user
place_names = []
print("Enter the names of the places (type 'done' when finished):")
while True:
    place = input("Place name: ")
    if place.lower() == 'done':
        break
    place_names.append(place)

# Get coordinates from place names
coords = get_coordinates(place_names, client)
if not coords:
    print("No valid coordinates found. Exiting.")
    exit()

# Get vehicle start location from the user
vehicle_start_name = input("Enter the starting location of the vehicle: ")
vehicle_start = get_coordinates([vehicle_start_name], client)
if not vehicle_start:
    print("Could not find coordinates for the starting location. Exiting.")
    exit()
vehicle_start = vehicle_start[0]

# Insert vehicle start location at the beginning
coords.insert(0, vehicle_start)

# Create distance matrix
distance_matrix = compute_euclidean_distance_matrix(coords)

# Solve TSP
route = solve_tsp(distance_matrix)
if route is None:
    print("No solution found. Exiting.")
    exit()

# Get route coordinates
route_coords = [coords[i] for i in route]

# Visualize the points on a map
m = folium.Map(location=list(reversed(vehicle_start)), tiles="cartodbpositron", zoom_start=14)
for coord in coords:
    folium.Marker(location=list(reversed(coord))).add_to(m)
folium.Marker(location=list(reversed(vehicle_start)), icon=folium.Icon(color="red")).add_to(m)

# Add TSP route to map with actual driving directions
for i in range(len(route_coords) - 1):
    start = route_coords[i]
    end = route_coords[i + 1]
    directions = client.directions(coordinates=[start, end], profile='driving-car', format='geojson')
    folium.PolyLine(locations=[list(reversed(coord)) for coord in directions['features'][0]['geometry']['coordinates']], color='blue').add_to(m)

# Connect the end back to the start for a complete loop
directions = client.directions(coordinates=[route_coords[-1], route_coords[0]], profile='driving-car', format='geojson')
folium.PolyLine(locations=[list(reversed(coord)) for coord in directions['features'][0]['geometry']['coordinates']], color='blue').add_to(m)

# Save the map
m.save('tsp_route.html')
print("Map with TSP route saved as 'tsp_route.html'")
