import pandas as pd
import requests
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


EXCEL_FILE = "orderlist.csv"
OSRM_URL = "http://localhost:5001/table/v1/driving/"
DEPOT = (77.5946, 12.9716)
NUM_VEHICLES = 5
VEHICLE_CAPACITY = 50
DEMAND_PER_CUSTOMER = 3


df = pd.read_csv(EXCEL_FILE)

print("CSV columns:", df.columns.tolist())
print("First few rows:")
print(df.head())

longitude_col = "Longitude"
latitude_col = "Latitide"


def is_valid_coordinate(lat, lon):
    try:
        lat_val = float(lat)
        lon_val = float(lon)
        if -90 <= lat_val <= 90 and -180 <= lon_val <= 180:

            if 12.5 <= lat_val <= 13.5 and 77.0 <= lon_val <= 78.0:
                return True
            elif lat_val == 25.42357674:
                print(f"Filtering out distant location: {lat_val}, {lon_val}")
                return False
    except (ValueError, TypeError):
        pass
    return False


print(f"Original data shape: {df.shape}")
valid_coords_mask = df.apply(lambda row: is_valid_coordinate(row[latitude_col], row[longitude_col]), axis=1)
df_valid = df[valid_coords_mask].copy()
print(f"After filtering invalid/distant coordinates: {df_valid.shape}")

invalid_rows = df[~valid_coords_mask]
if len(invalid_rows) > 0:
    print(f"Filtered out {len(invalid_rows)} rows with invalid/distant coordinates:")
    for idx, row in invalid_rows.iterrows():
        print(f"  Row {idx}: {row[latitude_col]}, {row[longitude_col]} - {row.get('City', 'Unknown')}")

depot_coords = f"{DEPOT[0]},{DEPOT[1]}"
customer_locations = df_valid.apply(lambda row: f"{row[longitude_col]},{row[latitude_col]}", axis=1).tolist()
locations = [depot_coords] + customer_locations

df = df_valid

print(f"Total locations: {len(locations)}")
print(f"First few locations: {locations[:3]}")


def get_distance_matrix(locations):
    base_url = "http://localhost:5001/table/v1/driving/"
    coords = ";".join(locations)
    url = f"{base_url}{coords}?annotations=distance"

    print(f"OSRM URL length: {len(url)} characters")

    r = requests.get(url)
    if r.status_code != 200:
        print(f"HTTP Status: {r.status_code}")
        print(f"Response: {r.text}")
        raise Exception(f"OSRM error: {r.text}")

    response_data = r.json()
    if "distances" not in response_data:
        raise Exception(f"No distances in response: {response_data}")

    return response_data["distances"]


try:
    distance_matrix = get_distance_matrix(locations)
    print(f"Distance matrix size: {len(distance_matrix)}x{len(distance_matrix[0])}")
except Exception as e:
    print(f"Error getting distance matrix: {e}")
    exit(1)


def create_data_model():
    data = {}
    data["distance_matrix"] = distance_matrix
    data["num_vehicles"] = NUM_VEHICLES
    data["depot"] = 0
    data["demands"] = [0] + [DEMAND_PER_CUSTOMER] * (len(distance_matrix) - 1)
    data["vehicle_capacities"] = [VEHICLE_CAPACITY] * NUM_VEHICLES
    return data


data = create_data_model()

manager = pywrapcp.RoutingIndexManager(len(data["distance_matrix"]),
                                       data["num_vehicles"], data["depot"])
routing = pywrapcp.RoutingModel(manager)


def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return int(data["distance_matrix"][from_node][to_node])


transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return data["demands"][from_node]


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,
    data["vehicle_capacities"],
    True,
    "Capacity"
)

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)
search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
)
search_parameters.time_limit.seconds = 60
search_parameters.log_search = True

total_demand = sum(data["demands"])
total_capacity = sum(data["vehicle_capacities"])
print(f"Total demand: {total_demand}")
print(f"Total capacity: {total_capacity}")
if total_demand > total_capacity:
    print("WARNING: Total demand exceeds total capacity!")
    print("Some customers may not be served in this single-trip solution")
print("Starting optimization...")
solution = routing.SolveWithParameters(search_parameters)


def simulate_multi_trip_delivery(data, manager, routing, solution):
    all_customers = set(range(1, len(data["distance_matrix"])))
    served_customers = set()
    round_number = 1

    print(f"\n=== MULTI-TRIP DELIVERY SIMULATION ===")
    print(f"Total customers to serve: {len(all_customers)}")

    while served_customers != all_customers:
        print(f"\n--- ROUND {round_number} ---")
        remaining_customers = all_customers - served_customers
        print(f"Customers remaining: {len(remaining_customers)}")

        round_served = set()
        total_round_distance = 0

        for vehicle_id in range(data["num_vehicles"]):
            if not remaining_customers:
                break

            index = routing.Start(vehicle_id)
            plan_output = f"\nVehicle {vehicle_id + 1} - Round {round_number}:\n"
            route_distance = 0
            route_load = 0
            stops_count = 0

            plan_output += f"  → Start from DEPOT\n"

            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)

                if node_index != 0:
                    if node_index in remaining_customers and route_load + data["demands"][
                        node_index] <= VEHICLE_CAPACITY:
                        stops_count += 1
                        route_load += data["demands"][node_index]
                        round_served.add(node_index)

                        customer_idx = node_index - 1
                        if customer_idx < len(df):
                            customer_name = df.iloc[customer_idx].get('Customer Name', f'Customer {customer_idx + 1}')
                            city = df.iloc[customer_idx].get('City', 'Unknown')
                            order_value = df.iloc[customer_idx].get('Order Value', 'N/A')
                            plan_output += f"  → Stop {stops_count}: {customer_name} ({city}) - Order: ₹{order_value}\n"
                        else:
                            plan_output += f"  → Stop {stops_count}: Customer {customer_idx + 1}\n"

                previous_index = index
                index = solution.Value(routing.NextVar(index))
                if not routing.IsEnd(index):
                    route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

            if stops_count > 0:
                plan_output += f"  → Return to DEPOT\n"
                plan_output += f"  Distance: {route_distance / 1000:.2f} km, Stops: {stops_count}, Load: {route_load}\n"
                print(plan_output)
                total_round_distance += route_distance

        served_customers.update(round_served)
        print(f"Round {round_number} summary:")
        print(f"  Customers served this round: {len(round_served)}")
        print(f"  Total customers served: {len(served_customers)}")
        print(f"  Round distance: {total_round_distance / 1000:.2f} km")

        if not round_served:
            print("WARNING: Could not serve any more customers. Check capacity constraints.")
            break

        round_number += 1

    print(f"\n=== FINAL SUMMARY ===")
    print(f"Total rounds needed: {round_number - 1}")
    print(f"Customers served: {len(served_customers)} out of {len(all_customers)}")
    if served_customers != all_customers:
        unserved = all_customers - served_customers
        print(f"Unserved customers: {list(unserved)}")


def print_solution(data, manager, routing, solution):
    """Print the single-trip solution first"""
    print(f"\n=== SINGLE-TRIP SOLUTION (for reference) ===")
    total_distance = 0
    total_load = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for Vehicle {vehicle_id + 1}:\n"
        route_distance = 0
        route_load = 0
        stops_count = 0

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]

            if node_index == 0:
                location_info = f"DEPOT"
            else:
                stops_count += 1
                customer_idx = node_index - 1
                if customer_idx < len(df):
                    customer_info = df.iloc[customer_idx].get('Customer Name', f'Customer {customer_idx + 1}')
                    city = df.iloc[customer_idx].get('City', 'Unknown')
                    location_info = f"{customer_info} ({city})"
                else:
                    location_info = f"Customer {customer_idx + 1}"

            plan_output += f" → {location_info} (Load: {route_load}) "

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        plan_output += f" → DEPOT\n"
        plan_output += f"Distance: {route_distance / 1000:.2f} km, Stops: {stops_count}\n"

        if stops_count > 0:
            print(plan_output)

        total_distance += route_distance
        total_load += route_load

    print(f"Total distance: {total_distance / 1000:.2f} km")
    print(f"Total load: {total_load}")

    simulate_multi_trip_delivery(data, manager, routing, solution)


if solution:
    print_solution(data, manager, routing, solution)
else:
    print("No solution found!")

    print("Debug info:")
    print(f"Number of locations: {len(locations)}")
    print(f"Number of vehicles: {NUM_VEHICLES}")
    print(f"Vehicle capacity: {VEHICLE_CAPACITY}")
    print(f"Total demand: {sum(data['demands'])}")