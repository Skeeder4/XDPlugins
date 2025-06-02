import numpy as np
import math
import argparse
import csv
from pathlib import Path

# --- OR-Tools specific imports and definitions ---
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Earth radius in kilometers
R = 6371
# Average vehicle speed in km/h
AVERAGE_SPEED_KMH = 50
TIME_LIMIT_SEC = 30 # As per prompt

# --- Data Classes ---
class Client:
    def __init__(self, id, lat, lon, demand, ready, due, service, node_idx=-1): # Added node_idx
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.demand = int(demand)
        self.ready = int(ready)
        self.due = int(due)
        self.service = int(service)
        self.node_idx = node_idx # Its index in the global 'all_nodes' list

class Vehicle:
    def __init__(self, id, start_lat, start_lon, cap, start_time, end_time, start_node_idx=-1, end_node_idx=-1): # Added node_idx
        self.id = str(id)
        self.start_lat = float(start_lat)
        self.start_lon = float(start_lon)
        self.cap = int(cap)
        self.start_time = int(start_time)
        self.end_time = int(end_time)
        self.start_node_idx = start_node_idx # Actual start node index in all_nodes
        self.end_node_idx = end_node_idx   # Actual end node index in all_nodes


class Transfer:
    def __init__(self, id, lat, lon, w_start, w_end, node_idx=-1): # Added node_idx
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.w_start = int(w_start)
        self.w_end = int(w_end)
        self.demand = 0 # Transfers don't have demand in this model
        self.service = 0 # Transfers don't have service time
        self.node_idx = node_idx # Its index in the global 'all_nodes' list

class Depot: # For distinct vehicle start/end locations if not co-located
    def __init__(self, id, lat, lon, node_idx=-1):
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.demand = 0
        self.service = 0
        self.ready = 0
        self.due = 24 * 60 # Open all day
        self.node_idx = node_idx


def load_csv_data(clients_fp, vehicles_fp, transfers_fp):
    clients_data = []
    vehicles_data = []
    transfers_data = []

    try:
        with open(clients_fp, mode='r', newline='') as file:
            reader = csv.DictReader(file)
            for row in reader:
                clients_data.append(Client(**row))

        with open(vehicles_fp, mode='r', newline='') as file:
            reader = csv.DictReader(file)
            for row in reader:
                vehicles_data.append(Vehicle(**row))

        with open(transfers_fp, mode='r', newline='') as file:
            reader = csv.DictReader(file)
            for row in reader:
                transfers_data.append(Transfer(**row))

    except FileNotFoundError as e:
        print(f"Error: File not found - {e.filename}")
        return None, None, None
    except Exception as e:
        print(f"Error loading CSV data: {e}")
        return None, None, None

    return clients_data, vehicles_data, transfers_data

def build_matrices(nodes):
    n = len(nodes)
    dist_matrix = np.zeros((n, n), dtype=float)
    time_matrix = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j in range(n):
            if i == j: continue
            node1 = nodes[i]
            node2 = nodes[j]
            lat1_rad, lon1_rad = math.radians(node1.lat), math.radians(node1.lon)
            lat2_rad, lon2_rad = math.radians(node2.lat), math.radians(node2.lon)
            dlon, dlat = lon2_rad - lon1_rad, lat2_rad - lat1_rad
            a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance_km = R * c
            dist_matrix[i, j] = distance_km
            time_minutes = (distance_km / AVERAGE_SPEED_KMH) * 60
            time_matrix[i, j] = int(round(time_minutes))
    return dist_matrix, time_matrix

def add_dimensions(routing, manager, dist_m, time_m, vehicles, nodes, clients_subset_for_tw):
    # Using clients_subset_for_tw for client-specific time windows
    # 'nodes' is the full list of all_nodes used by the manager

    # Distance callback (not used for cost, but can be added as a dimension if needed)
    # def distance_callback(from_index, to_index):
    #     from_node = manager.IndexToNode(from_index); to_node = manager.IndexToNode(to_index)
    #     return dist_m[from_node][to_node]
    # dist_callback_index = routing.RegisterTransitCallback(distance_callback)

    def time_evaluator(from_index, to_index):
        from_node_idx = manager.IndexToNode(from_index)
        to_node_idx = manager.IndexToNode(to_index)
        service_time = 0
        if from_node_idx < len(nodes):
            node_obj = nodes[from_node_idx]
            if hasattr(node_obj, 'service'): service_time = node_obj.service
        return time_m[from_node_idx][to_node_idx] + service_time
    time_evaluator_index = routing.RegisterTransitCallback(time_evaluator)

    horizon = 24 * 60
    routing.AddDimension(time_evaluator_index, slack_max=horizon, capacity=horizon,
                         fix_start_cumul_to_zero=False, name="Time")
    time_dimension = routing.GetDimensionOrDie("Time")

    def demand_evaluator(from_index):
        from_node_idx = manager.IndexToNode(from_index)
        # Depot nodes (if created separately) and Transfer nodes have 0 demand by default in their class
        # Client nodes will have their specific demand
        return nodes[from_node_idx].demand
    demand_evaluator_index = routing.RegisterUnaryTransitCallback(demand_evaluator)

    vehicle_capacities = [v.cap for v in vehicles]
    routing.AddDimensionWithVehicleCapacity(demand_evaluator_index, 0, vehicle_capacities,
                                           True, "Capacity")
    capacity_dimension = routing.GetDimensionOrDie("Capacity")

    for v_idx, vehicle in enumerate(vehicles):
        start_idx = routing.Start(v_idx)
        end_idx = routing.End(v_idx)
        time_dimension.CumulVar(start_idx).SetRange(vehicle.start_time, horizon)
        time_dimension.CumulVar(end_idx).SetRange(0, vehicle.end_time)
        # Capacity starts at 0 for vehicles due to fix_start_cumul_to_zero=True for Capacity dim

    # Client time windows applied to their actual node indices in 'all_nodes'
    for client_obj in clients_subset_for_tw: # clients_subset_for_tw are original Client objects
        # Their node_idx attribute should be set when all_nodes is constructed
        if client_obj.node_idx == -1: continue # Should not happen if processed correctly
        model_idx_for_client = manager.NodeToIndex(client_obj.node_idx)
        time_dimension.CumulVar(model_idx_for_client).SetRange(client_obj.ready, client_obj.due)

    return time_dimension, capacity_dimension

# add_pickups_deliveries function (remains commented out for stability)
# def add_pickups_deliveries(routing, manager, clients, nodes_map): ...

def write_outputs(args, solution_df, kpi_dict): # Placeholder
    output_dir = Path(args.out)
    # output_dir.mkdir(exist_ok=True) # Done in main now
    print(f"\n[write_outputs] Intended output directory: {output_dir}")
    if solution_df is not None and not solution_df.empty:
        solution_fp = output_dir / "solution_routes.csv"
        # solution_df.to_csv(solution_fp, index=False)
        print(f"[write_outputs] Solution routes would be saved to {solution_fp} (currently skipped)")
    else:
        print("[write_outputs] No solution DataFrame to save.")

    kpi_fp = output_dir / "kpi.txt"
    # with open(kpi_fp, "w") as f:
    #     for k, v in kpi_dict.items():
    #         f.write(f"{k}: {v}\n")
    print(f"[write_outputs] KPIs would be saved to {kpi_fp} (currently skipped)")


def main():
    parser = argparse.ArgumentParser(description="Mobile Hub Delivery Optimizer")
    parser.add_argument("--clients", required=True, help="Path to clients CSV file")
    parser.add_argument("--vehicles", required=True, help="Path to vehicles CSV file")
    parser.add_argument("--transfers", required=True, help="Path to transfers CSV file")
    parser.add_argument("--out", default="mobile_hub_optimizer/output", help="Output directory path")
    args = parser.parse_args()

    Path(args.out).mkdir(parents=True, exist_ok=True) # Ensure output dir exists

    clients_list, vehicles_list, transfers_list = load_csv_data(args.clients, args.vehicles, args.transfers)

    if clients_list is None:
        print("Failed to load data. Exiting.")
        return

    # --- Construct all_nodes list and map vehicle start/end locations ---
    # Order: Clients, then Transfers, then Depots (if any are distinct)
    all_nodes = []
    node_idx_counter = 0

    # Add clients
    for c in clients_list:
        c.node_idx = node_idx_counter
        all_nodes.append(c)
        node_idx_counter += 1

    # Add transfers
    for t in transfers_list:
        t.node_idx = node_idx_counter
        all_nodes.append(t)
        node_idx_counter += 1

    # Create and add distinct depot nodes based on vehicle start/end locations
    # This is crucial: OR-Tools needs specific node indices for vehicle starts/ends.
    depot_locations = {} # To store unique lat/lon for depots and their node_idx

    for v_idx, v in enumerate(vehicles_list):
        start_loc_key = (v.start_lat, v.start_lon)
        # Check if this location already exists as a client or transfer node
        # This simplified check assumes exact lat/lon match.
        # A more robust method would use a small tolerance or pre-assigned depot IDs.

        assigned_start_node = -1
        for node_in_all_nodes in all_nodes: # Check against already added clients/transfers
            if math.isclose(node_in_all_nodes.lat, v.start_lat) and \
               math.isclose(node_in_all_nodes.lon, v.start_lon):
                assigned_start_node = node_in_all_nodes.node_idx
                break

        if assigned_start_node != -1:
            v.start_node_idx = assigned_start_node
        else: # New, distinct depot location
            if start_loc_key not in depot_locations:
                depot_id = f"Depot_S{len(depot_locations)+1}"
                new_depot = Depot(id=depot_id, lat=v.start_lat, lon=v.start_lon, node_idx=node_idx_counter)
                all_nodes.append(new_depot)
                depot_locations[start_loc_key] = node_idx_counter
                v.start_node_idx = node_idx_counter
                node_idx_counter += 1
            else: # Depot for this lat/lon already created
                v.start_node_idx = depot_locations[start_loc_key]

        # For this model, assume vehicles return to their start node.
        v.end_node_idx = v.start_node_idx


    if not all_nodes:
        print("No nodes (clients, transfers, depots) to process.")
        return

    dist_m, time_m = build_matrices(all_nodes)

    print("\n--- OR-Tools Setup ---")
    print(f"Total nodes for routing model (incl. depots): {len(all_nodes)}")
    # for i, n in enumerate(all_nodes): print(f"Node {i}: id={n.id} type={type(n).__name__} lat={n.lat} lon={n.lon}")
    # print(f"Vehicles with assigned start_node_idx: ")
    # for v in vehicles_list: print(f"  {v.id} starts at node {v.start_node_idx} (loc: {v.start_lat},{v.start_lon})")

    manager = pywrapcp.RoutingIndexManager(len(all_nodes), len(vehicles_list),
                                           [v.start_node_idx for v in vehicles_list],
                                           [v.end_node_idx for v in vehicles_list])
    routing = pywrapcp.RoutingModel(manager)
    print(f"Num nodes for OR-Tools manager: {manager.GetNumberOfNodes()}, Num vehicles: {len(vehicles_list)}")

    try:
        time_dim, cap_dim = add_dimensions(routing, manager, dist_m, time_m,
                                           vehicles_list, all_nodes, clients_list) # Pass original clients_list for TW
        print(f"\nDimensions added successfully: Time ({time_dim}), Capacity ({cap_dim})")

        # add_pickups_deliveries call remains commented out
        # print("\nSkipped add_pickups_deliveries for now (potential segfault source).")

        def arc_cost_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(dist_m[from_node][to_node]) # Using distance as cost for this example

        transit_callback_index = routing.RegisterTransitCallback(arc_cost_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        print("Arc costs set based on distance matrix.")
        print(f"Routing model status before search: {routing.status()}")

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.time_limit.FromSeconds(TIME_LIMIT_SEC)

        print("[Main] About to call routing.SolveWithParameters()...")
        solution = routing.SolveWithParameters(search_parameters)
        print("[Main] routing.SolveWithParameters() call completed.")

        solution_df_placeholder, kpi_dict_placeholder = None, {} # In case no solution

        if solution:
            print("\n--- Basic Solution Output ---")
            try:
                print(f"Objective: {solution.ObjectiveValue()}")
                for vehicle_s_idx in range(len(vehicles_list)):
                    print(f"Route for vehicle {vehicles_list[vehicle_s_idx].id} (OR-Tools veh index: {vehicle_s_idx}):")
                    index = routing.Start(vehicle_s_idx)
                    route_output = []
                    while not routing.IsEnd(index):
                        node_physical_idx = manager.IndexToNode(index)
                        node_obj = all_nodes[node_physical_idx]
                        arrival_time = solution.Min(time_dim.CumulVar(index))
                        route_output.append(f"NodeID:{node_obj.id}({node_physical_idx})@Arr:{arrival_time}")
                        index = solution.Value(routing.NextVar(index))

                    end_node_physical_idx = manager.IndexToNode(routing.End(vehicle_s_idx))
                    end_node_obj = all_nodes[end_node_physical_idx]
                    end_arrival_time = solution.Min(time_dim.CumulVar(routing.End(vehicle_s_idx)))
                    route_output.append(f"NodeID:{end_node_obj.id}({end_node_physical_idx})@Arr:{end_arrival_time} (End)")
                    print(" -> ".join(route_output))
            except Exception as e:
                print(f"Error during basic solution printing: {e}")
            print("--- End Basic Solution Output ---\n")
            # Placeholder for where full solution extraction would fill these
            # For now, write_outputs will get minimal info or rely on globals if it were more complex
            kpi_dict_placeholder = {"objective_value": solution.ObjectiveValue()}

        else:
            print("No solution found.")
            kpi_dict_placeholder = {"objective_value": -1, "status": "NoSolution"}

        write_outputs(args, solution_df_placeholder, kpi_dict_placeholder) # Call placeholder

    except Exception as e:
        print(f"\nError during OR-Tools setup or solving: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
