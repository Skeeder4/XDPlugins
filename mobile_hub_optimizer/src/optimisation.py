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
TIME_LIMIT_SEC = 30
# DELTA_SYNC_MIN: Defined for planned transfer synchronization, but not currently used as transfer logic is disabled.
DELTA_SYNC_MIN = 15

# --- Data Classes ---
class Client:
    def __init__(self, id, lat, lon, demand, ready, due, service, node_idx=-1):
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.demand = int(demand)
        self.ready = int(ready)
        self.due = int(due)
        self.service = int(service)
        self.node_idx = node_idx # Its index in the global 'all_nodes' list

class Vehicle:
    def __init__(self, id, start_lat, start_lon, cap, start_time, end_time, start_node_idx=-1, end_node_idx=-1):
        self.id = str(id)
        self.start_lat = float(start_lat)
        self.start_lon = float(start_lon)
        self.cap = int(cap)
        self.start_time = int(start_time)
        self.end_time = int(end_time)
        self.start_node_idx = start_node_idx # Actual start node index in all_nodes
        self.end_node_idx = end_node_idx   # Actual end node index in all_nodes

class Transfer:
    def __init__(self, id, lat, lon, w_start, w_end, node_idx=-1):
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.w_start = int(w_start)
        self.w_end = int(w_end)
        self.demand = 0
        self.service = 0
        self.node_idx = node_idx
        self.ready = int(w_start)
        self.due = int(w_end)

class Depot:
    def __init__(self, id, lat, lon, node_idx=-1):
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.demand = 0
        self.service = 0
        self.ready = 0
        self.due = 24 * 60
        self.node_idx = node_idx


def load_and_prepare_data(clients_fp, vehicles_fp, transfers_fp):
    clients_list = []
    vehicles_list = []
    transfers_list = []
    try:
        with open(clients_fp, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader: clients_list.append(Client(**row))
        with open(vehicles_fp, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader: vehicles_list.append(Vehicle(**row))
        with open(transfers_fp, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader: transfers_list.append(Transfer(**row))
    except FileNotFoundError as e:
        print(f"Error: File not found - {e.filename}"); return None, None, None, None, None, None
    except Exception as e:
        print(f"Error loading CSV data: {e}"); return None, None, None, None, None, None

    all_nodes_list_with_nodeidx = []
    node_idx_counter = 0
    for c in clients_list:
        c.node_idx = node_idx_counter
        all_nodes_list_with_nodeidx.append(c)
        node_idx_counter += 1
    for t in transfers_list:
        t.node_idx = node_idx_counter
        all_nodes_list_with_nodeidx.append(t)
        node_idx_counter += 1

    depot_locations = {}
    vehicle_starts_phys_indices = []
    vehicle_ends_phys_indices = []

    for v in vehicles_list:
        start_loc_key = (v.start_lat, v.start_lon)
        assigned_start_node_idx = -1
        for existing_node in all_nodes_list_with_nodeidx[:len(clients_list) + len(transfers_list)]:
            if math.isclose(existing_node.lat, v.start_lat) and \
               math.isclose(existing_node.lon, v.start_lon):
                assigned_start_node_idx = existing_node.node_idx
                break

        if assigned_start_node_idx != -1:
            v.start_node_idx = assigned_start_node_idx
        else:
            if start_loc_key not in depot_locations:
                depot_id = f"Depot_S{len(depot_locations)+1}"
                new_depot = Depot(id=depot_id, lat=v.start_lat, lon=v.start_lon, node_idx=node_idx_counter)
                all_nodes_list_with_nodeidx.append(new_depot)
                depot_locations[start_loc_key] = node_idx_counter
                v.start_node_idx = node_idx_counter
                node_idx_counter += 1
            else:
                v.start_node_idx = depot_locations[start_loc_key]
        v.end_node_idx = v.start_node_idx

        vehicle_starts_phys_indices.append(v.start_node_idx)
        vehicle_ends_phys_indices.append(v.end_node_idx)

    return clients_list, vehicles_list, transfers_list, all_nodes_list_with_nodeidx, vehicle_starts_phys_indices, vehicle_ends_phys_indices


def build_matrices(nodes):
    n = len(nodes)
    dist_matrix = np.zeros((n, n), dtype=float)
    time_matrix = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j in range(n):
            if i == j: continue
            node1, node2 = nodes[i], nodes[j]
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
    def time_evaluator(from_index, to_index):
        from_node_idx = manager.IndexToNode(from_index)
        to_node_idx = manager.IndexToNode(to_index)
        service_time = 0
        if 0 <= from_node_idx < len(nodes):
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
        node_obj = nodes[from_node_idx]
        return node_obj.demand
    demand_evaluator_index = routing.RegisterUnaryTransitCallback(demand_evaluator)
    vehicle_capacities = [v.cap for v in vehicles]
    routing.AddDimensionWithVehicleCapacity(demand_evaluator_index, 0, vehicle_capacities,
                                           True, "Capacity")
    capacity_dimension = routing.GetDimensionOrDie("Capacity")

    for v_idx, vehicle in enumerate(vehicles):
        start_idx, end_idx = routing.Start(v_idx), routing.End(v_idx)
        time_dimension.CumulVar(start_idx).SetRange(vehicle.start_time, horizon)
        time_dimension.CumulVar(end_idx).SetRange(0, vehicle.end_time)

    for node_with_tw in clients_subset_for_tw:
        if node_with_tw.node_idx == -1: continue
        if node_with_tw.node_idx < manager.GetNumberOfNodes():
             model_idx_for_node = manager.NodeToIndex(node_with_tw.node_idx)
             time_dimension.CumulVar(model_idx_for_node).SetRange(node_with_tw.ready, node_with_tw.due)

    return time_dimension, capacity_dimension

def add_pickups_deliveries(routing, manager, time_dim, clients_data, transfers_data, nodes):
    print("Executing add_pickups_deliveries (currently as a passthrough due to instability with AddPickupAndDelivery constraints)...")
    pass

def make_search_parameters(time_limit_seconds):
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.FromSeconds(time_limit_seconds)
    return search_parameters

def get_solution_details_for_testing(solution, routing, manager, time_dim, cap_dim,
                                     vehicles_list_for_model, all_nodes_list_for_manager):
    """
    Extracts structured solution details for testing purposes.
    This function is NOT called by the standard main() execution.
    """
    if not solution:
        return None, {} # (objective_value, routes_data dict)

    objective_value = solution.ObjectiveValue()
    routes_data = {} # vehicle_id -> list of (node_id_str, phys_idx, arrival_time, load)

    for v_idx in range(routing.vehicles()):
        if v_idx >= len(vehicles_list_for_model): # Should not happen if data is consistent
            print(f"Warning: vehicle index {v_idx} out of bounds for vehicles_list_for_model.")
            continue

        vehicle_obj = vehicles_list_for_model[v_idx]
        vehicle_id_str = vehicle_obj.id
        current_vehicle_route_tuples = []

        model_idx = routing.Start(v_idx)
        while not routing.IsEnd(model_idx):
            node_physical_idx = manager.IndexToNode(model_idx)

            if node_physical_idx < 0 or node_physical_idx >= len(all_nodes_list_for_manager):
                print(f"Warning: node_physical_idx {node_physical_idx} out of bounds for all_nodes_list_for_manager.")
                # Potentially add a placeholder or skip this node if it's an internal/unknown index
                model_idx = solution.Value(routing.NextVar(model_idx))
                continue

            node_obj = all_nodes_list_for_manager[node_physical_idx]
            node_id_str = node_obj.id

            arrival_time = int(solution.Min(time_dim.CumulVar(model_idx)))
            load_at_node = int(solution.Value(cap_dim.CumulVar(model_idx)))
            current_vehicle_route_tuples.append(
                (node_id_str, node_physical_idx, arrival_time, load_at_node)
            )
            model_idx = solution.Value(routing.NextVar(model_idx))

        # Append details for the end node of the route
        end_model_idx = routing.End(v_idx)
        end_node_physical_idx = manager.IndexToNode(end_model_idx)

        if 0 <= end_node_physical_idx < len(all_nodes_list_for_manager):
            end_node_obj = all_nodes_list_for_manager[end_node_physical_idx]
            end_node_id_str = end_node_obj.id
            end_arrival_time = int(solution.Min(time_dim.CumulVar(end_model_idx)))
            end_load = int(solution.Value(cap_dim.CumulVar(end_model_idx)))
            current_vehicle_route_tuples.append(
                (end_node_id_str, end_node_physical_idx, end_arrival_time, end_load)
            )
        else:
            print(f"Warning: end_node_physical_idx {end_node_physical_idx} out of bounds for all_nodes_list_for_manager.")

        routes_data[vehicle_id_str] = current_vehicle_route_tuples

    return objective_value, routes_data

def run_optimization_logic(all_nodes_list, vehicles_list,
                           vehicle_starts_phys_indices, vehicle_ends_phys_indices,
                           clients_list, transfers_list,
                           const_time_limit_sec):

    dist_m, time_m = build_matrices(all_nodes_list)

    print("\n--- OR-Tools Setup (inside run_optimization_logic) ---")
    print(f"Total nodes for routing model: {len(all_nodes_list)}")

    manager = pywrapcp.RoutingIndexManager(len(all_nodes_list), len(vehicles_list),
                                           vehicle_starts_phys_indices,
                                           vehicle_ends_phys_indices)
    routing = pywrapcp.RoutingModel(manager)
    print(f"Num nodes for OR-Tools manager: {manager.GetNumberOfNodes()}, Num vehicles: {len(vehicles_list)}")

    time_dim, cap_dim = add_dimensions(routing, manager, dist_m, time_m,
                                       vehicles_list, all_nodes_list, clients_list)
    print(f"\nDimensions added successfully: Time ({time_dim}), Capacity ({cap_dim})")

    add_pickups_deliveries(routing, manager, time_dim, clients_list, transfers_list, all_nodes_list)

    def arc_cost_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index); to_node = manager.IndexToNode(to_index)
        return int(dist_m[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(arc_cost_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    print("Arc costs set based on distance matrix.")
    print(f"Routing model status before search: {routing.status()}")

    search_parameters = make_search_parameters(const_time_limit_sec)

    print("[run_optimization_logic] About to call routing.SolveWithParameters()...")
    solution = routing.SolveWithParameters(search_parameters)
    print("[run_optimization_logic] routing.SolveWithParameters() call completed.")

    # Pass through all necessary items for main's printing logic
    return solution, routing, manager, time_dim, cap_dim, all_nodes_list, vehicles_list

def write_outputs(args_out_path, solution_found, objective_value):
    out_dir = Path(args_out_path)
    print(f"\n[write_outputs] Output directory: {out_dir}")
    placeholder_kpi_path = out_dir / "kpi.txt"
    with open(placeholder_kpi_path, "w", encoding='utf-8') as f:
        f.write("KPI data generation is limited due to instability in detailed solution extraction.\n")
        if solution_found: f.write(f"Objective value from solver: {objective_value}\n")
        else: f.write("No solution was found by the solver.\n")
    placeholder_solution_path = out_dir / "solution_routes.csv"
    with open(placeholder_solution_path, "w", encoding='utf-8') as f:
        f.write("Detailed solution routes (CSV) are not generated due to instability in solution extraction.\n")
        f.write("Basic route information for each vehicle (if solution found) is printed to the console.\n")
    print(f"Placeholder KPI and solution files written to {out_dir} with explanation notes.")

def main():
    parser = argparse.ArgumentParser(description="Mobile Hub Delivery Optimizer")
    parser.add_argument("--clients", required=True, help="Path to clients CSV file")
    parser.add_argument("--vehicles", required=True, help="Path to vehicles CSV file")
    parser.add_argument("--transfers", required=True, help="Path to transfers CSV file")
    parser.add_argument("--out", default="mobile_hub_optimizer/output", help="Output directory path")
    args = parser.parse_args()

    Path(args.out).mkdir(parents=True, exist_ok=True)

    clients_data, vehicles_data, transfers_data, \
    all_nodes_data, vehicle_starts_idx, vehicle_ends_idx = \
        load_and_prepare_data(args.clients, args.vehicles, args.transfers)

    if clients_data is None:
        print("Exiting due to data loading failure.")
        return

    try:
        solution, routing_model, manager_obj, time_dim_obj, cap_dim_obj, \
        all_nodes_for_print, vehicles_for_print = \
            run_optimization_logic(
                all_nodes_data, vehicles_data,
                vehicle_starts_idx, vehicle_ends_idx,
                clients_data, transfers_data,
                TIME_LIMIT_SEC
            )

        objective_value_from_solver = -1
        solution_was_found = False

        if solution:
            solution_was_found = True
            objective_value_from_solver = solution.ObjectiveValue()
            print("\n--- Basic Solution Output (from main) ---")
            print(f"Objective: {objective_value_from_solver}")
            for vehicle_s_idx in range(len(vehicles_for_print)):
                route_output = []
                print(f"Route for vehicle {vehicles_for_print[vehicle_s_idx].id} (OR-Tools veh index: {vehicle_s_idx}):")
                index = routing_model.Start(vehicle_s_idx)
                while not routing_model.IsEnd(index):
                    node_physical_idx = manager_obj.IndexToNode(index)
                    node_obj = all_nodes_for_print[node_physical_idx]
                    arrival_time = solution.Min(time_dim_obj.CumulVar(index))
                    load_at_node = solution.Value(cap_dim_obj.CumulVar(index))
                    route_output.append(f"NodeID:{node_obj.id}({node_physical_idx})@Arr:{arrival_time},Load:{load_at_node}")
                    index = solution.Value(routing_model.NextVar(index))
                end_node_physical_idx = manager_obj.IndexToNode(routing_model.End(vehicle_s_idx))
                end_node_obj = all_nodes_for_print[end_node_physical_idx]
                end_arrival_time = solution.Min(time_dim_obj.CumulVar(routing_model.End(vehicle_s_idx)))
                final_load = solution.Value(cap_dim_obj.CumulVar(routing_model.End(vehicle_s_idx)))
                route_output.append(f"NodeID:{end_node_obj.id}({end_node_physical_idx})@Arr:{end_arrival_time},Load:{final_load} (End)")
                print(" -> ".join(route_output))
            print("--- End Basic Solution Output ---\n")
        else:
            print("No solution found.")

        write_outputs(args.out, solution_was_found, objective_value_from_solver)

    except Exception as e:
        print(f"\nError during optimization logic or solution processing: {e}")
        import traceback
        traceback.print_exc()
        write_outputs(args.out, False, -1)

if __name__ == '__main__':
    main()
