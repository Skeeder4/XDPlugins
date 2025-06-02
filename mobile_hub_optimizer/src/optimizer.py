import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def load_data(data_path_prefix=""):
    """Loads customer, driver, and hub data from CSV files."""
    try:
        customers_df = pd.read_csv(f"{data_path_prefix}data/customers.csv")
        drivers_df = pd.read_csv(f"{data_path_prefix}data/drivers.csv")
        hubs_df = pd.read_csv(f"{data_path_prefix}data/hubs.csv") # Loaded but not used yet
        print(f"Customers: {len(customers_df)}, Drivers: {len(drivers_df)}, Hubs: {len(hubs_df)}")
        return customers_df, drivers_df, hubs_df
    except FileNotFoundError as e:
        print(f"Error loading data files: {e}")
        return None, None, None
    except Exception as e:
        print(f"An unexpected error occurred during data loading: {e}")
        return None, None, None


def manhattan_distance(pos1, pos2):
    """Computes the Manhattan distance between two points (x,y tuples)."""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def create_data_model(customers_df, drivers_df):
    """Stores the data for the problem."""
    data = {}

    data["locations"] = [(0, 0)] # Dummy depot at (0,0)
    data["demands"] = [0] # Demand for dummy depot is 0

    for index, row in customers_df.iterrows():
        data["locations"].append((row["x_coord"], row["y_coord"]))
        data["demands"].append(row["quantity"])

    data["num_locations"] = len(data["locations"])
    data["num_vehicles"] = len(drivers_df)
    data["vehicle_capacities"] = drivers_df["capacity"].tolist()
    data["depot"] = 0

    print(f"Data model: {data['num_locations']} locations, {data['num_vehicles']} vehicles.")
    print(f"Demands: {data['demands']}")
    print(f"Vehicle capacities: {data['vehicle_capacities']}")
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    if not solution:
        print("No solution found!")
        return

    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0
    capacity_dimension = routing.GetDimensionOrDie('Capacity')

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            load_var = capacity_dimension.CumulVar(index)
            plan_output += f" {node_index} Load({solution.Value(load_var)}) ->"

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            arc_distance = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            route_distance += arc_distance

        # Add end node
        node_index = manager.IndexToNode(index)
        load_var = capacity_dimension.CumulVar(index)
        plan_output += f" {node_index} Load({solution.Value(load_var)})\n" # Depot, load should be 0 or reflect return

        plan_output += f"Distance of the route: {route_distance}m\n"
        plan_output += f"Load of the route: {route_load}\n" # This is manually calculated sum of demands
        print(plan_output)
        total_distance += route_distance
        total_load += route_load # Sum of demands picked up by vehicles
    print(f"Total Distance of all routes: {total_distance}m")
    print(f"Total Load of all routes: {total_load}")


def main():
    """Entry point of the program."""
    customers_df, drivers_df, _ = load_data(data_path_prefix="mobile_hub_optimizer/")
    if customers_df is None or drivers_df is None :
        print("Failed to load data. Exiting.")
        return

    data = create_data_model(customers_df, drivers_df)

    manager = pywrapcp.RoutingIndexManager(
        data["num_locations"], data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return manhattan_distance(data["locations"][from_node], data["locations"][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # search_parameters.local_search_metaheuristic = (
    #    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    # )
    # search_parameters.time_limit.FromSeconds(10) # Increased time limit slightly

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found for the problem with capacity constraints!")

if __name__ == "__main__":
    main()
