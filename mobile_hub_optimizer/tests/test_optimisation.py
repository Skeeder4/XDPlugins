import pytest
from pathlib import Path
import sys

# Add the src directory to sys.path to allow importing optimisation module
SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

try:
    from optimisation import (
        load_and_prepare_data,
        run_optimization_logic,
        get_solution_details_for_testing,
        Client, Vehicle, Transfer, Depot
    )
except ImportError as e:
    print(f"Error importing from optimisation: {e}")
    print(f"Ensure optimisation.py is in {SRC_DIR} and sys.path is correct: {sys.path}")
    raise

TEST_DIR = Path(__file__).resolve().parent
BASE_PROJECT_DIR = TEST_DIR.parent
DATA_DIR = BASE_PROJECT_DIR / "data"

DEFAULT_CLIENT_CSV = DATA_DIR / "clients.csv"
DEFAULT_VEHICLE_CSV = DATA_DIR / "vehicles.csv"
DEFAULT_TRANSFER_CSV = DATA_DIR / "transfers.csv"
DEFAULT_TIME_LIMIT_SEC = 10 # Reduced time limit for tests

# Expected objective for the default adjusted dataset (from Turn 25/31/33)
EXPECTED_OBJECTIVE_ADJUSTED_DATA = 102

def run_optimization_for_testing(client_csv_path_str=str(DEFAULT_CLIENT_CSV),
                               vehicle_csv_path_str=str(DEFAULT_VEHICLE_CSV),
                               transfer_csv_path_str=str(DEFAULT_TRANSFER_CSV),
                               time_limit_sec=DEFAULT_TIME_LIMIT_SEC):
    """
    Helper function to load data, run optimization, and extract solution details for testing.
    Returns: (objective, routes_data, clients_list, vehicles_list, all_nodes_list, manager, routing, time_dim, cap_dim, solution_obj)
    'objective' will be None and 'routes_data' empty if no solution.
    Also returns manager, routing, dimensions, and solution_obj for further specific checks if needed.
    """
    client_path = Path(client_csv_path_str)
    vehicle_path = Path(vehicle_csv_path_str)
    transfer_path = Path(transfer_csv_path_str)

    print(f"\nRunning optimization for test with data: C:{client_path.name}, V:{vehicle_path.name}, T:{transfer_path.name}")

    (clients_list, vehicles_list, transfers_list,
     all_nodes_list_with_nodeidx,
     vehicle_starts_phys_indices, vehicle_ends_phys_indices) = load_and_prepare_data(
        client_path, vehicle_path, transfer_path
    )

    if clients_list is None:
        print("Test data loading failed. Aborting test run.")
        return None, {}, [], [], [], None, None, None, None, None


    solution_obj, routing_model, manager_obj, time_dim_obj, cap_dim_obj, _, _ = run_optimization_logic(
        all_nodes_list_with_nodeidx,
        vehicles_list,
        vehicle_starts_phys_indices,
        vehicle_ends_phys_indices,
        clients_list,
        transfers_list,
        time_limit_sec
    )

    if solution_obj:
        objective, routes = get_solution_details_for_testing(
            solution_obj, routing_model, manager_obj, time_dim_obj, cap_dim_obj,
            vehicles_list,
            all_nodes_list_with_nodeidx
        )
        return (objective, routes, clients_list, vehicles_list, all_nodes_list_with_nodeidx,
                manager_obj, routing_model, time_dim_obj, cap_dim_obj, solution_obj)
    else:
        return (None, {}, clients_list, vehicles_list, all_nodes_list_with_nodeidx,
                manager_obj, routing_model, time_dim_obj, cap_dim_obj, solution_obj)

@pytest.fixture(scope="module") # Run once per module for efficiency
def solution_data_fixture():
    print("\n--- Setting up solution_data_fixture (running optimization once) ---")
    data_tuple = run_optimization_for_testing()
    objective = data_tuple[0] # First element is objective
    assert objective is not None, "No solution found for test data, cannot proceed with detailed tests."
    # For now, allow objective to vary slightly if other tests are the focus
    # assert objective == EXPECTED_OBJECTIVE_ADJUSTED_DATA, f"Objective value {objective} did not match expected {EXPECTED_OBJECTIVE_ADJUSTED_DATA}"
    print(f"--- Fixture: Optimization run complete. Objective: {objective} ---")
    return data_tuple


def test_helper_can_run_and_load_data():
    print(f"Checking data file paths exist: C:{DEFAULT_CLIENT_CSV.exists()}, V:{DEFAULT_VEHICLE_CSV.exists()}, T:{DEFAULT_TRANSFER_CSV.exists()}")
    assert DEFAULT_CLIENT_CSV.exists(), f"Default client CSV not found for test setup: {DEFAULT_CLIENT_CSV}"
    assert DEFAULT_VEHICLE_CSV.exists(), f"Default vehicle CSV not found for test setup: {DEFAULT_VEHICLE_CSV}"
    assert DEFAULT_TRANSFER_CSV.exists(), f"Default transfer CSV not found for test setup: {DEFAULT_TRANSFER_CSV}"

    (objective, routes, clients, vehicles, all_nodes,
     manager, routing, time_dim, cap_dim, solution) = run_optimization_for_testing()

    print(f"Test (test_helper_can_run_and_load_data) helper run completed. Objective: {objective}")

    assert clients is not None and len(clients) > 0, "Client data should be loaded"
    assert vehicles is not None and len(vehicles) > 0, "Vehicle data should be loaded"
    assert all_nodes is not None and len(all_nodes) > 0, "All_nodes list should be populated"
    # Not asserting objective value here, that's for a dedicated test.

def test_solve_example_data_finds_solution_and_matches_objective(solution_data_fixture):
    objective, _, _, _, _, _, _, _, _, _ = solution_data_fixture
    # Assertion for objective is already in the fixture, but can be more specific here if needed
    assert objective is not None, "Solver failed to find a solution for the example data (checked in test)."
    assert objective == EXPECTED_OBJECTIVE_ADJUSTED_DATA, \
           f"Objective {objective} != expected {EXPECTED_OBJECTIVE_ADJUSTED_DATA}"

def test_vehicle_capacity_respected(solution_data_fixture):
    objective, routes, _, vehicles_list, _, _, _, _, _, _ = solution_data_fixture

    vehicle_map = {v.id: v for v in vehicles_list}
    for vehicle_id_str, route_legs in routes.items():
        vehicle_obj = vehicle_map.get(vehicle_id_str)
        assert vehicle_obj is not None, f"Vehicle ID {vehicle_id_str} from routes not found in input vehicles list."
        for leg_node_id_str, _, _, load_at_node in route_legs: # leg is (node_id_str, physical_idx, arrival_time, load_at_node)
            assert 0 <= load_at_node <= vehicle_obj.cap, \
                   f"Vehicle {vehicle_id_str} exceeded capacity at node {leg_node_id_str}. Load: {load_at_node}, Cap: {vehicle_obj.cap}"

def test_client_time_windows(solution_data_fixture):
    objective, routes, clients_list, _, all_nodes_list, _, _, _, _, _ = solution_data_fixture

    client_map_by_id = {c.id: c for c in clients_list}

    for vehicle_id_str, route_legs in routes.items():
        for leg_node_id_str, leg_phys_idx, leg_arrival_time, _ in route_legs:
            if leg_node_id_str in client_map_by_id:
                client_obj = client_map_by_id[leg_node_id_str]
                # Service time is accounted for by the time dimension a_i + s_i + t_ij = a_j
                # So arrival time should be within [ready, due]
                # Departure time (arrival + service) should be <= due.
                # The get_solution_details_for_testing returns arrival_time.
                # We also need service time to check full window.
                # For simplicity, check arrival against ready and due. A more precise check would be arrival+service <= due.
                assert client_obj.ready <= leg_arrival_time, \
                       f"Client {client_obj.id} arrival {leg_arrival_time} before ready time {client_obj.ready}"
                # The arrival at a node must be such that service can be completed by due time.
                # OR-Tools time window constraint on CumulVar(node) applies to arrival time at node.
                # The service time is added to travel time on arc FROM this node.
                # So CumulVar(node) is arrival_time. CumulVar(node) + service_time is departure_time.
                # Time Dim: TW_node_ready <= CumulVar(node) <= TW_node_due
                # This means arrival must be within TW. If service happens, departure is CumulVar(node) + service_time.
                # This departure time isn't directly constrained by TW_node_due via AddDimension.
                # It's implicitly handled if the next node's TW is met.
                # For this test, let's assume client_obj.due is the latest an *arrival* can happen.
                # This is consistent with how OR-Tools time windows are typically set on CumulVar.
                assert leg_arrival_time <= client_obj.due, \
                       f"Client {client_obj.id} arrival {leg_arrival_time} after due time {client_obj.due}"


def test_vehicle_operating_times(solution_data_fixture):
    objective, routes, _, vehicles_list, _, _, _, _, _, _ = solution_data_fixture
    vehicle_map = {v.id: v for v in vehicles_list}

    for vehicle_id_str, route_legs in routes.items():
        vehicle_obj = vehicle_map.get(vehicle_id_str)
        assert vehicle_obj is not None

        if not route_legs: continue

        # route_legs[0] is the start depot of the vehicle. Its arrival time is its start time from depot.
        first_leg_arrival_time = route_legs[0][2]
        last_leg_arrival_time = route_legs[-1][2]

        assert first_leg_arrival_time >= vehicle_obj.start_time, \
               f"Vehicle {vehicle_id_str} first node arrival {first_leg_arrival_time} before vehicle's overall start_time {vehicle_obj.start_time}"
        assert last_leg_arrival_time <= vehicle_obj.end_time, \
               f"Vehicle {vehicle_id_str} last node arrival {last_leg_arrival_time} after vehicle's overall end_time {vehicle_obj.end_time}"

def test_all_clients_served(solution_data_fixture):
    objective, routes, clients_list, _, _, _, _, _, _, _ = solution_data_fixture

    input_client_ids = {c.id for c in clients_list}
    serviced_client_ids = set()
    for vehicle_id_str, route_legs in routes.items():
        for leg_node_id_str, _, _, _ in route_legs:
            if leg_node_id_str in input_client_ids:
                serviced_client_ids.add(leg_node_id_str)

    assert input_client_ids == serviced_client_ids, \
           f"Not all clients were serviced. Missing: {input_client_ids - serviced_client_ids}"

def test_depot_start_end(solution_data_fixture):
    objective, routes, _, vehicles_list, all_nodes_list, _, _, _, _, _ = solution_data_fixture

    vehicle_map = {v.id: v for v in vehicles_list}
    # all_nodes_list from run_optimization_for_testing is all_nodes_list_with_nodeidx from load_and_prepare_data
    # Its elements have .node_idx which is their physical index.
    all_nodes_map_by_phys_idx = {node.node_idx: node for node in all_nodes_list}

    for v_id_str, route_legs in routes.items():
        vehicle_obj = vehicle_map[v_id_str]
        assert route_legs, f"Route for vehicle {v_id_str} is empty."

        expected_start_node_obj = all_nodes_map_by_phys_idx[vehicle_obj.start_node_idx]
        expected_end_node_obj = all_nodes_map_by_phys_idx[vehicle_obj.end_node_idx]

        route_start_node_phys_idx = route_legs[0][1]
        route_end_node_phys_idx = route_legs[-1][1]

        assert route_start_node_phys_idx == vehicle_obj.start_node_idx, \
               f"Vehicle {v_id_str} route started at phys_idx {route_start_node_phys_idx} (ID: {route_legs[0][0]}), expected phys_idx {vehicle_obj.start_node_idx} (ID: {expected_start_node_obj.id})"
        assert route_end_node_phys_idx == vehicle_obj.end_node_idx, \
               f"Vehicle {v_id_str} route ended at phys_idx {route_end_node_phys_idx} (ID: {route_legs[-1][0]}), expected phys_idx {vehicle_obj.end_node_idx} (ID: {expected_end_node_obj.id})"

# Placeholder for more specific tests if P&D or hub logic were active
# def test_transfer_logic():
#     assert False, "Transfer logic tests not implemented as feature is disabled."
