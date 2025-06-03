# Mobile Hub Delivery Optimizer

This script was intended as a prototype for optimizing delivery routes featuring package transfers between vehicles at mobile hubs. It uses Google OR-Tools for route calculation.

## Final Project Outcome & Limitations

**Delivered Functionality: CVRPTW Solver**

The script in its current stable state is a functional prototype of a **Capacitated Vehicle Routing Problem with Time Windows (CVRPTW)** solver.
It successfully achieves the following:
*   Loads client, vehicle, and transfer hub data from user-specified CSV files.
*   Calculates distance and time matrices between all loaded nodes using the Haversine formula.
*   Models multiple vehicles, each with individual capacity, defined start/end operational times, and specific start/end depot locations (which can be co-located with client or hub sites, or be distinct).
*   Enforces client time windows (ready and due times) and service times at client locations.
*   Utilizes Google OR-Tools to find vehicle routes that aim to minimize total travel distance while respecting all aforementioned constraints.
*   If a solution is found:
    *   Prints basic solution output to the console, including the overall objective value (total distance) and routes for each vehicle (sequence of visited node IDs, physical indices, arrival times, and vehicle load at each stop).
    *   Creates placeholder output files (`kpi.txt`, `solution_routes.csv`) in a specified output directory. These files contain notes explaining the limited nature of detailed output (see below).

**Advanced Features - Not Implemented Due to Technical Roadblocks:**

The primary and advanced goal of this project—enabling **dynamic, optimized package transfers between different vehicles at mobile hubs**—is **NOT implemented**. This includes the following specific functionalities that could not be realized:
*   **Optimized Multi-Leg Shipments:** Modeling and optimizing scenarios where a package is picked up by one vehicle, dropped off at a mobile hub, and then picked up by a different vehicle for final delivery.
*   **Capacity Adjustments at Hubs:** Dynamically adjusting vehicle capacities at hubs to reflect package drop-offs and pickups during transfers.
*   **Synchronization of Vehicles at Hubs:** Enforcing constraints to make multiple vehicles meet at a hub within a specified time window (`DELTA_SYNC_MIN`) to perform transfers.
*   **Vehicle Replenishment:** Allowing vehicles to return to a depot to reload and continue servicing further clients.

**Reason for Non-Implementation of Advanced Features:**
The attempts to implement the core transfer logic using various configurations of OR-Tools' `AddPickupAndDelivery` constraints (for modeling client-specific services, package movements to/from hubs, and linking shipment legs) and other model restructuring techniques (like duplicated hub nodes for explicit transfer modeling, or custom demand callbacks for capacity tracking during transfers) consistently resulted in **unrecoverable segmentation faults originating from the OR-Tools library within the provided testing environment.**

These were not Python-level errors but low-level crashes in the C++ backend of OR-Tools, occurring when the solver was invoked or, in some cases, even when functions handling these OR-Tools objects were merely called. Despite numerous focused experiments to isolate and work around these instabilities (as documented in the development history), a stable method for implementing the required P&D or advanced synchronization constraints for transfers could not be achieved.

**Impact of Non-Implementation:**
*   The `add_pickups_deliveries` function, originally intended for the detailed transfer logic, currently acts as a **passthrough function** (it performs no operations) to maintain script stability.
*   Consequently, the script operates as a standard CVRPTW solver. "Transfer hub" locations loaded from CSV are treated like any other potential waypoint; vehicles may pass through them if it's optimal for their individual routes according to the CVRPTW objective, but **no package exchanges or inter-vehicle coordination for transfers occur at these hubs.**
*   The `DELTA_SYNC_MIN` constant, defined for hub synchronization, is not used.

**Solution Output Limitation:**
*   Detailed solution output to structured CSV/text files (e.g., `solution_routes.csv` for routes, `kpi.txt` for key performance indicators) is **minimal**.
*   **Reason:** Similar to the P&D issues, attempts to pass the OR-Tools `solution` object to a dedicated Python function for detailed data extraction and formatting also led to segmentation faults.
*   **Current State:** The script creates placeholder files with notes explaining this limitation. Essential solution information (objective value and basic routes with arrival times and load) is printed directly to the console from the main execution block, which was found to be a more stable method of accessing basic solution data.

**Error Handling:**
*   Error handling for invalid input data formats or edge-case unsolvable scenarios is basic and could be further improved.

**Summary:**
This prototype serves as a foundational CVRPTW solver. To achieve the originally envisioned mobile hub transfer capabilities, significant further investigation and debugging of the OR-Tools interaction within the specific runtime environment, or exploration of alternative modeling techniques and potentially different OR-Tools versions/APIs, would be necessary.

## Setup and Installation

1.  **Python:** Ensure you have Python 3.8+ installed.
2.  **Dependencies:** Install the required Python libraries. It's recommended to use a virtual environment.
    ```bash
    pip install numpy ortools
    ```
    *(Note: Pandas was part of an earlier plan for data output but is not actively used in the current version due to the solution output limitations described above.)*

## Running the Script

The script is run from the command line:
```bash
python mobile_hub_optimizer/src/optimisation.py --clients mobile_hub_optimizer/data/clients.csv --vehicles mobile_hub_optimizer/data/vehicles.csv --transfers mobile_hub_optimizer/data/transfers.csv --out mobile_hub_optimizer/output
```

### Command-Line Arguments:
*   `--clients`: Path to the CSV file for client data. (Required)
*   `--vehicles`: Path to the CSV file for vehicle data. (Required)
*   `--transfers`: Path to the CSV file for transfer hub data. (Required)
*   `--out`: Output directory for placeholder files. Defaults to `mobile_hub_optimizer/output`.

## Input File Formats

### 1. Clients CSV (`--clients`)
*   **Columns:** `id,lat,lon,demand,ready,due,service`
*   **Example (`clients.csv`):**
    ```csv
    id,lat,lon,demand,ready,due,service
    C1,10.0,10.0,10,480,600,10
    ...
    ```

### 2. Vehicles CSV (`--vehicles`)
*   **Columns:** `id,start_lat,start_lon,cap,start_time,end_time`
*   **Example (`vehicles.csv`):**
    ```csv
    id,start_lat,start_lon,cap,start_time,end_time
    V1,10.0,10.0,25,420,1080
    ...
    ```

### 3. Transfers CSV (`--transfers`)
*   **Columns:** `id,lat,lon,w_start,w_end`
*   **Example (`transfers.csv`):**
    ```csv
    id,lat,lon,w_start,w_end
    H1,10.05,10.05,420,1080
    ...
    ```
