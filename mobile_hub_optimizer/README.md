# Mobile Hub Delivery Optimizer

This script provides a prototype for optimizing delivery routes where packages can be transferred between delivery agents at designated mobile hub points. It uses Google OR-Tools to find routes that minimize total travel distance while respecting constraints such as vehicle capacities, client time windows, and synchronization at transfer hubs (though advanced transfer logic is currently limited).

## Project Status

**Note:** This is a prototype. The core routing model is set up, but advanced features like detailed solution output and complex transfer logic (package exchanges and synchronization) are currently simplified or pending full implementation due to technical issues encountered during development (segmentation faults with solution extraction and instability with certain P&D constraints in the test environment).

## Setup and Installation

1.  **Python:** Ensure you have Python 3.8+ installed.
2.  **Dependencies:** Install the required Python libraries. It's recommended to use a virtual environment.
    ```bash
    pip install numpy ortools pandas
    ```
    *(Note: Pandas is listed as it was part of the original plan for data handling, though currently not used in the simplified output.)*

## Running the Script

The script is run from the command line, providing paths to the input CSV files.

```bash
python mobile_hub_optimizer/src/optimisation.py --clients mobile_hub_optimizer/data/clients.csv --vehicles mobile_hub_optimizer/data/vehicles.csv --transfers mobile_hub_optimizer/data/transfers.csv --out mobile_hub_optimizer/output
```

### Command-Line Arguments:

*   `--clients`: Path to the CSV file containing client data. (Required)
*   `--vehicles`: Path to the CSV file containing vehicle data. (Required)
*   `--transfers`: Path to the CSV file containing transfer hub data. (Required)
*   `--out`: Path to the directory where output files will be saved (e.g., `solution.csv`, `kpi.txt`). Defaults to `mobile_hub_optimizer/output`. The script will attempt to create this directory if it doesn't exist.

## Input File Formats

All input files must be in CSV format.

### 1. Clients CSV (`--clients`)

*   **Columns:** `id,lat,lon,demand,ready,due,service`
*   `id`: Unique identifier for the client (string).
*   `lat`: Latitude (float).
*   `lon`: Longitude (float).
*   `demand`: Quantity of goods to be delivered (integer).
*   `ready`: Earliest time for service (integer, minutes from midnight).
*   `due`: Latest time for service (integer, minutes from midnight).
*   `service`: Time required for service at the client's location (integer, minutes).

**Example (`clients.csv`):**
```csv
id,lat,lon,demand,ready,due,service
C1,10.0,10.0,10,480,600,10
C2,10.1,10.1,5,540,660,5
...
```

### 2. Vehicles CSV (`--vehicles`)

*   **Columns:** `id,start_lat,start_lon,cap,start_time,end_time`
*   `id`: Unique identifier for the vehicle (string).
*   `start_lat`: Starting latitude of the vehicle (float).
*   `start_lon`: Starting longitude of the vehicle (float).
*   `cap`: Carrying capacity of the vehicle (integer).
*   `start_time`: Earliest time the vehicle can start its route (integer, minutes from midnight).
*   `end_time`: Latest time the vehicle must finish its route (integer, minutes from midnight).

**Example (`vehicles.csv`):**
```csv
id,start_lat,start_lon,cap,start_time,end_time
V1,10.0,10.0,25,420,1080
V2,10.0,10.0,20,420,1080
...
```

### 3. Transfers CSV (`--transfers`)

*   **Columns:** `id,lat,lon,w_start,w_end`
*   `id`: Unique identifier for the transfer hub (string).
*   `lat`: Latitude of the hub (float).
*   `lon`: Longitude of the hub (float).
*   `w_start`: Earliest time the hub is available (integer, minutes from midnight).
*   `w_end`: Latest time the hub is available (integer, minutes from midnight).

**Example (`transfers.csv`):**
```csv
id,lat,lon,w_start,w_end
H1,10.05,10.05,420,1080
H2,10.1,10.0,420,1080
...
```

## Output

Currently, the script provides basic output to the console if a solution is found:
*   The objective value of the solution.
*   A textual representation of the route for the first vehicle (node indices and arrival times).

The `--out` argument specifies a directory where more detailed output files (`solution.csv`, `kpi.txt`) would be written by the `write_outputs` function. However, due to issues with detailed solution extraction, the `write_outputs` function is not fully populated or tested.

## Current Limitations & Known Issues

*   **Solution Output:** Detailed solution extraction (into DataFrames and KPIs) is currently bypassed due to segmentation faults encountered when trying to pass OR-Tools solution objects to helper functions in the Python environment. Basic output is printed directly to the console.
*   **Advanced Transfer Logic:** The `add_pickups_deliveries` function, intended to model complex package transfers and synchronization between vehicles at hubs, is currently commented out due to stability issues (segmentation faults) in the test environment. This means the current model primarily solves a Capacitated Vehicle Routing Problem with Time Windows (CVRPTW), and the "mobile hub transfer" aspect is not fully operational.
*   **Error Handling:** Error handling for invalid inputs or unsolvable scenarios can be improved.

This prototype serves as a foundational step. Further development would be needed to address the stability issues and fully implement the intended mobile hub transfer functionalities.
