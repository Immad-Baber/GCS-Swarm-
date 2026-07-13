# Drone Swarm SITL & Telemetry Server

This project contains a Software-In-The-Loop (SITL) simulation environment and a telemetry server for managing and visualizing a drone swarm. 

## System Requirements & Version Prerequisites

To guarantee compatibility and successfully run this project on any PC, please ensure you use the following software versions:

- **Operating System:** Linux (e.g. Ubuntu 20.04/22.04), WSL (Windows Subsystem for Linux), or Git Bash on Windows.
- **Python:** `Python 3.8.x` or newer (Tested successfully on Python 3.10+). Ensure the `python3` command is available in your PATH.
- **ArduPilot / ArduCopter SITL:** Firmware version **4.3.x or 4.4.x** (or the latest stable `master` branch). By default, the scripts look for the ArduPilot directory at `~/ardupilot`. If installed elsewhere, set the `ARDUPILOT_HOME` environment variable.
- **MAVProxy:** `v1.8.71` (installed automatically via `requirements.txt`).
- **pymavlink:** `v2.4.47` (installed automatically via `requirements.txt`).
- **Quart (Async Web Framework):** `>=0.19.0` (installed automatically via `requirements.txt`).
- **Network:** Unrestricted local UDP ports starting from `14550` for MAVLink communications.

All required Python dependencies and their exact working versions are strictly defined in the `requirements.txt` file.

## Setup & Installation

Before running the simulation or server, set up your Python environment:

1. **Create a virtual environment:**
   ```bash
   python3 -m venv venv
   ```

2. **Activate the virtual environment:**
   - On Linux / WSL: `source venv/bin/activate`
   - On Windows (Git Bash): `source venv/Scripts/activate`

3. **Install the dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

## Running the Project

You will need to run the simulation and the telemetry server in separate terminal windows.

### 1. Start the SITL Simulation

To start the ArduPilot SITL instances, use the `start_sitl.sh` script. By default, it will launch **3 drones**.

```bash
bash start_sitl.sh
```

To run with a different number of drones, pass the desired count as an argument. For example, to start 5 drones:

```bash
bash start_sitl.sh 5
```

### 2. Start the Telemetry Server

Open a new terminal window, ensure your virtual environment is active, and run:

```bash
bash run_telemetry_server.sh
```

## Running Test Cases

Before running any test scripts, make sure the **SITL simulation is already running**.

Activate your virtual environment and execute the desired test case using Python. For example:

- **Movement Test:**
  ```bash
  python test_move.py
  ```

- **Landing Test:**
  ```bash
  python test_land.py
  ```

**Other available tests include:**
- `python test_move2.py`
- `python test_move3.py`
- `python test_drone3.py`
- `python test_drone3_mode.py`

## Cleanup

To reset the environment and clean up generated log files and instance directories, simply delete the following folders/files from the project root:

- `venv/`
- `final_venv/`
- `instance_*/`
- `logs/`
- `terrain/`
- `sitl_instance_*.log`
- `/tmp/sitl_instance_*.log`
