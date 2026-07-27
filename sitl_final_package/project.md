# GCS Swarm Commander: Complete Project Documentation

This document serves as the exhaustive record of the GCS Swarm Commander project. It details the complete evolution from a single-drone simulation into a highly concurrent, fully autonomous multi-drone swarm ecosystem. It encompasses all algorithms, architectures, strategies, and solutions implemented to overcome complex technical challenges.

---

## 1. Project Evolution: From Single to Swarm
The project originally operated under a single-drone paradigm, handling basic MAVLink connections, telemetry collection, and waypoint navigation. To scale this into a multi-drone swarm capable of complex collaborative behaviors, the entire backend architecture had to be overhauled.

**Key Evolutionary Steps:**
*   **Multithreaded MAVLink Handling:** `pymavlink` is natively blocking. Scaling to multiple drones required wrapping each `SITLAdapter` instance in its own dedicated OS thread using Python's `concurrent.futures.ThreadPoolExecutor` and `asyncio.to_thread`. This prevented network bottlenecks where waiting for one drone's telemetry would freeze the entire swarm.
*   **Decoupled Adapter State:** Variables like `flight_path`, `boot_time`, and `abort_mission` were isolated within individual `SITLAdapter` object instances rather than relying on global state.
*   **Swarm Manager Orchestration:** A central `SwarmManager` singleton was created to aggregate all connected drones, manage thread-safe state via `threading.Lock()`, and dispatch concurrent commands (like `takeoff_all` and `arm_all`).

---

## 2. Comprehensive System Architecture

The architecture is divided into three primary layers: Simulation, Backend Control, and Frontend Visualization.

### 2.1 Simulation Layer (ArduCopter SITL)
*   The `start_sitl.sh` script dynamically launches up to 10 instances of ArduCopter firmware in Software-In-The-Loop mode.
*   Each drone is assigned a unique System ID (SYSID) and UDP port (14551, 14552, etc.).
*   Drones are physically spawned in a predefined V-formation on the ground to prevent catastrophic collisions upon takeoff.

### 2.2 Backend Control Layer (Python, Quart, MAVProxy)
*   **`SITLAdapter`:** Manages the low-level MAVLink connection to a specific UDP port. Parses binary telemetry into Python dictionaries.
*   **`SwarmManager`:** The orchestrator. Exposes REST API endpoints for the UI (e.g., `/api/swarm/takeoff_all`) and manages the thread pool executing commands across all `SITLAdapter` instances.
*   **`FormationManager`:** Handles advanced spatial mathematics (leader-follower logic).
*   **Quart Telemetry Server:** An asynchronous web server running at `0.0.0.0:5000`. It receives HTTP POST telemetry from the swarm threads, drops it into an `asyncio.Queue`, and asynchronously broadcasts it via WebSockets.

### 2.3 Frontend GCS Layer (HTML/CSS/JS)
*   A fully custom HTML5 interface relying on Vanilla JS and `Leaflet.js` for mapping.
*   The UI listens to the WebSocket broadcast (`ws://localhost:5000/ws`).
*   Dynamically generates sidebar tabs for new drones as they come online.
*   Renders flight paths, updates telemetry gauges, and rotates drone icons based on their MAVLink heading data.
*   Features an interactive **Obstacle Map** panel to click-to-place virtual hazards in real-time.

---

## 3. Setup and Execution Instructions

### 3.1 Prerequisites
To run this project, you must have the following installed:
*   **Operating System:** Linux (Ubuntu 20.04/22.04), WSL, or Git Bash on Windows.
*   **Python:** Python 3.8+ (Tested on 3.10+).
*   **ArduPilot SITL Firmware:** v4.3.x or 4.4.x.

### 3.2 Installation
1.  **Create a Virtual Environment:**
    ```bash
    python3 -m venv final_venv
    source final_venv/bin/activate
    ```
2.  **Install Dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

### 3.3 Running the Swarm (Step-by-Step)
You will need three separate terminal instances (tabs) to run the full stack:

**Terminal 1: Start the SITL Simulator Instances**
```bash
# By default, this launches 3 drones. You can pass a number up to 10.
bash start_sitl.sh 3
```
*IMPORTANT: Wait 3-4 minutes for all drones to output `EKF3 IMU0 is using GPS` in their logs before proceeding.*

**Terminal 2: Start the Telemetry & Web Server**
```bash
cd mavlink_integration
source ../final_venv/bin/activate
python3 telemetry_server.py
```
*Once running, open `http://localhost:5000` in your web browser to view the Live GCS Dashboard.*

**Terminal 3: Run the Mission Controller**
```bash
cd mavlink_integration
source ../final_venv/bin/activate
python3 main.py --drones 3
```
*The controller will automatically connect to all drones, switch them to GUIDED mode, arm them, and begin the V-formation flight plan.*

---

## 4. Swarm Formation Mechanics

### 4.1 V-Formation & Offsets
The swarm flies in a mathematically precise V-Formation.
*   `drone_1` is designated as the **Leader**.
*   Followers (drones 2-10) are assigned fixed **Body-Frame Offsets** (dx, dy). For example, `(-25m, -10m)`.
*   These exact offsets are hardcoded in both the `start_sitl.sh` spawn logic and the Python `SwarmManager`. Matching these offsets was a critical fix to stop drones from violently diverging from the flight path upon takeoff.

### 4.2 Dynamic Coordinate Shifting & Rotation
*   **Bearing Calculation:** Before flying, the system calculates the angle between the home position and the first waypoint.
*   **Rotation Matrix:** The body-frame offsets (dx, dy) are passed through a 2D trigonometric rotation matrix based on the flight bearing. This guarantees the V-shape always points *forward* in the direction of flight, rather than simply locking facing North.
*   **Leader Grounding:** Followers calculate their final GPS waypoints by measuring the Leader's actual physical GPS location, computing the offset, and shifting the entire JSON mission accordingly.

### 4.3 Dynamic Leader Election
If `drone_1` disconnects or crashes (altitude drops), the `FormationManager` detects the failure and automatically promotes the next available, healthy drone (e.g., `drone_2`) to take over as the absolute reference point for the swarm.

---

## 5. Advanced Obstacle Avoidance (Artificial Potential Fields)

To make the swarm autonomous, we implemented an **Artificial Potential Field (APF)** algorithm. This replaced passive waypoint following with highly reactive, sensorless avoidance.

### 5.1 The APF Algorithm
The environment is treated as a magnetic field. The target waypoint pulls the drone (Attractive Force), and obstacles push it away (Repulsive Force).
`Force = Peak_Force * ((R_influence - Current_Distance) / (R_influence - R_surface))^2`
Using a quadratic falloff ensures the drone gracefully curves around obstacles rather than snapping violently away.

### 5.2 Additive Layering Strategy
Instead of rewriting the `waypoint_navigator.py` logic, the avoidance logic was surgically injected into `drone_controller.py`'s `wait_until_position_reached` loop.
Every 2 seconds, the drone queries the `obstacle_map`. If a repulsive vector exists, it mathematically adds that vector to the target coordinate and re-sends the `DO_REPOSITION` command to ArduPilot. This provides universal avoidance for *all* mission types seamlessly.

### 5.3 Hazard Classifications
1.  **Static Obstacles (Walls/Buildings):** Possess a defined roof height (`max_alt_m`). If the drone flies 15m above this altitude, the APF force is completely ignored, allowing safe fly-overs.
2.  **Dynamic Obstacles (Birds):** Modeled using a fast-ticking (10 Hz) background Python thread that continuously updates their physical GPS location based on velocity vectors. The warning radius is massive (40m) to account for high-speed intercepts.
3.  **Wind Zones / Turbulence:** Modeled as infinite vertical cylinders (altitude filtering is disabled). Features a `strength` multiplier that drastically inflates the repulsive force, forcing wide avoidance sweeps.

### 5.4 Local Minima Handling
If a drone gets trapped in a corner formed by multiple overlapping obstacles, the opposing repulsive vectors perfectly cancel out the attractive forward vector. The system detects this state and forces the drone to hover safely in place. Once the hazard (like a moving bird) passes, the drone instantly resumes the mission.

---

## 6. Summary of Major Challenges & Solutions

| Challenge | Solution Implemented |
| :--- | :--- |
| **Blocking Network I/O freezing the Swarm** | Transitioned from single-threaded procedural execution to a highly concurrent `ThreadPoolExecutor` and `asyncio` architecture, assigning one thread per drone. |
| **Drones crashing into unseen environments** | Engineered the pure-Python Artificial Potential Field (APF) layer, acting as a virtual companion computer to alter MAVLink targets dynamically. |
| **Erratic diverging upon takeoff** | Synchronized the Python backend `OFFSETS` dictionary identically with the Bash `start_sitl.sh` spawn logic, preventing massive initial positional corrections. |
| **UI Freezing due to Telemetry Overload** | Built the Quart asynchronous WebSocket server and utilized Leaflet canvas markers to handle massive telemetry data dumps without dropping browser frames. |
| **Fake Collision Tests** | Replaced hardcoded `time.sleep()` mock tests with actual dynamic obstacle injection APIs (`/api/obstacles/add_dynamic`) that physically challenge the ArduPilot physics engine. |

---

## 7. Automated Testing Suite

The system includes 18 fully automated integration scenarios via `test_swarm_scenarios.py`, accessible via the UI dropdown.
Notable tests include:
*   **Test 3 (Pattern Formation):** Verifies the rotational matrix and offset scaling.
*   **Test 4 (Self-Healing):** Tests the Dynamic Leader Election when `drone_1` is killed.
*   **Test 6 (Collision Avoidance):** Spawns a high-speed dynamic bird directly crossing the swarm's flight path.
*   **Test 18 (Real-World Avoidance):** Drops a gauntlet of static buildings and severe wind zones on top of the swarm to stress-test the APF algorithms and local minima escape logic.
