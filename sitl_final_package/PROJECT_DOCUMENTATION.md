# GCS Swarm Commander - Comprehensive Project Documentation

## 1. Project Overview
The **GCS Swarm Commander** is a complete Software-In-The-Loop (SITL) simulation environment, telemetry server, and Ground Control Station (GCS) designed to manage, command, and monitor a multi-drone swarm. The project incorporates advanced swarm flight mechanics, real-time telemetry visualization, and a sophisticated Artificial Potential Field (APF) based obstacle avoidance system.

The core objective of the project is to provide a highly scalable, robust platform for autonomous drone swarm research and operations, allowing up to 10 drones to fly in coordinated formations while actively avoiding dynamic and static hazards without requiring mission-level logic changes.

---

## 2. System Architecture
The system is built using a modern, asynchronous architecture split into several distinct layers:

### 2.1 Core Components
*   **ArduPilot SITL:** The physics and flight dynamics engine used to simulate the drones.
*   **MAVProxy & pymavlink:** The communication bridge used to interface with the SITL instances via the MAVLink protocol.
*   **Quart Telemetry Server:** An asynchronous Python web framework (Quart) that provides a REST API for command execution and a WebSocket server for live telemetry broadcasting.
*   **GCS Frontend (`index.html`):** A custom-built, dark-themed dashboard utilizing HTML, CSS, JavaScript, and Leaflet.js for real-time map tracking, telemetry display, and interactive obstacle placement.

### 2.2 Telemetry Data Flow
1.  **Drone to Adapter:** Each ArduCopter SITL instance runs on a unique UDP port. A dedicated `SITLAdapter` thread in Python connects to this port using `pymavlink` and queries MAVLink messages (e.g., `GLOBAL_POSITION_INT`, `SYS_STATUS`, `ATTITUDE`, `HEARTBEAT`) at 1 Hz.
2.  **Adapter to Server:** The `SITLAdapter` packages this data into JSON and sends an HTTP POST request to the Quart server's `/send_telemetry` endpoint.
3.  **Server to Frontend:** The Quart server pushes the received JSON into an `asyncio.Queue`. A background broadcast worker reads from this queue and pushes the data to all connected web clients via WebSockets (`ws://localhost:5000/ws`).
4.  **Frontend Visualization:** The UI receives the WebSocket stream, updates the battery, altitude, and mode indicators, and moves the drone markers and flight paths on the Leaflet map.

---

## 3. Setup and Execution

### 3.1 Prerequisites
*   Linux (Ubuntu), WSL (Windows Subsystem for Linux), or Git Bash on Windows.
*   Python 3.8+
*   ArduPilot/ArduCopter SITL firmware (v4.3.x or 4.4.x).
*   Required Python packages (found in `requirements.txt`): `pymavlink`, `quart`, `quart-cors`, `requests`.

### 3.2 Installation
```bash
# Create and activate a virtual environment
python3 -m venv final_venv
source final_venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3.3 Running the Swarm (Step-by-Step)
You will need three separate terminal instances to run the full stack:

**Terminal 1: Start the SITL Simulator**
```bash
# Launch 3 drones (can be scaled up to 10)
./start_sitl.sh 3
```
*Wait 3-4 minutes for the drones to acquire a GPS lock before proceeding.*

**Terminal 2: Start the Telemetry Server**
```bash
cd mavlink_integration
source ../final_venv/bin/activate
python3 telemetry_server.py
```
*Open `http://localhost:5000` in your web browser to view the GCS Dashboard.*

**Terminal 3: Run the Mission Controller**
```bash
cd mavlink_integration
source ../final_venv/bin/activate
python3 main.py --drones 3
```

---

## 4. Algorithms and Strategies

### 4.1 Swarm Formation Algorithm (V-Formation)
The swarm is designed to fly in a coordinated V-shape formation using a **Leader-Follower Strategy**.

*   **Leader:** `drone_1` acts as the apex (center) of the formation.
*   **Followers:** Subsequent drones are assigned fixed body-frame offsets (dx, dy) relative to the leader. For example:
    *   Drone 2 (Left Wing): `(-25m, -10m)`
    *   Drone 3 (Right Wing): `(25m, -10m)`
*   **Rotation:** Before execution, the system calculates the bearing angle of the flight path. The body-frame offsets are then rotated using a 2D rotation matrix so that the V-formation always faces the direction of travel, rather than locking to cardinal North.
*   **Coordinate Shifting:** The `SwarmManager` reads the mission waypoints (from `mission1.json`) and dynamically shifts the entire mission to match the leader's actual real-world GPS spawn location to prevent sudden, erratic drone movements upon takeoff.

### 4.2 Artificial Potential Fields (APF) - Obstacle Avoidance
To navigate complex environments autonomously, the project utilizes an Artificial Potential Field (APF) algorithm. This is a purely mathematical approach where the drone is treated as a particle moving through a magnetic field.

*   **Attractive Force:** The destination waypoint acts as an attractive magnet, pulling the drone forward.
*   **Repulsive Force:** Obstacles act as repulsive magnets, pushing the drone away.
*   **Quadratic Falloff:** To ensure smooth, graceful maneuvering, the repulsive force follows a quadratic curve. The force is zero outside the `influence_radius`, and scales quadratically as the drone approaches the physical surface of the obstacle.

**Equation:**
`Force = Peak_Force * ((R_influence - Current_Distance) / (R_influence - R_surface))^2`

### 4.3 Additive Layering Architecture
A major challenge was adding obstacle avoidance without rewriting the high-level mission logic (`waypoint_navigator.py`). The solution was **Additive Layering**.
The APF avoidance logic is surgically injected into the lowest-level positional loop (`wait_until_position_reached` in `drone_controller.py`). Every 2 seconds, this loop checks the `obstacle_map`. If a repulsive vector is generated, it is mathematically added to the target destination coordinate. This seamlessly tricks the ArduPilot physics engine into curving around the obstacle without the mission planner ever knowing it happened.

---

## 5. Handling Specific Environmental Hazards

The `obstacle_map.py` singleton manages a live representation of the environment, supporting three distinct hazard types:

### 5.1 Static Obstacles (Buildings & Walls)
*   **Properties:** Possess a radius and a maximum altitude (`max_alt_m`).
*   **Altitude Filtering:** If a drone flies higher than the roof of the building (plus a 15m safety buffer), the APF algorithm ignores the building entirely, allowing the drone to fly straight over it without being pushed laterally.
*   **Behavior:** Applies a standard repulsive vector if the drone is at or below roof height.

### 5.2 Dynamic Obstacles (Birds & Moving Aircraft)
*   **Properties:** Possess an altitude, radius, and continuous velocities (`vel_lat_dps`, `vel_lon_dps`).
*   **Background Threading:** A dedicated Python thread runs at 10 Hz, continuously updating the physical coordinates of the dynamic obstacles.
*   **Altitude Matching:** Birds only affect drones flying in their exact altitude band (±15m). A drone at 50m ignores a bird flying at 10m.
*   **Collision Envelope:** Because dynamic obstacles move quickly, their influence radius is aggressively expanded (40 meters) to ensure early detection and avoidance.

### 5.3 Wind Zones & Turbulence
*   **Properties:** Modeled as infinite vertical cylinders with a `strength` multiplier.
*   **Behavior:** The altitude filter is disabled for wind zones; turbulence affects the drone regardless of height. The `strength` multiplier directly amplifies the APF repulsive force, forcing drones to take a much wider, sweeping berth around severe weather zones compared to solid objects.

---

## 6. GCS Frontend Integration

The telemetry server and GCS frontend include features to bridge the mathematical algorithms with human operators:
1.  **Live APF Visualization:** The UI Leaflet map dynamically draws static walls (red), wind zones (orange), and dynamic birds (purple pulsing markers). The UI polls the backend `/api/obstacles/status` every 2 seconds to keep the visual map synchronized with the APF logic.
2.  **Interactive Obstacle Placement:** Operators can click quick-add buttons or click directly on the map to spawn buildings, wind zones, or birds directly in the path of the swarm to test reactivity live.
3.  **Swarm Commands:** Dedicated UI panels allow users to command individual drones (Arm, Takeoff, Land) or issue swarm-wide fleet commands.

---

## 7. Automated Testing Strategy

To ensure absolute reliability, testing is handled in two tiers:

1.  **Mathematical Unit Tests (`test_obstacle_avoidance.py`):**
    *   Verifies that empty maps apply zero force.
    *   Validates altitude filtering logic (flying over buildings).
    *   Confirms wind multipliers scale correctly.
    *   Ensures that forces cancel out properly when overlapping (preventing the drone from being launched into infinity).
2.  **SITL Integration Tests (`test_swarm_scenarios.py`):**
    *   Contains 18 distinct scenarios.
    *   Tests live ArduPilot physics reacting to the APF commands.
    *   Scenario 6 spawns a live bird intercepting the swarm.
    *   Scenario 18 spawns a gauntlet of static buildings and wind turbulence to test real-world complex avoidance.

## 8. Summary of Solutions & Approaches
*   **Problem:** Drones crashing into unseen obstacles because SITL lacks visual sensors.
    *   **Solution:** Built a pure Python Artificial Potential Field (APF) layer to act as a virtual companion computer intercepting and altering MAVLink target commands.
*   **Problem:** Drones behaving erratically during formation takeoff (flying off to random locations).
    *   **Solution:** Removed excessively large offsets and aligned the `SwarmManager` formation offsets exactly with the `start_sitl.sh` spawn coordinates. Implemented dynamic coordinate shifting to ground the JSON missions to the leader's actual GPS spawn location.
*   **Problem:** Drones getting stuck in "Local Minima" (cornered by overlapping obstacles).
    *   **Solution:** The APF forces mathematically cancel out, causing the drone to gracefully hover in place rather than executing dangerous maneuvers, resuming its mission automatically once the dynamic hazard clears.
