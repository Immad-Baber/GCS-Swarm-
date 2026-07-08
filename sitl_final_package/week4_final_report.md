# GCS+ Radar

**Team Lead**
Ms Malaika Tehsin

**Submitted By**
Syed Sheharyar Ali, Immad Babar

**Date Of Creation**
07/2/2026

**Objective**
Swarm Level Commands and Basic Formation 

**Summer 2026**
UAV Dependability Lab
FAST – National University of Computer & Emerging Science

---

## Overview
Week 4 adds interactive command-and-control capabilities to the Ground Control Station (GCS). The system now supports:
- **Swarm commands**: arm all, takeoff all, land all with concurrent execution
- **Autonomous Geometric Missions**: The system now supports dynamic, fully autonomous flight paths. Users can select predefined geometric missions (Line, Square, Triangle) from the dashboard. The backend dynamically calculates waypoints relative to the swarm's live GPS position and executes them flawlessly with auto-landing upon completion.
- **Individual drone control**: arm, takeoff, and land a single selected drone
- **Basic formation flight**: leader-follower triangle formation with 10m spacing
- **Formation logging**: inter-drone distances and positions logged to JSON-lines
- **REST API**: 12 new endpoints for all operations
- **GCS Dashboard**: interactive Swarm Command Panel with buttons, loading states, and feedback
- **Behavior test scenarios**: 3 automated test scripts exercising the full feature set

---

## Files Added

**1. `swarm_manager.py`: Swarm Manager Module**
* **Purpose:** Central orchestration layer that holds references to all drone SITLAdapter instances and exposes swarm wide and individual drone commands.
* **Thread safety:** All drone operations use ThreadPoolExecutor for concurrent execution and a threading lock for dict access.

**2. `formation_manager.py`: Formation Logic**
* **Purpose:** Implements leader-follower and fixed-offset formation logic with GPS coordinate math.
* **Formation:** Triangle (Leader at front, followers left/back and right/back at 10m offsets).

**3. `formation_logger.py`: Formation State Logger**
* **Purpose:** Logs formation snapshots to `logs/formation_log.jsonl` for analysis, tracking target positions, actual positions, and inter-drone distances.

**4. `test_swarm_scenarios.py`: Behavior-Based Test Scenarios**
* **Purpose:** Automated test script that exercises swarm commands via the REST API (Swarm lifecycle, individual control, and formation flight).

**5. `gen_missions.py` — Procedural Mission Generator**
* **Purpose:** A utility script to mathematically generate perfect geometric flight paths (Square, Triangle, Line) based on a starting origin coordinate.
* **Outputs:** Generates `mission1.json` (Line), `mission2.json` (Square), and `mission3.json` (Triangle) containing exact latitude, longitude, and altitude waypoints.

**6. `mission1.json`, `mission2.json`, `mission3.json` — Mission Definition Files**
* **Purpose:** JSON files containing the sequential waypoints for autonomous navigation. Loaded dynamically by the swarm manager upon takeoff.

---

## Files Modified

**7. `telemetry_server.py`: REST API Endpoints**
* **Changes:** Added 12 new REST API endpoints for swarm control, individual drone control, formation commands, and status queries.
* **Mission Integration:** Updated `/api/swarm/takeoff_all` and `/api/drone/<id>/takeoff` to accept a `mission` parameter.
* **Architecture Update:** Added a standalone background worker (`telemetry_polling_worker`) that continuously polls all connected drones for telemetry and broadcasts it over WebSockets. This decouples manual UI control from autonomous scripts.

**8. `index.html`: GCS Dashboard Command Panel**
* **Changes:** Added interactive sections to the sidebar.
* **Swarm Commands:** Connect, Arm All, Takeoff All, Land All, Altitude input.
* **Mission Selection:** Added a "MISSION" selection dropdown (Mission 1, Mission 2, Mission 3) for executing predefined autonomous shapes.
* **Formation Commands:** Triangle formation, Distances grid.
* **Individual Drone Control:** Arm, Takeoff, Land for the currently selected drone.

---

## How to Run

**Step 1: Start SITL Drones (in WSL)**
```bash
# Open a WSL terminal
cd /mnt/d/gcs_s2025/gcs_s2025/sitl_final_package
# Start 3 ArduCopter SITL instances
bash start_sitl.sh 3
# Wait ~3-4 minutes for GPS lock
```

**Step 2: Start the GCS Server (in WSL same or new terminal)**
```bash
cd /mnt/d/gcs_s2025/gcs_s2025/sitl_final_package/mavlink_integration
source ../venv/bin/activate
python telemetry_server.py
```

**Step 3: Open the GCS Dashboard (in your browser)**
Navigate to `http://localhost:5000`

**Step 4: Use the Dashboard**
- Click **CONNECT** -> Connects to 3 SITL drones
- Click **ARM ALL** -> Arms all drones (sets GUIDED mode automatically)
- Select a Mission (e.g. Mission 2 - Square) and set altitude.
- Click **TAKEOFF ALL** -> All drones take off and autonomously fly the mission, landing upon completion.
- Click **TRIANGLE** -> Drones move into triangle formation.
- Click **DISTANCES** -> Shows inter-drone distances.
- Click on a drone tab (e.g., DRONE 2) -> Individual commands appear.

---

## Bug Fixes & Improvements 

The following bugs were identified during live testing of the Week 4 GCS dashboard and corrected. All fixes preserve existing functionality.

**Bug 1: Altitude Display Always Shows 10m (Frontend + Backend)**
* **Root Cause:** `takeoff()` used a flat `time.sleep(10)` and returned True unconditionally, lying about reaching target altitude if requested > 10m.
* **Fix Applied:** Replaced the flat sleep with a polling loop that monitors `GLOBAL_POSITION_INT` and waits until the drone reaches 90% of the commanded altitude. Enhanced the UI to show actual vs. commanded altitude with a progress bar and status badges (CLIMBING -> AT TARGET).

**Bug 2: Individual LAND Button Did Not Work**
* **Root Cause:** Single-shot land command with no confirmation often dropped. Also, the adapter blocked forever if landing failed.
* **Fix Applied:** Replaced with a 3-attempt retry loop that checks the HEARTBEAT for mode switch. Added a 120-second graceful timeout to the landing loop.

**Bug 3: Individual Takeoff Altitude Shared with Fleet Altitude Input**
* **Root Cause:** The Individual Drone Control section had no altitude input of its own.
* **Fix Applied:** Added a dedicated altitude input field `#alt-input-one` directly inside the Individual Drone Control section.

**Bug 4: Individual Arm and Land Buttons Hanging the UI**
* **Root Cause:** `arm_drone` and `land_drone` blocked the API response until fully complete (which could take minutes).
* **Fix Applied:** Added timeouts for arming. Added `wait_for_land=False` so the REST API returns success immediately while the drone lands asynchronously in the background.

**Bug 5: Individual "LAND" Command Causes Drone to Fly Away**
* **Root Cause:** Sending `MAV_CMD_NAV_LAND` with 0, 0 coords commanded the drone to fly to Null Island (Equator/Africa).
* **Fix Applied:** Changed mechanism to use a MAVLink mode change to `LAND`, forcing an immediate vertical descent.

**Bug 6: Server Crash (Disconnected) during Individual Commands**
* **Root Cause:** Race conditions over the MAVLink socket between the continuous telemetry worker and individual command polling loops.
* **Fix Applied:** Removed raw `recv_match()` calls from command loops, relying exclusively on the thread-safe telemetry cache.

**Bug 7: Drones Not Moving During Formation Commands**
* **Root Cause:** Socket draining caused formation coordinate calculations to receive `None` and silently abort.
* **Fix Applied:** Formation scripts now safely read `GLOBAL_POSITION_INT` from the thread-safe dictionary cache.

**Bug 8: Drones Remembering Previous Landing Positions ("Zombie Spawns")**
* **Root Cause:** ArduCopter SITL instances use a virtual EEPROM stored inside their `instance_X` directories. Without wiping, drones would spawn exactly where they landed in the previous test (often miles away) rather than in their initial V-formation.
* **Fix Applied:** Modified `start_sitl.sh` to explicitly run `rm -rf instance_$INSTANCE` and wipe the virtual EEPROM folders on every startup, guaranteeing a 100% clean spawn at the launchpad.

**Bug 9: Geometric Shapes Warped by Hardcoded JSON Coordinates**
* **Root Cause:** The `mission2.json` and `mission3.json` waypoints were hardcoded to the launchpad. If taking off from elsewhere, drones would fly backward to the start before drawing the shape.
* **Fix Applied:** Implemented a **Dynamic Origin Shift**. `swarm_manager.py` reads the drone's live GPS position at takeoff, calculates the delta against the JSON file's origin, and shifts every waypoint mathematically. Shapes are perfectly drawn from any starting point.

**Bug 10: Follower Drones Flying Miles Away (Double-Offset Bug)**
* **Root Cause:** Follower drones (Drone 2, Drone 3) calculated their autonomous paths relative to their own GPS positions, double-counting the V-formation spacing (physical + mathematical).
* **Fix Applied:** Refactored `takeoff_all` and `takeoff_drone` to designate the Leader Drone's (Drone 1) live GPS position as the absolute shared origin for the entire swarm.

**Bug 11: Waypoint Navigation Failing to Reach Exact Targets**
* **Root Cause:** The `wait_until_position_reached` function prematurely flagged waypoints as "reached", resulting in rounded corners and deformed geometric paths. 
* **Fix Applied:** Refactored `wait_until_position_reached` into a robust blocking loop utilizing the haversine formula to verify the drone is precisely within 1.0 meters of the target coordinate before proceeding. Spun off execution into a background daemon thread (`_mission_worker`) for autonomous auto-landing capability.
