import logging
import os
from log_setup import setup_logger
from sitl_adapter import SITLAdapter
from waypoint_navigator import WaypointNavigator
import asyncio
import argparse
import time
import math

# Resolve waypoints.json relative to this script's directory so it works
# regardless of the current working directory.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WAYPOINT_FILE = os.path.join(SCRIPT_DIR, "waypoints.json")

def run_drone_mission(drone_id, connection_str, waypoints):
    logging.info(f"[{drone_id}] Starting mission on {connection_str}...")
    try:
        adapter = SITLAdapter(drone_id, connection_str)
        adapter.initialize()
    except RuntimeError as e:
        logging.error(f"[{drone_id}] ❌ CONNECTION FAILED: {e}")
        logging.error(f"[{drone_id}] ⚠ Is 'bash start_sitl.sh 5' running in WSL? Check /tmp/sitl_instance_*.log")
        return
    except Exception as e:
        logging.error(f"[{drone_id}] ❌ Unexpected connection error: {e}")
        return

    adapter.log_status()

    # Wait for GPS and EKF lock before setting GUIDED mode
    logging.info(f"[{drone_id}] ⏳ Waiting for GPS and EKF lock...")
    while True:
        adapter.master.recv_match(blocking=False)
        gps = adapter.master.messages.get('GPS_RAW_INT')
        pos = adapter.master.messages.get('GLOBAL_POSITION_INT')

        gps_ok = gps and gps.fix_type >= 3
        ekf_ok = pos and pos.lat != 0

        if gps_ok and ekf_ok:
            logging.info(f"[{drone_id}] 🌍 GPS+EKF aligned! Fix: {gps.fix_type}")
            break
        time.sleep(1)

    if not adapter.set_mode("GUIDED"):
        logging.error(f"[{drone_id}] ❌ Failed to set GUIDED mode")
        return
    adapter.log_status()

    if not adapter.arm_vehicle():
        logging.error(f"[{drone_id}] ❌ Failed to arm vehicle")
        return
    adapter.log_status()

    first_alt = waypoints[0]["altitude"]
    if not adapter.takeoff(first_alt):
        logging.error(f"[{drone_id}] ❌ Takeoff failed")
        return
    adapter.log_status()

    navigator = WaypointNavigator(adapter)
    if not navigator.execute(waypoints):
        logging.error(f"[{drone_id}] ❌ Waypoint navigation failed")
        return

    adapter.land()
    adapter.log_status()
    adapter.export_flight_path()
    logging.info(f"[{drone_id}] ✅ MISSION COMPLETE")


async def main():
    setup_logger()
    logging.info("=== MULTI-DRONE GCS SYSTEM STARTED ===")

    parser = argparse.ArgumentParser(description="Multi-Drone GCS Controller")
    parser.add_argument("--drones", type=int, default=3, choices=range(2, 11),
                        help="Number of drones to connect (2 to 10, default 3)")
    args = parser.parse_args()

    # Load waypoints using a temporary navigator instance
    temp_navigator = WaypointNavigator(None)
    try:
        base_waypoints = temp_navigator.load_from_json(WAYPOINT_FILE)
    except Exception as e:
        logging.error(f"❌ Failed to load waypoints: {e}")
        return

    # Body-frame V-formation offsets (in meters) — matches start_sitl.sh exactly
    # DX = lateral (+ = Right wing, - = Left wing), DY = longitudinal (- = behind leader)
    # Wide lateral spread + shallow depth = clearly visible V-shape
    OFFSETS = {
        0: (0,    0),     # Drone 1: Leader (Apex of V)
        1: (-25, -10),    # Drone 2: Left Wing Row 2
        2: (25,  -10),    # Drone 3: Right Wing Row 2
        3: (-50, -20),    # Drone 4: Left Wing Row 3
        4: (50,  -20),    # Drone 5: Right Wing Row 3
        5: (0,   -20),    # Drone 6: Center Row 3
        6: (-75, -30),    # Drone 7: Left Wing Row 4
        7: (75,  -30),    # Drone 8: Right Wing Row 4
        8: (-25, -30),    # Drone 9: Inner Left Row 4
        9: (25,  -30),    # Drone 10: Inner Right Row 4
    }

    tasks = []
    for i in range(args.drones):
        drone_id = f"drone_{i+1}"
        port = 14551 + i
        connection_str = f"udpin:0.0.0.0:{port}"

        # Base coordinate conversion factors
        lat_deg_per_meter = 1.0 / 111320.0
        base_lat = base_waypoints[0]["latitude"]
        lon_deg_per_meter = 1.0 / (111320.0 * math.cos(math.radians(base_lat)))

        # Calculate bearing angle from home to first waypoint (direction of flight)
        lat1, lon1 = 33.6844, 73.0479
        lat2 = base_waypoints[0]["latitude"]
        lon2 = base_waypoints[0]["longitude"]
        dy_path = lat2 - lat1
        dx_path = (lon2 - lon1) * math.cos(math.radians(lat1))
        bearing = math.atan2(dx_path, dy_path)

        # Get the body-frame offset for this drone index
        dx_body, dy_body = OFFSETS.get(i, (0, 0))

        # Rotate body-frame offsets by the bearing angle to align with flight path
        dx = dx_body * math.cos(bearing) + dy_body * math.sin(bearing)
        dy = -dx_body * math.sin(bearing) + dy_body * math.cos(bearing)

        offset_lat = dy * lat_deg_per_meter
        offset_lon = dx * lon_deg_per_meter

        drone_waypoints = []
        for wp in base_waypoints:
            drone_waypoints.append({
                "latitude":  wp["latitude"]  + offset_lat,
                "longitude": wp["longitude"] + offset_lon,
                "altitude":  wp["altitude"]
            })

        logging.info(
            f"[{drone_id}] V-Formation offset: dx_body={dx_body}m, dy_body={dy_body}m "
            f"→ lat_offset={offset_lat*111320:.2f}m, lon_offset={offset_lon*111320:.2f}m"
        )

        # Run each drone's mission in a separate OS thread to prevent event loop blocking
        tasks.append(asyncio.to_thread(run_drone_mission, drone_id, connection_str, drone_waypoints))

    # Run all drone missions concurrently
    await asyncio.gather(*tasks)
    logging.info("=== ALL SWARM MISSIONS COMPLETE ===")

if __name__ == "__main__":
    asyncio.run(main())

