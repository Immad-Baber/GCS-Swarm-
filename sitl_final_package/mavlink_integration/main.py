import logging
from log_setup import setup_logger
from sitl_adapter import SITLAdapter
from waypoint_navigator import WaypointNavigator
import asyncio
import argparse
import time

WAYPOINT_FILE = "waypoints.json"

def run_drone_mission(drone_id, connection_str, waypoints):
    logging.info(f"[{drone_id}] Starting mission task on {connection_str}...")
    adapter = SITLAdapter(drone_id, connection_str)
    adapter.initialize()
    adapter.log_status()

    # Wait for GPS and EKF lock to align position estimate before setting GUIDED mode
    logging.info(f"[{drone_id}] ⏳ Waiting for GPS and EKF lock to align position estimate...")
    while True:
        adapter.master.recv_match(blocking=False)
        gps = adapter.master.messages.get('GPS_RAW_INT')
        pos = adapter.master.messages.get('GLOBAL_POSITION_INT')
        
        gps_ok = gps and gps.fix_type >= 3
        ekf_ok = pos and pos.lat != 0
        
        if gps_ok and ekf_ok:
            logging.info(f"[{drone_id}] 🌍 Position estimate aligned! GPS Fix: {gps.fix_type}")
            break
        time.sleep(1)

    if not adapter.set_mode("GUIDED"):
        logging.error(f"[{drone_id}] Failed to set mode to GUIDED")
        return
    adapter.log_status()

    if not adapter.arm_vehicle():
        logging.error(f"[{drone_id}] Failed to arm vehicle")
        return
    adapter.log_status()

    # Takeoff to the first waypoint's altitude
    first_alt = waypoints[0]["altitude"]
    if not adapter.takeoff(first_alt):
        logging.error(f"[{drone_id}] ❌ Takeoff failed")
        return
    adapter.log_status()

    # Execute all waypoints
    navigator = WaypointNavigator(adapter)
    if not navigator.execute(waypoints):
        logging.error(f"[{drone_id}] ❌ Waypoint navigation failed")
        return

    adapter.land()
    adapter.log_status()
    adapter.export_flight_path()
    logging.info(f"[{drone_id}] === MISSION COMPLETE ===")

async def main():
    setup_logger()
    logging.info("=== MULTI-DRONE GCS SYSTEM STARTED ===")

    parser = argparse.ArgumentParser(description="Multi-Drone GCS Controller")
    parser.add_argument("--drones", type=int, default=3, choices=range(2, 6),
                        help="Number of drones to connect (2 to 5, default 3)")
    args = parser.parse_args()

    # Load waypoints using a temporary navigator instance
    temp_navigator = WaypointNavigator(None)
    try:
        base_waypoints = temp_navigator.load_from_json(WAYPOINT_FILE)
    except Exception as e:
        logging.error(f"❌ Failed to load waypoints: {e}")
        return

    tasks = []
    for i in range(args.drones):
        drone_id = f"drone_{i+1}"
        port = 14551 + i
        connection_str = f"udpin:0.0.0.0:{port}"

        # Dynamically offset waypoints for swarm formation (~16.6m offset per drone)
        offset_lat = i * 0.00015
        offset_lon = i * 0.00015
        drone_waypoints = []
        for wp in base_waypoints:
            drone_waypoints.append({
                "latitude": wp["latitude"] + offset_lat,
                "longitude": wp["longitude"] + offset_lon,
                "altitude": wp["altitude"]
            })

        # Run each drone's mission task in a separate OS thread to prevent event loop blocking
        tasks.append(asyncio.to_thread(run_drone_mission, drone_id, connection_str, drone_waypoints))

    # Run all drone missions concurrently
    await asyncio.gather(*tasks)
    logging.info("=== ALL SWARM MISSIONS COMPLETE ===")

if __name__ == "__main__":
    asyncio.run(main())

