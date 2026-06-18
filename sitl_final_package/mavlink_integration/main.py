import logging
from log_setup import setup_logger
from sitl_adapter import SITLAdapter
from waypoint_navigator import WaypointNavigator
import asyncio
import httpx
import time

WAYPOINT_FILE = "waypoints.json"

# push_telemetry removed because sitl_adapter.log_status() now posts telemetry
async def main():
    setup_logger()
    logging.info("=== GCS SYSTEM STARTED ===")

    adapter = SITLAdapter("udp:localhost:14551")
    adapter.initialize()
    adapter.log_status()

    if not adapter.set_mode("GUIDED"):
        logging.error("Failed to set mode")
        return
    adapter.log_status()

    if not adapter.arm_vehicle():
        logging.error("Failed to arm vehicle")
        return
    adapter.log_status()

    # Load waypoints
    navigator = WaypointNavigator(adapter)

    try:
        waypoints = navigator.load_from_json(WAYPOINT_FILE)
    except Exception as e:
        logging.error(f"❌ Failed to load waypoints: {e}")
        return

    # Takeoff to the first waypoint's altitude
    first_alt = waypoints[0]["altitude"]
    if not adapter.takeoff(first_alt):
        logging.error("❌ Takeoff failed")
        return
    adapter.log_status()

    # Execute all waypoints
    if not navigator.execute(waypoints):
        logging.error("❌ Waypoint navigation failed")
        return

    adapter.land()
    adapter.log_status()
    adapter.export_flight_path()

    logging.info("=== MISSION COMPLETE ===")

if __name__ == "__main__":
    asyncio.run(main())