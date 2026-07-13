import time
import logging
import sys
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def test_drone():
    connection_string = 'udp:127.0.0.1:14551'
    logging.info(f"Connecting to {connection_string}...")
    
    # Connect
    master = mavutil.mavlink_connection(connection_string)
    hb = master.wait_heartbeat(timeout=10)
    if not hb:
        logging.error("Failed to connect: No heartbeat received from SITL.")
        sys.exit(1)
    logging.info(f"Connected! System ID: {master.target_system}, Component ID: {master.target_component}")

    # Set parameters to bypass checks
    logging.info("Setting parameters to bypass checks...")
    master.mav.param_set_send(master.target_system, master.target_component, b'ARMING_CHECK', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(1)

    # Change to GUIDED mode
    logging.info("Setting to GUIDED mode...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Wait for GUIDED mode confirmation
    start_time = time.time()
    while time.time() - start_time < 5:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            mode_str = mavutil.mode_string_v10(msg)
            logging.info(f"Current Mode: {mode_str}")
            if 'GUIDED' in mode_str:
                break
    else:
        logging.error("Failed to enter GUIDED mode.")
        sys.exit(1)

    # Force Arm
    logging.info("Sending Force Arm Command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 21196, 0, 0, 0, 0, 0
    )

    # Wait for ARM confirmation
    armed = False
    start_time = time.time()
    while time.time() - start_time < 5:
        msg = master.recv_match(blocking=True, timeout=1)
        if not msg:
            continue
        if msg.get_type() == 'STATUSTEXT':
            logging.info(f"STATUSTEXT: {msg.text}")
        elif msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            logging.info(f"ACK: Command {msg.command} Result {msg.result}")
        elif msg.get_type() == 'HEARTBEAT':
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if armed:
                logging.info("✅ Drone Armed!")
                break
    
    if not armed:
        logging.error("Drone failed to arm.")
        sys.exit(1)

    # Takeoff
    logging.info("Sending Takeoff Command (5m)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 5
    )

    # Monitor Altitude
    for _ in range(15):
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            alt = msg.relative_alt / 1000.0
            logging.info(f"Altitude: {alt:.2f}m")
            if alt > 4.5:
                logging.info("✅ Takeoff target reached!")
                break
    
    # Land
    logging.info("Sending Land Command...")
    land_mode = master.mode_mapping()['LAND']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        land_mode
    )

    for _ in range(15):
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            alt = msg.relative_alt / 1000.0
            logging.info(f"Descending... Altitude: {alt:.2f}m")
            if alt < 0.2:
                logging.info("✅ Landed!")
                break

    logging.info("Test complete.")

if __name__ == '__main__':
    test_drone()
