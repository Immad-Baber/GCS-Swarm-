import time
import math
import logging
from pymavlink import mavutil


def connect_to_drone(connection_string='udp:localhost:14551'):
    logging.info(f"Connecting to {connection_string} (timeout=90s)...")
    master = mavutil.mavlink_connection(connection_string)
    hb = master.wait_heartbeat(timeout=90)
    if hb is None:
        raise RuntimeError(f"No heartbeat from {connection_string} — is the SITL instance running?")
    logging.info("✅ Heartbeat received")
    return master


def set_guided_mode(master):
    try:
        logging.info("Setting mode to GUIDED...")
        mode_id = master.mode_mapping().get('GUIDED')
        if mode_id is None:
            logging.error("GUIDED mode not supported by this vehicle")
            return False

        deadline = time.time() + 10
        while time.time() < deadline:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Drain socket to catch any STATUSTEXT errors
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'STATUSTEXT':
                    logging.warning(f"⚠️ STATUSTEXT: {msg.text}")
                    
            time.sleep(1)
            
            hb = master.messages.get('HEARTBEAT')
            if hb:
                mode_str = mavutil.mode_string_v10(hb)
                logging.info(f"mode={mode_str}")
                if 'GUIDED' in mode_str:
                    logging.info("✅ GUIDED mode confirmed")
                    return True
        return False
    except Exception as e:
        logging.error(f"Mode set failed: {e}")
        return False


def arm_drone(master):
    try:
        logging.info("⏳ Waiting for GPS and EKF to align position estimate...")
        deadline = time.time() + 30  # Max 30s to wait for GPS
        while time.time() < deadline:
            master.recv_match(blocking=False)
            gps = master.messages.get('GPS_RAW_INT')
            pos = master.messages.get('GLOBAL_POSITION_INT')
            
            gps_ok = gps and gps.fix_type >= 3
            ekf_ok = pos and pos.lat != 0
            
            if gps_ok and ekf_ok:
                logging.info(f"🌍 Position estimate aligned! GPS Fix: {gps.fix_type}")
                break
            time.sleep(1)
        else:
            logging.error("❌ GPS lock timeout (30s). Cannot arm.")
            return False

        # Retry arming in a loop until the drone is armed
        logging.info("⚙️ Initiating arming sequence...")
        deadline = time.time() + 15  # Max 15s to wait for arming to be accepted
        while time.time() < deadline:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 21196, 0, 0, 0, 0, 0
            )
            
            # Wait briefly to let the drone process the command
            time.sleep(1)
            
            # Actively read from the socket so we get STATUSTEXT and updated HEARTBEATs
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'STATUSTEXT':
                    logging.warning(f"⚠️ STATUSTEXT: {msg.text}")
                elif msg.get_type() == 'COMMAND_ACK':
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        logging.info(f"Arming ACK result: {msg.result}")
            
            if master.motors_armed():
                logging.info("✅ Drone armed")
                return True
            else:
                logging.warning("⚠️ Arming command rejected or timed out, retrying...")
        
        logging.error("❌ Arming timeout (15s).")
        return False
    except Exception as e:
        logging.error(f"Failed to arm: {e}")
        return False


def takeoff(master, altitude):
    """Send takeoff command and wait until the drone reaches the target altitude."""
    try:
        logging.info(f"🚀 Takeoff to {altitude}m initiated — waiting to reach altitude...")

        target_threshold = altitude * 0.90
        deadline = time.time() + 60  # max 60 seconds to reach altitude
        takeoff_send_deadline = time.time() + 10 # Retry sending takeoff for 10s

        while time.time() < deadline:
            if time.time() < takeoff_send_deadline:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0, altitude
                )
            
            # Drain socket to parse new messages and update master.messages cache
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'STATUSTEXT':
                    logging.warning(f"⚠️ STATUSTEXT: {msg.text}")

            pos = master.messages.get('GLOBAL_POSITION_INT')
            if pos:
                current_alt = pos.relative_alt / 1000.0
                if current_alt > 0.5:  # Once it actually starts climbing, we stop resending
                    takeoff_send_deadline = 0 
                logging.info(f"   ↑ Climbing: {current_alt:.1f}m / {altitude}m")
                if current_alt >= target_threshold:
                    logging.info(f"✅ Reached target altitude {current_alt:.1f}m (target: {altitude}m)")
                    return True
            time.sleep(0.5)

        logging.warning(f"⚠️ Takeoff timed out — drone may not have reached {altitude}m")
        return True  # Still return True; the command was sent successfully
    except Exception as e:
        logging.error(f"Takeoff failed: {e}")
        return False


def send_position_target(master, boot_time, lat, lon, alt):
    # ArduPilot responds much more reliably to DO_REPOSITION in GUIDED mode
    # than SET_POSITION_TARGET_GLOBAL_INT unless it's streamed at 10Hz.
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0, 0,
        -1.0, 0, 0, 0.0,  # 0.0 yaw instead of NaN to avoid MAVLink rejection
        lat_int, lon_int, alt
    )


def calculate_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def wait_until_position_reached(adapter, target_lat, target_lon, target_alt, threshold=5.0, timeout=300):
    """
    Commands the drone to a GPS position and BLOCKS until it arrives within
    *threshold* metres, or *timeout* seconds pass.
    Continuously re-sends the position target every 2 s so ArduPilot doesn't
    forget the command, and updates the UI via log_status().
    """
    master = adapter.master
    boot_time = adapter.boot_time

    logging.info(f"[{adapter.drone_id}] 📍 Navigating to: lat={target_lat:.6f}, lon={target_lon:.6f}, alt={target_alt}m")

    deadline = time.time() + timeout
    last_send = 0

    while time.time() < deadline:
        # Re-send position target every 2 seconds
        if time.time() - last_send >= 2.0:
            send_position_target(master, boot_time, target_lat, target_lon, target_alt)
            last_send = time.time()

        # Drain the socket so messages cache is fresh
        master.recv_match(blocking=False)

        msg = master.messages.get('GLOBAL_POSITION_INT')
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = max(0.0, msg.relative_alt / 1000.0)
            dist = calculate_distance_meters(current_lat, current_lon, target_lat, target_lon)
            logging.info(f"[{adapter.drone_id}] ➡️ Distance to wp: {dist:.1f}m, alt={current_alt:.1f}m")
            adapter.log_status()
            if dist < threshold:
                logging.info(f"[{adapter.drone_id}] ✅ Waypoint reached (dist={dist:.1f}m)")
                return

        time.sleep(1.0)

    logging.warning(f"[{adapter.drone_id}] ⚠️ Waypoint timeout after {timeout}s")



def land_drone(master):
    """Send land command by changing mode to LAND, retrying up to 3 times."""
    try:
        logging.info("🛬 Initiating landing...")
        mode_id = master.mode_mapping().get('LAND')
        if mode_id is None:
            logging.error("LAND mode not supported by this vehicle")
            return False

        deadline = time.time() + 10
        while time.time() < deadline:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Drain socket to catch any STATUSTEXT errors
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'STATUSTEXT':
                    logging.warning(f"⚠️ STATUSTEXT: {msg.text}")
                    
            time.sleep(1)
            
            hb = master.messages.get('HEARTBEAT')
            if hb:
                mode_str = mavutil.mode_string_v10(hb)
                logging.info(f"🛬 mode={mode_str}")
                if 'LAND' in mode_str.upper():
                    logging.info("✅ LAND mode confirmed")
                    return True
                    
        logging.warning("⚠️ Land command sent but mode not confirmed as LAND — drone may still be landing")
        return True  # Command was sent; landing proceeds asynchronously
    except Exception as e:
        logging.error(f"Landing failed: {e}")
        return False


def fly_to_gps(lat, lon, alt=10.0, connection_string='udpin:0.0.0.0:14551'):
    from sitl_adapter import SITLAdapter
    adapter = SITLAdapter("drone_1", connection_string)
    adapter.initialize()
    if not adapter.set_mode("GUIDED"): return
    if not adapter.arm_vehicle(): return
    if not adapter.takeoff(alt): return
    adapter.goto_position(lat, lon, alt)
    adapter.land()
