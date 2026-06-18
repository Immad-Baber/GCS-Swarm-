import time
import math
import logging
from pymavlink import mavutil


def connect_to_drone(connection_string='udp:localhost:14551'):
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    logging.info("✅ Heartbeat received")
    return master


def set_guided_mode(master):
    try:
        mode_id = master.mode_mapping()['GUIDED']
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        logging.info("🔁 Set mode to GUIDED")
        time.sleep(2)
        return True
    except Exception as e:
        logging.error(f"Mode set failed: {e}")
        return False


def arm_drone(master):
    try:
        logging.info("⏳ Waiting for GPS and EKF to align position estimate...")
        while True:
            master.recv_match(blocking=False)
            gps = master.messages.get('GPS_RAW_INT')
            pos = master.messages.get('GLOBAL_POSITION_INT')
            
            gps_ok = gps and gps.fix_type >= 3
            ekf_ok = pos and pos.lat != 0
            
            if gps_ok and ekf_ok:
                logging.info(f"🌍 Position estimate aligned! GPS Fix: {gps.fix_type}")
                break
            time.sleep(1)

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        logging.info("⚙️ Arming drone")
        master.motors_armed_wait()
        logging.info("✅ Drone armed")
        return True
    except Exception as e:
        logging.error(f"Failed to arm: {e}")
        return False


def takeoff(master, altitude):
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )
        logging.info(f"🚀 Takeoff to {altitude}m initiated")
        time.sleep(10)
        return True
    except Exception as e:
        logging.error(f"Takeoff failed: {e}")
        return False


def send_position_target(master, boot_time, lat, lon, alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    master.mav.set_position_target_global_int_send(
        int((time.time() - boot_time) * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        lat_int, lon_int, alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )


def calculate_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def wait_until_position_reached(master, boot_time, target_lat, target_lon,
                                target_alt, threshold=3.0):
    """
    Sends position targets repeatedly until the drone reaches the location.
    """
    from sitl_adapter import SITLAdapter  # Import here to avoid circular import
    temp_adapter = SITLAdapter("udp:localhost:14551")
    temp_adapter.master = master  # Use existing master
    temp_adapter.boot_time = boot_time

    logging.info(f"📍 Navigating to: lat={target_lat}, lon={target_lon}, alt={target_alt}m")

    while True:
        # Send position target
        send_position_target(master, boot_time, target_lat, target_lon, target_alt)

        # Receive and log position
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0

            dist = calculate_distance_meters(
                current_lat, current_lon, target_lat, target_lon)
            logging.info(f"📡 Current: lat={current_lat:.6f}, lon={current_lon:.6f}, "
                         f"alt={current_alt:.1f} → Distance: {dist:.2f}m")

            # 🆕 Log battery + mode status during every iteration
            temp_adapter.log_status(override_pos=(current_lat, current_lon, current_alt))

            if dist < threshold:
                logging.info("✅ Target location reached")
                break

        time.sleep(0.2)

    logging.info("⏸️ Hovering at target location...")
    time.sleep(1)



def land_drone(master):
    try:
        logging.info("🛬 Initiating landing...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        return True
    except Exception as e:
        logging.error(f"Landing failed: {e}")
        return False


def fly_to_gps(lat, lon, alt=10.0, connection_string='udp:localhost:14551'):
    boot_time = time.time()
    master = connect_to_drone(connection_string)
    if not set_guided_mode(master): return
    if not arm_drone(master): return
    if not takeoff(master, alt): return
    wait_until_position_reached(master, boot_time, lat, lon, alt)
    land_drone(master)