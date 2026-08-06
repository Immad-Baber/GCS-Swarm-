# drone_controller.py
# ─────────────────────────────────────────────────────────────────────────────
# Low-Level MAVLink Command & Flight Controller Module
#
# Technical Purpose:
#   Provides foundational MAVLink communication primitives for communicating
#   with ArduPilot instances via PyMAVLink socket connections.
#
# Core Functions & Mechanics:
#   1. `connect_to_drone`: Establishes UDP connection and waits for HEARTBEAT.
#   2. `set_guided_mode`: Sends MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (GUIDED mode).
#   3. `arm_drone`: Verifies EKF/GPS fix alignment before sending ARM command.
#   4. `takeoff`: Dispatches MAV_CMD_NAV_TAKEOFF and polls relative altitude.
#   5. `send_position_target`: Emits SET_POSITION_TARGET_GLOBAL_INT packets.
#   6. APF Integration: Checks `obstacle_map` during position target calculations
#      to inject real-time repelling vector offsets.
# ─────────────────────────────────────────────────────────────────────────────

import time
import math
import logging
from pymavlink import mavutil
from obstacle_map import obstacle_map  # Universal obstacle avoidance layer



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
        deadline = time.time() + 30
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

        logging.info("⚙️ Initiating arming sequence...")
        deadline = time.time() + 15
        while time.time() < deadline:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(1)
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
                logging.info("✅ Drone armed and ready")
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
        deadline = time.time() + 60
        takeoff_send_deadline = time.time() + 10

        while time.time() < deadline:
            if time.time() < takeoff_send_deadline:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0, altitude
                )
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'STATUSTEXT':
                    logging.warning(f"⚠️ STATUSTEXT: {msg.text}")
            pos = master.messages.get('GLOBAL_POSITION_INT')
            if pos:
                current_alt = pos.relative_alt / 1000.0
                if current_alt > 0.5:
                    takeoff_send_deadline = 0
                logging.info(f"   ↑ Climbing: {current_alt:.1f}m / {altitude}m")
                if current_alt >= target_threshold:
                    logging.info(f"✅ AIRBORNE — reached {current_alt:.1f}m (target: {altitude}m)")
                    return True
            time.sleep(0.5)

        logging.warning(f"⚠️ Takeoff timed out — drone may not have reached {altitude}m")
        return True
    except Exception as e:
        logging.error(f"Takeoff failed: {e}")
        return False


def send_position_target(master, boot_time, lat, lon, alt):
    """Send a DO_REPOSITION command — ArduPilot's most reliable GUIDED nav command."""
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0, 0,
        -1.0, 0, 0, 0.0,
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


def wait_until_position_reached(adapter, target_lat, target_lon, target_alt,
                                 threshold=6.0, timeout=300):
    """
    Command the drone to a GPS position and BLOCK until it arrives within
    *threshold* metres, or *timeout* seconds elapse.

    Key improvements over previous versions
    ────────────────────────────────────────
    1. **Goal-biased APF**: passes the waypoint position to get_avoidance_vector()
       so the APF chooses the tangent direction that leads AROUND the obstacle
       toward the goal, eliminating circular orbit traps.

    2. **Near-waypoint APF suppression**: when within 2.5× threshold the APF
       is disabled entirely — avoidance noise would cause oscillation.

    3. **Settle check**: waypoint is only declared "reached" after the drone
       stays within threshold for 1.5 s continuously, preventing overshoot bounce.

    4. **Stuck detection**: if the drone has not made more than 1 m progress
       toward the waypoint in 30 s, the APF is temporarily bypassed for 12 s
       so the drone can break free from a local minimum.

    5. **Continuous re-send**: position command resent every 1.5 s for tracking.
    """
    master = adapter.master
    boot_time = adapter.boot_time

    logging.info(f"[{adapter.drone_id}] 📍 Navigating to: lat={target_lat:.6f}, "
                 f"lon={target_lon:.6f}, alt={target_alt}m")

    deadline = time.time() + timeout
    last_send = 0.0
    avoidance_active = False   # Track whether APF is currently redirecting
    first_close_time = None    # For settle check
    dlat_smooth = 0.0
    dlon_smooth = 0.0

    # ── Stuck detection state ─────────────────────────────────────────────────
    best_dist_seen = None     # Best (closest) distance achieved so far
    best_dist_time = time.time()  # When that best distance was set
    apf_bypass_until = 0.0   # If > now, APF is suppressed to escape local min

    # Send initial waypoint command immediately so ArduPilot starts straight-line nav
    send_position_target(master, boot_time, target_lat, target_lon, target_alt)
    last_send = time.time()

    while time.time() < deadline:
        now = time.time()

        # When path is clear: re-send the direct waypoint every 10s only.
        # ArduPilot's native L1 controller handles straight-line between two
        # points without our interference — frequent resends cause zigzag.
        # When APF avoidance is active: resend every 0.4s (handled below).
        if not avoidance_active and now - last_send >= 10.0:
            send_position_target(master, boot_time, target_lat, target_lon, target_alt)
            last_send = now

        # Drain the socket so messages cache is fresh
        master.recv_match(blocking=False)

        # Abort if the flag was set
        if getattr(adapter, 'abort_mission', False):
            logging.info(f"[{adapter.drone_id}] ⚠️ Mission aborted via flag.")
            return False

        # Check if drone entered LAND mode manually
        hb = master.messages.get('HEARTBEAT')
        if hb:
            mode_str = mavutil.mode_string_v10(hb)
            if 'LAND' in mode_str.upper():
                logging.info(f"[{adapter.drone_id}] ⚠️ Drone is in LAND mode. Aborting.")
                return False

        msg = master.messages.get('GLOBAL_POSITION_INT')
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = max(0.0, msg.relative_alt / 1000.0)
            dist = calculate_distance_meters(current_lat, current_lon,
                                             target_lat, target_lon)
            logging.info(f"[{adapter.drone_id}] Distance to wp: {dist:.1f}m, "
                         f"alt={current_alt:.1f}m")
            adapter.log_status()

            # ── Stuck detection ───────────────────────────────────────────────
            if best_dist_seen is None or dist < best_dist_seen:
                best_dist_seen = dist
                best_dist_time = now

            if (now - best_dist_time > 30.0
                    and dist > threshold * 2
                    and now > apf_bypass_until):
                # No progress in 30 s and still far from waypoint → bypass APF
                apf_bypass_until = now + 12.0
                best_dist_time = now  # reset so it doesn't trigger every tick
                logging.warning(
                    f"[{adapter.drone_id}] ⚠️ Stuck detected (dist={dist:.1f}m, "
                    f"no progress in 30s) — bypassing APF for 12s"
                )
            # ─────────────────────────────────────────────────────────────────

            # ── Goal-biased APF obstacle avoidance ───────────────────────────
            # Disabled when within 2.5× threshold (trust ArduPilot near wp)
            # or when stuck bypass is active (let drone break free freely).
            apf_active = (dist > threshold * 2.5) and (now >= apf_bypass_until)
            effective_lat, effective_lon = target_lat, target_lon

            if apf_active:
                try:
                    dlat, dlon = obstacle_map.get_avoidance_vector(
                        current_lat, current_lon, current_alt,
                        goal_lat=target_lat, goal_lon=target_lon
                    )
                    # Instant avoidance onset when approaching obstacle; smooth decay when leaving
                    if abs(dlat) > abs(dlat_smooth):
                        dlat_smooth = dlat
                    else:
                        dlat_smooth += 0.3 * (dlat - dlat_smooth)

                    if abs(dlon) > abs(dlon_smooth):
                        dlon_smooth = dlon
                    else:
                        dlon_smooth += 0.3 * (dlon - dlon_smooth)

                    # Reset small residual noise to 0 so clear paths stay 100% straight
                    if abs(dlat_smooth) < 1e-7: dlat_smooth = 0.0
                    if abs(dlon_smooth) < 1e-7: dlon_smooth = 0.0

                    if abs(dlat_smooth) > 1e-7 or abs(dlon_smooth) > 1e-7:
                        # Obstacle is actively deflecting — project lookahead 20m ahead
                        if dist > 20.0:
                            ratio = 20.0 / dist
                            lookahead_lat = current_lat + (target_lat - current_lat) * ratio
                            lookahead_lon = current_lon + (target_lon - current_lon) * ratio
                        else:
                            lookahead_lat = target_lat
                            lookahead_lon = target_lon

                        effective_lat = lookahead_lat + dlat_smooth
                        effective_lon = lookahead_lon + dlon_smooth

                        # Rapid resend (0.4s) ONLY while actively avoiding
                        if now - last_send >= 0.4:
                            send_position_target(
                                master, boot_time,
                                effective_lat, effective_lon, target_alt
                            )
                            last_send = now
                        avoidance_active = True
                        logging.warning(
                            f"[{adapter.drone_id}] AVOIDANCE: "
                            f"dlat={dlat*111320:.2f}m dlon={dlon*111320:.2f}m"
                        )
                    else:
                        # APF returned zero — path is clear, resume direct nav
                        if avoidance_active:
                            # Snap back to direct waypoint immediately
                            send_position_target(master, boot_time, target_lat, target_lon, target_alt)
                            last_send = now
                            avoidance_active = False
                except Exception as e:
                    logging.error(f"[{adapter.drone_id}] APF error (ignored): {e}")
            else:
                # APF not active (near waypoint or bypass) — ensure direct target
                if avoidance_active:
                    send_position_target(master, boot_time, target_lat, target_lon, target_alt)
                    last_send = now
                    avoidance_active = False
            # ─────────────────────────────────────────────────────────────────

            # ── Waypoint reached + settle check ──────────────────────────────
            if dist < threshold:
                if first_close_time is None:
                    first_close_time = now
                    logging.info(f"[{adapter.drone_id}] Close ({dist:.1f}m) — settling...")
                if now - first_close_time >= 1.5:
                    logging.info(
                        f"[{adapter.drone_id}] ✅ WAYPOINT REACHED "
                        f"(dist={dist:.1f}m, settled 1.5s) — pushing to GCS console"
                    )
                    time.sleep(0.3)
                    return True
            else:
                first_close_time = None  # reset if we drift back out
            # ─────────────────────────────────────────────────────────────────

        time.sleep(0.2)

    logging.warning(f"[{adapter.drone_id}] ⚠️ Waypoint timeout after {timeout}s")
    return False


def land_drone(master):
    """Send land command by changing mode to LAND, retrying up to 10 s."""
    try:
        logging.info("🛬 Initiating landing...")
        mode_id = master.mode_mapping().get('LAND')
        if mode_id is None:
            logging.error("LAND mode not supported by this vehicle")
            return False

        deadline = time.time() + 10
        while time.time() < deadline:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
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
                logging.info(f"🛬 LANDING — mode={mode_str}")
                if 'LAND' in mode_str.upper():
                    logging.info("✅ LAND mode confirmed — descending to ground")
                    return True

        logging.warning("⚠️ Land command sent but mode not confirmed as LAND")
        return True
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
