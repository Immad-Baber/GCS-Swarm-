#sitl_adapter.py
from mavlink_interface import MAVLinkInterface
from pymavlink import mavutil
import asyncio
import csv
import os
import json
import time
import logging
import threading
from datetime import datetime
import requests

from drone_controller import (
    connect_to_drone,
    set_guided_mode,
    arm_drone,
    takeoff,
    wait_until_position_reached,
    land_drone,
)


class SITLAdapter:
    def __init__(self, drone_id: str, connection_str: str):
        self.drone_id = drone_id
        self.flight_path = []  # stores dicts: {time, lat, lon, alt}
        self.connection_str = connection_str
        self.master = None
        self.boot_time = None
        self.abort_mission = False

        # Rate-limit log_status() to prevent socket starvation.
        # 0.2s interval → up to 5 telemetry pushes/sec per drone so the GCS
        # console tracks state changes within one navigation loop tick.
        self._log_lock = threading.Lock()
        self._last_log_time = 0.0
        self._min_log_interval = 0.2   # seconds — max 5 telemetry POSTs/sec per drone

        # Background keepalive thread — continuously drains MAVLink socket
        # to prevent buffer overflow that causes drone freeze.
        self._keepalive_thread = None
        self._keepalive_running = False

    def initialize(self):
        self.master = connect_to_drone(self.connection_str)
        self.boot_time = time.time()

        def set_msg_interval(msg_id, us_interval=1000000):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                us_interval,
                0, 0, 0, 0, 0
            )

        # Critical messages at 5 Hz (200ms) — needed for real-time GCS console sync
        # Non-critical messages kept at 1 Hz to avoid socket saturation
        MAVLINK_MSGS_5HZ = {
            'GLOBAL_POSITION_INT': 33,   # Position → map updates every 200ms
            'HEARTBEAT': 0,              # Mode/arm status → console updates instantly
            'ATTITUDE': 30,              # Heading → formation rotation sync
        }
        MAVLINK_MSGS_1HZ = {
            'SYS_STATUS': 1,             # Battery — slow-changing, 1 Hz fine
            'RC_CHANNELS_RAW': 35,       # RC signal — slow-changing
            'GPS_RAW_INT': 24            # GPS fix type — slow-changing
        }

        for name, msg_id in MAVLINK_MSGS_5HZ.items():
            set_msg_interval(msg_id, 200000)  # 5 Hz — real-time sync
            logging.info(f"Requested {name} telemetry at 5 Hz")

        for name, msg_id in MAVLINK_MSGS_1HZ.items():
            set_msg_interval(msg_id, 1000000)  # 1 Hz — slow-changing data
            logging.info(f"Requested {name} telemetry at 1 Hz")

        # Configure battery capacity to prevent failsafe during flight
        def set_param(name, val):
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode('utf-8'),
                val,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            logging.info(f"Setting parameter {name} -> {val}")

        set_param("BATT_CAPACITY", 99999.0)
        set_param("BATT_FS_LOW_ACT", 0.0)
        set_param("BATT_FS_CRT_ACT", 0.0)
        set_param("FENCE_ENABLE", 0.0)
        set_param("ARMING_CHECK", 0.0)

        # Start background keepalive to prevent socket buffer overflow
        self._start_keepalive()

    def _start_keepalive(self):
        """
        Start a background daemon thread that continuously drains the MAVLink
        receive buffer. This prevents socket overflow which causes drone threads
        to block indefinitely (the freeze bug).
        """
        if self._keepalive_thread and self._keepalive_thread.is_alive():
            return

        self._keepalive_running = True

        def _keepalive_loop():
            logging.info(f"[{self.drone_id}] 🔄 Keepalive thread started")
            consecutive_errors = 0
            while self._keepalive_running:
                try:
                    # Drain all pending MAVLink messages at low priority (0.1s poll)
                    msg = self.master.recv_match(blocking=True, timeout=0.1)
                    if msg is not None:
                        consecutive_errors = 0
                except Exception as e:
                    consecutive_errors += 1
                    if consecutive_errors % 10 == 0:  # Log every 10 errors
                        logging.error(f"[{self.drone_id}] Keepalive error #{consecutive_errors}: {e}")
                    time.sleep(0.5)
                    if consecutive_errors > 60:
                        logging.error(f"[{self.drone_id}] Too many keepalive errors — stopping keepalive.")
                        break

            logging.info(f"[{self.drone_id}] 🔴 Keepalive thread stopped")

        self._keepalive_thread = threading.Thread(
            target=_keepalive_loop,
            daemon=True,
            name=f"keepalive-{self.drone_id}"
        )
        self._keepalive_thread.start()

    def _stop_keepalive(self):
        """Stop the keepalive thread."""
        self._keepalive_running = False

    def arm_vehicle(self):
        result = arm_drone(self.master)
        if result:
            # Immediately push "ARMED" event to GCS console — no rate-limit bypass needed
            # because _last_log_time reset below forces an instant push
            self._last_log_time = 0.0
            self.log_status(action_label="✅ ARMED")
        return result

    def set_mode(self, mode="GUIDED"):
        return set_guided_mode(self.master)

    def takeoff(self, altitude):
        result = takeoff(self.master, altitude)
        if result:
            self._last_log_time = 0.0
            self.log_status(action_label=f"🚀 AIRBORNE → {altitude}m")
        return result

    def goto_position(self, lat, lon, alt):
        result = wait_until_position_reached(self, lat, lon, alt)
        if result:
            self._last_log_time = 0.0
            self.log_status(action_label=f"📍 WAYPOINT REACHED ({lat:.5f}, {lon:.5f})")
        return result

    def land(self, wait_for_land=True):
        land_drone(self.master)
        if not wait_for_land:
            return

        # Wait until fully landed and disarmed so telemetry updates GCS UI
        logging.info(f"[{self.drone_id}] ⏳ Waiting for drone to land and disarm...")
        deadline = time.time() + 60  # max 60s to land
        low_alt_counter = 0

        while time.time() < deadline:
            if getattr(self, 'abort_mission', False):
                logging.info(f"[{self.drone_id}] ⚠️ Landing aborted via flag.")
                return

            # Note: keepalive thread drains messages continuously — just read from cache
            hb = self.master.messages.get('HEARTBEAT')
            pos = self.master.messages.get('GLOBAL_POSITION_INT')

            armed = False
            alt = 0.0
            if hb:
                armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if pos:
                alt = max(0.0, pos.relative_alt / 1000.0)

            logging.info(f"[{self.drone_id}] 🛬 Landing... Altitude: {alt:.2f}m, Armed: {armed}")

            # Send updated telemetry to GCS UI
            if pos:
                self.log_status(override_pos=(pos.lat / 1e7, pos.lon / 1e7, alt))
            else:
                self.log_status()

            # Auto-disarm when near ground (<0.8m)
            if alt < 0.8 and armed:
                try:
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        0, 0, 0, 0, 0, 0, 0
                    )
                except Exception:
                    pass

            if alt < 0.5:
                low_alt_counter += 1
                if low_alt_counter >= 2 or not armed:
                    logging.info(f"[{self.drone_id}] ✅ Drone has landed and disarmed successfully!")
                    return
            else:
                low_alt_counter = 0

            time.sleep(1)

        logging.warning(f"[{self.drone_id}] ⚠️ Land timeout (60s) — completing landing.")

    def get_position(self):
        msg = self.master.messages.get('GLOBAL_POSITION_INT')
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            logging.debug(f"[TELEMETRY] Lat: {lat}, Lon: {lon}, Alt: {alt}")
            return lat, lon, alt
        return None

    def log_status(self, override_pos=None, action_label=None):
        """
        Collect and broadcast telemetry for this drone.

        Rate-limited to self._min_log_interval seconds to prevent:
        - Multiple threads calling simultaneously and starving the socket
        - Flooding the server with too many HTTP POSTs

        Parameters
        ----------
        override_pos : tuple (lat, lon, alt), optional
            Force-override position values (used during landing)
        action_label : str, optional
            If provided, adds a label to the log entry identifying what
            action just occurred, for console/terminal synchronization.
        """
        now = time.time()

        # Non-blocking rate-limit check (acquire with timeout=0 is non-blocking)
        if not self._log_lock.acquire(timeout=0.05):
            # Another thread is already logging for this drone — skip
            return
        try:
            # Rate-limit: don't post more often than _min_log_interval
            if now - self._last_log_time < self._min_log_interval:
                return
            self._last_log_time = now

            telemetry_data = {}

            # Battery Status
            msg = self.master.messages.get('SYS_STATUS')
            if msg:
                voltage = msg.voltage_battery / 1000.0
                remaining = msg.battery_remaining
                current = msg.current_battery / 100.0
                logging.info(f"[{self.drone_id}] 🔋 Battery: {remaining}% ({voltage:.2f}V, {current:.1f}A)")
                telemetry_data.update({
                    "battery": {
                        "voltage": voltage,
                        "remaining": remaining,
                        "current": current
                    }
                })

            # Mode and Arm Status
            hb = self.master.messages.get('HEARTBEAT')
            if hb:
                mode_id = hb.custom_mode
                mode_str = mavutil.mode_string_v10(hb)
                armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                logging.info(f"[{self.drone_id}] 🚁 Mode: {mode_str} (ID: {mode_id}) | Armed: {armed}")
                telemetry_data.update({
                    "mode": mode_str,
                    "armed": armed
                })

            # Attitude / Heading from cache
            att = self.master.messages.get('ATTITUDE')
            if att:
                import math
                yaw_deg = math.degrees(att.yaw)
                telemetry_data.update({
                    "attitude": {
                        "yaw": yaw_deg
                    }
                })

            # Position Logging
            pos = self.master.messages.get('GLOBAL_POSITION_INT')
            hdg = None
            if pos:
                lat = pos.lat / 1e7
                lon = pos.lon / 1e7
                alt = max(0.0, pos.relative_alt / 1000.0)
                hdg = pos.hdg / 100.0 if pos.hdg != 65535 else 0.0
            elif override_pos:
                lat, lon, alt = override_pos
                alt = max(0.0, alt)
            else:
                lat = lon = alt = None

            if lat is not None:
                timestamp = datetime.utcnow().isoformat()

                self.flight_path.append({
                    "time": timestamp,
                    "lat": lat,
                    "lon": lon,
                    "alt": alt
                })

                logging.info(f"[{self.drone_id}] 📍 Position: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")

                pos_data = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "timestamp": timestamp
                }
                if hdg is not None:
                    pos_data["heading"] = hdg

                telemetry_data.update({
                    "position": pos_data
                })

            # RC Signal Strength (read from cache — keepalive drains socket)
            rc = self.master.messages.get('RC_CHANNELS_RAW')
            if rc:
                rssi = rc.rssi
                signal_percent = round((rssi / 255) * 100)
                logging.info(f"📶 RC Signal Strength: {signal_percent}%")
                telemetry_data.update({
                    "rc_signal": signal_percent
                })

            # Attitude (read from cache)
            att2 = self.master.messages.get('ATTITUDE')
            if att2:
                import math
                roll = att2.roll * (180 / math.pi)
                pitch = att2.pitch * (180 / math.pi)
                yaw = att2.yaw * (180 / math.pi)
                logging.info(f"🧭 Attitude: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
                telemetry_data.update({
                    "attitude": {
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw
                    }
                })

            # Include action label if provided (for console timing correlation)
            if action_label:
                telemetry_data["action"] = action_label

            # Emit telemetry via a fire-and-forget background thread so
            # log_status() returns IMMEDIATELY without blocking the navigation loop.
            # Previously this was a blocking requests.POST (up to 1s timeout) which
            # caused the follower tracking loop and waypoint navigator to freeze
            # momentarily, delaying console updates by up to 1s per call.
            if telemetry_data:
                telemetry_data["drone_id"] = self.drone_id
                _payload = dict(telemetry_data)  # snapshot to avoid race

                def _fire_post(payload=_payload):
                    try:
                        requests.post(
                            "http://127.0.0.1:5000/send_telemetry",
                            json=payload,
                            timeout=2
                        )
                    except Exception:
                        pass  # Non-fatal — telemetry is best-effort

                t = threading.Thread(target=_fire_post, daemon=True)
                t.start()
        finally:
            self._log_lock.release()

    def export_flight_path(self):
        folder = os.path.abspath(os.path.join(os.getcwd(), os.pardir, "logs"))
        os.makedirs(folder, exist_ok=True)

        # CSV
        csv_file = os.path.join(folder, f"{self.drone_id}_flight_path.csv")
        with open(csv_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["time", "lat", "lon", "alt"])
            writer.writeheader()
            writer.writerows(self.flight_path)

        # GeoJSON
        geojson_file = os.path.join(folder, f"{self.drone_id}_flight_path.geojson")
        geojson = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [p["lon"], p["lat"], p["alt"]] for p in self.flight_path
                ],
            },
            "properties": {},
        }
        with open(geojson_file, "w") as f:
            json.dump(geojson, f, indent=2)

        logging.info(f"✈️ Flight path saved → {csv_file}, {geojson_file}")