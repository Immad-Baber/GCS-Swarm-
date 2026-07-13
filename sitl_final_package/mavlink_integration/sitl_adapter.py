#sitl_adapter.py 
from mavlink_interface import MAVLinkInterface
from pymavlink import mavutil
import asyncio
import csv
import os
import json
import time
import logging
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

        MAVLINK_MSGS = {
            'SYS_STATUS': 1,
            'HEARTBEAT': 0,
            'GLOBAL_POSITION_INT': 33,
            'ATTITUDE': 30,
            'RC_CHANNELS_RAW': 35,
            'GPS_RAW_INT': 24
        }

        for name, msg_id in MAVLINK_MSGS.items():
            set_msg_interval(msg_id, 1000000)  # 1 Hz
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

    def arm_vehicle(self):
        return arm_drone(self.master)

    def set_mode(self, mode="GUIDED"):
        return set_guided_mode(self.master)

    def takeoff(self, altitude):
        return takeoff(self.master, altitude)

    def goto_position(self, lat, lon, alt):
        return wait_until_position_reached(self, lat, lon, alt)

    def land(self, wait_for_land=True):
        land_drone(self.master)
        if not wait_for_land:
            return

        # Wait until fully landed and disarmed so that main.py stays alive
        # to send disarmed and alt=0 telemetry to the UI!
        logging.info("⏳ Waiting for drone to land and disarm...")
        deadline = time.time() + 120  # max 2 minutes to land
        while time.time() < deadline:
            if getattr(self, 'abort_mission', False):
                logging.info("⚠️ Landing aborted via flag.")
                return
            
            # Drain socket to parse new messages and update master.messages cache
            self.master.recv_match(blocking=False)
            
            hb = self.master.messages.get('HEARTBEAT')
            pos = self.master.messages.get('GLOBAL_POSITION_INT')
            
            armed = False
            alt = 0.0
            if hb:
                armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if pos:
                alt = max(0.0, pos.relative_alt / 1000.0)
            
            logging.info(f"🛬 Landing... Altitude: {alt:.2f}m, Armed: {armed}")
            
            # Send updated telemetry to GCS UI
            if pos:
                self.log_status(override_pos=(pos.lat / 1e7, pos.lon / 1e7, alt))
            else:
                self.log_status()
            
            if not armed and alt < 0.3:
                logging.info("✅ Drone has landed and disarmed successfully!")
                return
            
            time.sleep(1)

        logging.warning("⚠️ Land timeout (120s) — drone may not have fully landed/disarmed")

    def get_position(self):
        msg = self.master.messages.get('GLOBAL_POSITION_INT')
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            logging.debug(f"[TELEMETRY] Lat: {lat}, Lon: {lon}, Alt: {alt}")
            return lat, lon, alt
        return None

    def log_status(self, override_pos=None):
        # Drain all pending messages from the socket to update the master.messages cache
        while True:
            m = self.master.recv_match(blocking=False)
            if m is None:
                break

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

        # RC Signal Strength
        rc = self.master.recv_match(type='RC_CHANNELS_RAW', blocking=False)
        if rc:
            rssi = rc.rssi
            signal_percent = round((rssi / 255) * 100)
            logging.info(f"📶 RC Signal Strength: {signal_percent}%")
            telemetry_data.update({
                "rc_signal": signal_percent
            })

        # Attitude
        att = self.master.recv_match(type='ATTITUDE', blocking=False)
        if att:
            roll = att.roll * (180 / 3.14159)
            pitch = att.pitch * (180 / 3.14159)
            yaw = att.yaw * (180 / 3.14159)
            logging.info(f"🧭 Attitude: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
            telemetry_data.update({
                "attitude": {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                }
            })

        # Emit all collected telemetry data via HTTP POST
        if telemetry_data:
            telemetry_data["drone_id"] = self.drone_id
            try:
                requests.post("http://127.0.0.1:5000/send_telemetry", json=telemetry_data, timeout=1)
            except Exception as e:
                logging.error(f"Failed to post telemetry: {e}")


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