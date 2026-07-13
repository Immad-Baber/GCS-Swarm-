# mission_uploader.py
from pymavlink import mavutil
import logging


class MissionUploader:
    def __init__(self, mavlink_interface):
        self.mav = mavlink_interface.master

    def upload_mission(self, waypoints):
        count = len(waypoints)

        logging.info(f"Uploading {count} waypoints...")
        self.mav.mav.mission_count_send(
            self.mav.target_system,
            self.mav.target_component,
            count
        )

        for i, wp in enumerate(waypoints):
            lat, lon, alt = wp
            self.mav.mav.mission_item_int_send(
                self.mav.target_system,
                self.mav.target_component,
                i,  # sequence
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # current
                1,  # autocontinue
                0, 0, 0, 0,  # params 1-4
                int(lat * 1e7),
                int(lon * 1e7),
                alt
            )
            logging.debug(f"Sent waypoint {i}: lat={lat}, lon={lon}, alt={alt}")

        ack = self.mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            logging.info("Mission upload successful.")
            return True
        else:
            logging.error("Mission upload failed.")
            return False
