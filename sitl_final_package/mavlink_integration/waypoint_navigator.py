import json
import logging
import os

class WaypointNavigator:
    def __init__(self, adapter):
        self.adapter = adapter

    def load_from_json(self, filepath):
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Waypoint file not found: {filepath}")
        
        with open(filepath, "r") as f:
            data = json.load(f)

        if "waypoints" not in data:
            raise ValueError("JSON must contain a 'waypoints' key.")
        
        waypoints = data["waypoints"]
        if not isinstance(waypoints, list):
            raise ValueError("'waypoints' must be a list.")

        # Validate each waypoint has lat/lon/alt
        for i, wp in enumerate(waypoints):
            if not all(k in wp for k in ("latitude", "longitude", "altitude")):
                raise ValueError(f"Waypoint {i} missing required keys.")

        return waypoints

    def execute(self, waypoints):
        if not waypoints:
            logging.error("❌ No waypoints to navigate.")
            return False

        for idx, wp in enumerate(waypoints):
            lat = wp["latitude"]
            lon = wp["longitude"]
            alt = wp["altitude"]
            logging.info(f"➡️ Navigating to waypoint {idx+1}/{len(waypoints)} → lat={lat}, lon={lon}, alt={alt}")
            if not self.adapter.goto_position(lat, lon, alt):
                logging.error(f"❌ Failed at waypoint {idx+1}")
                return False
            self.adapter.log_status()
        
        return True