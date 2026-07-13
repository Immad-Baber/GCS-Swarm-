# formation_manager.py
# ─────────────────────────────────────────────────────────────────────────────
# Week 4 – Formation Manager
# Implements leader-follower and fixed-offset formation logic.
# The primary formation is a triangle with drone_1 as leader:
#     drone_1 = leader (origin)
#     drone_2 = 10 meters left / back
#     drone_3 = 10 meters right / back
# ─────────────────────────────────────────────────────────────────────────────

import math
import logging
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from drone_controller import calculate_distance_meters


# ── GPS coordinate helpers ────────────────────────────────────────────────

# Earth radius in meters
EARTH_RADIUS = 6371000.0

# Metres per degree of latitude (constant)
LAT_DEG_PER_METER = 1.0 / 111320.0


def lon_deg_per_meter(lat_deg: float) -> float:
    """
    Returns how many degrees of longitude equal one meter at the
    given latitude.
    """
    return 1.0 / (111320.0 * math.cos(math.radians(lat_deg)))


def offset_position(lat: float, lon: float, dx_meters: float,
                    dy_meters: float) -> tuple:
    """
    Return (new_lat, new_lon) after applying a meter offset.
    dx_meters = east (+) / west (-)
    dy_meters = north (+) / south (-)
    """
    new_lat = lat + dy_meters * LAT_DEG_PER_METER
    new_lon = lon + dx_meters * lon_deg_per_meter(lat)
    return new_lat, new_lon


def rotate_offset(dx: float, dy: float, heading_deg: float) -> tuple:
    """
    Rotate a body-frame offset (dx, dy) by the leader's heading so
    that the formation is always aligned with the direction of flight.

    heading_deg: clockwise from North (0 = North, 90 = East).
    """
    theta = math.radians(heading_deg)
    rotated_dx = dx * math.cos(theta) - dy * math.sin(theta)
    rotated_dy = dx * math.sin(theta) + dy * math.cos(theta)
    return rotated_dx, rotated_dy


# ── Formation definitions ────────────────────────────────────────────────

# Each formation is a dict of {drone_id: (dx_body, dy_body)} in meters
# relative to the leader.  dx = lateral (+right, -left), dy = longitudinal
# (+forward, -back).

FORMATIONS = {
    "triangle": {
        # Leader is at the origin — omitted from the offsets dict.
        # drone_2: 10 metres left and 10 metres back
        "drone_2": (-10.0, -10.0),
        # drone_3: 10 metres right and 10 metres back
        "drone_3": (10.0, -10.0),
    },
}


class FormationManager:
    """
    Computes and commands formation positions for a swarm.

    Usage:
        fm = FormationManager(swarm_manager)
        positions = fm.compute_formation_positions("triangle", spacing=10)
        fm.move_to_formation("triangle", spacing=10)
    """

    LEADER_ID = "drone_1"

    def __init__(self, swarm_manager):
        """
        Parameters
        ----------
        swarm_manager : SwarmManager
            Reference to the active SwarmManager instance so we can read
            leader position and command followers.
        """
        self.swarm_manager = swarm_manager

    # ── Compute target positions ──────────────────────────────────────────

    def compute_formation_positions(
        self,
        formation_type: str = "triangle",
        spacing: float = 10.0,
    ) -> dict:
        """
        Compute the GPS target position for every follower drone based on
        the leader's current position and heading.

        Parameters
        ----------
        formation_type : str
            Name of the formation (must exist in FORMATIONS dict).
        spacing : float
            Scaling factor applied to the formation offset distances.
            With spacing=10 the default triangle offsets are used as-is
            because they are already 10 m.  A spacing of 20 would double
            the distances.

        Returns
        -------
        dict : {drone_id: {"lat": float, "lon": float, "alt": float}}
            Target positions for each follower drone.
        """
        if formation_type not in FORMATIONS:
            raise ValueError(
                f"Unknown formation '{formation_type}'. "
                f"Available: {list(FORMATIONS.keys())}"
            )

        # Get leader position
        leader_status = self.swarm_manager.get_drone_status(self.LEADER_ID)
        if "position" not in leader_status:
            raise RuntimeError("Leader drone has no position data")

        leader_lat = leader_status["position"]["lat"]
        leader_lon = leader_status["position"]["lon"]
        leader_alt = leader_status["position"]["alt"]

        # Get leader heading (default to 0 = North if unavailable)
        leader_heading = 0.0
        leader_adapter = self.swarm_manager.get_adapter(self.LEADER_ID)
        if leader_adapter:
            att = leader_adapter.master.messages.get('ATTITUDE')
            if att:
                leader_heading = math.degrees(att.yaw)
                if leader_heading < 0:
                    leader_heading += 360.0

        logging.info(
            f"[FormationManager] Leader at lat={leader_lat:.6f}, "
            f"lon={leader_lon:.6f}, alt={leader_alt:.1f}m, "
            f"heading={leader_heading:.1f}°"
        )

        # Compute scale factor relative to the default 10m spacing
        scale = spacing / 10.0

        offsets = FORMATIONS[formation_type]
        targets = {}

        # Leader keeps its own position
        targets[self.LEADER_ID] = {
            "lat": leader_lat,
            "lon": leader_lon,
            "alt": leader_alt,
        }

        for drone_id, (dx_body, dy_body) in offsets.items():
            # Scale offsets
            dx_scaled = dx_body * scale
            dy_scaled = dy_body * scale

            # Rotate by leader heading so formation faces direction of flight
            dx_world, dy_world = rotate_offset(dx_scaled, dy_scaled,
                                               leader_heading)

            # Convert to GPS
            target_lat, target_lon = offset_position(
                leader_lat, leader_lon, dx_world, dy_world
            )

            targets[drone_id] = {
                "lat": target_lat,
                "lon": target_lon,
                "alt": leader_alt,  # same altitude as leader
            }

            logging.info(
                f"[FormationManager] {drone_id} target → "
                f"lat={target_lat:.6f}, lon={target_lon:.6f}, "
                f"alt={leader_alt:.1f}m  "
                f"(body offset: dx={dx_scaled:.1f}m, dy={dy_scaled:.1f}m)"
            )

        return targets

    # ── Command drones to formation positions ─────────────────────────────

    def move_to_formation(
        self,
        formation_type: str = "triangle",
        spacing: float = 10.0,
    ) -> dict:
        """
        Compute formation positions and command each follower to fly there.

        Returns
        -------
        dict : {drone_id: True/False}
        """
        targets = self.compute_formation_positions(formation_type, spacing)
        results = {}

        def _goto(drone_id, target):
            adapter = self.swarm_manager.get_adapter(drone_id)
            if adapter is None:
                logging.error(f"[FormationManager] {drone_id} adapter not found")
                return False
            try:
                logging.info(
                    f"[FormationManager] Commanding {drone_id} to "
                    f"lat={target['lat']:.6f}, lon={target['lon']:.6f}, "
                    f"alt={target['alt']:.1f}m"
                )
                adapter.goto_position(target["lat"], target["lon"],
                                      target["alt"])
                adapter.log_status()
                return True
            except Exception as e:
                logging.error(
                    f"[FormationManager] {drone_id} goto failed: {e}")
                return False

        # Move followers concurrently — leader stays put
        follower_targets = {
            did: t for did, t in targets.items() if did != self.LEADER_ID
        }

        with ThreadPoolExecutor(
            max_workers=max(len(follower_targets), 1)
        ) as pool:
            futures = {
                pool.submit(_goto, did, t): did
                for did, t in follower_targets.items()
            }
            for future in as_completed(futures):
                did = futures[future]
                try:
                    results[did] = future.result()
                except Exception as e:
                    results[did] = False

        # Leader is trivially "in position"
        results[self.LEADER_ID] = True

        logging.info(f"[FormationManager] move_to_formation → {results}")
        return results

    # ── Formation distance logging ────────────────────────────────────────

    def log_formation_distances(self) -> dict:
        """
        Compute and log the distance between every pair of drones.
        Returns a dict of {pair_key: distance_meters}.
        """
        drone_ids = self.swarm_manager.get_connected_drone_ids()
        positions = {}

        for did in drone_ids:
            status = self.swarm_manager.get_drone_status(did)
            if "position" in status:
                positions[did] = status["position"]

        distances = {}
        drone_list = list(positions.keys())
        for i in range(len(drone_list)):
            for j in range(i + 1, len(drone_list)):
                a, b = drone_list[i], drone_list[j]
                pa, pb = positions[a], positions[b]
                dist = calculate_distance_meters(
                    pa["lat"], pa["lon"], pb["lat"], pb["lon"]
                )
                key = f"{a}<->{b}"
                distances[key] = round(dist, 2)
                logging.info(
                    f"[FormationManager] Distance {a} ↔ {b}: {dist:.2f}m"
                )

        return distances
