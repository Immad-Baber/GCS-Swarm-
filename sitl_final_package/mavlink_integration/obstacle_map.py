"""
obstacle_map.py
───────────────────────────────────────────────────────────────────────────────
Singleton environment model for the autonomous drone obstacle avoidance system.

Design goals:
  • Zero dependencies beyond stdlib (math, threading, time, logging)
  • Purely additive: if no obstacles are loaded, get_avoidance_vector() returns (0, 0)
  • Thread-safe reads via a single RLock
  • Dynamic obstacles move in a background thread (no asyncio, no extra deps)
  • Goal-biased APF: tangent direction chosen so drone escapes TOWARD the goal,
    not in a random direction that causes circular orbiting.
  • Wind zones: lateral DRIFT only (no repulsion) — drones fly through with deviation.

Usage (from drone_controller.py):
    from obstacle_map import obstacle_map
    dlat, dlon = obstacle_map.get_avoidance_vector(cur_lat, cur_lon, cur_alt,
                                                    goal_lat, goal_lon)
    # Add dlat/dlon to the commanded target position
"""

import math
import time
import threading
import logging
import os

# Set up dedicated obstacle file logger
_parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
_logs_dir = os.path.join(_parent_dir, "logs")
os.makedirs(_logs_dir, exist_ok=True)
_obs_file = os.path.join(_logs_dir, "obstacles.log")

obs_logger = logging.getLogger("ObstacleMap")
obs_logger.setLevel(logging.INFO)
_fh = logging.FileHandler(_obs_file, encoding='utf-8')
_fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
obs_logger.addHandler(_fh)
obs_logger.propagate = True

# ── Earth geometry helpers ────────────────────────────────────────────────────

_METERS_PER_DEG_LAT = 111_320.0  # constant


def _m_per_deg_lon(lat_deg: float) -> float:
    return 111_320.0 * math.cos(math.radians(lat_deg))


def _dist_meters(lat1, lon1, lat2, lon2) -> float:
    dlat = (lat2 - lat1) * _METERS_PER_DEG_LAT
    dlon = (lon2 - lon1) * _m_per_deg_lon((lat1 + lat2) / 2)
    return math.hypot(dlat, dlon)


# ── Obstacle data classes ──────────────────────────────────────────────────────

class StaticObstacle:
    """
    A fixed obstacle (building, wall, no-fly zone).

    Parameters
    ----------
    lat, lon   : GPS centre of the obstacle
    radius_m   : horizontal radius in metres
    max_alt_m  : obstacle height in metres (drones above this are unaffected)
    label      : human-readable name shown in logs
    """
    def __init__(self, lat: float, lon: float, radius_m: float,
                 max_alt_m: float = 100.0, label: str = "static"):
        self.lat = lat
        self.lon = lon
        self.radius_m = radius_m
        self.max_alt_m = max_alt_m
        self.label = label

    def __repr__(self):
        return (f"StaticObstacle({self.label!r}, lat={self.lat:.6f}, "
                f"lon={self.lon:.6f}, r={self.radius_m}m, h={self.max_alt_m}m)")


class WindZone:
    """
    A turbulent air region — drones FLY THROUGH with lateral drift deviation.
    Unlike static obstacles, wind zones do NOT repel drones.  Instead they
    apply a small constant lateral push while the drone is inside the zone,
    causing a visible deviation from the planned path that fades once the
    drone exits.

    Parameters
    ----------
    lat, lon    : GPS centre
    radius_m    : affected radius in metres
    strength    : drift multiplier (1.0 = ~4 m lateral push at zone centre)
    label       : human-readable name
    wind_dir_deg: direction wind blows TO (0=N, 90=E, 180=S, 270=W)
    """
    def __init__(self, lat: float, lon: float, radius_m: float,
                 strength: float = 1.0, label: str = "wind",
                 wind_dir_deg: float = 90.0):
        self.lat = lat
        self.lon = lon
        self.radius_m = radius_m
        self.strength = strength
        self.label = label
        self.wind_dir_deg = wind_dir_deg

    def __repr__(self):
        return (f"WindZone({self.label!r}, lat={self.lat:.6f}, "
                f"lon={self.lon:.6f}, r={self.radius_m}m, s={self.strength})")


class DynamicObstacle:
    """
    A moving obstacle (bird, unknown drone, flying object).

    Parameters
    ----------
    lat, lon      : initial GPS position
    alt_m         : altitude at which it flies
    radius_m      : safety radius in metres
    vel_lat_dps   : velocity in latitude  degrees-per-second
    vel_lon_dps   : velocity in longitude degrees-per-second
    label         : human-readable name
    """
    def __init__(self, lat: float, lon: float, alt_m: float,
                 radius_m: float,
                 vel_lat_dps: float = 0.0, vel_lon_dps: float = 0.0,
                 label: str = "dynamic"):
        self.lat = lat
        self.lon = lon
        self.alt_m = alt_m
        self.radius_m = radius_m
        self.vel_lat_dps = vel_lat_dps
        self.vel_lon_dps = vel_lon_dps
        self.label = label

    def update(self, dt: float):
        """Advance position by dt seconds."""
        self.lat += self.vel_lat_dps * dt
        self.lon += self.vel_lon_dps * dt

    def __repr__(self):
        return (f"DynamicObstacle({self.label!r}, lat={self.lat:.6f}, "
                f"lon={self.lon:.6f}, alt={self.alt_m}m, r={self.radius_m}m)")


# ── ObstacleMap singleton ──────────────────────────────────────────────────────

class ObstacleMap:
    """
    Thread-safe environment model.

    All public methods are safe to call from any thread.
    The background update thread moves DynamicObstacles at 10 Hz.
    """

    # ── APF tuning constants ──────────────────────────────────────────────────
    # These values are REDUCED vs previous version to avoid circular orbit traps.
    # The attraction force from ArduPilot's waypoint tracking can now overpower
    # the repulsion, allowing drones to route around obstacles rather than orbit.

    _STATIC_INFLUENCE_M  = 22.0   # metres beyond radius — force starts here
    _DYNAMIC_INFLUENCE_M = 28.0
    # Wind: no influence radius — handled as drift, not repulsion
    _WIND_MAX_DRIFT_M    = 4.0    # max lateral drift when at zone centre

    # Peak force magnitude in metres (the maximum commanded position shift).
    _STATIC_PEAK_FORCE_M  = 18.0
    _DYNAMIC_PEAK_FORCE_M = 18.0

    # Altitude tolerance
    _ALT_BAND_M = 25.0

    # Inter-drone obstacle tuning (smaller than external obstacles)
    _DRONE_INFLUENCE_M   = 20.0
    _DRONE_PEAK_FORCE_M  = 14.0

    def __init__(self):
        self._lock = threading.RLock()
        self._static: list[StaticObstacle]  = []
        self._wind:   list[WindZone]         = []
        self._dynamic: list[DynamicObstacle] = []
        # Drone-to-drone obstacles: list of (DynamicObstacle, adapter) pairs.
        # Kept separate so clear_drone_obstacles() never touches user-added
        # dynamic obstacles (birds, etc.).
        self._drone_obs: list = []   # [(DynamicObstacle, adapter), ...]
        self.ui_log_callback = None

        # Background thread that advances dynamic obstacles
        self._running = True
        self._last_tick = time.time()
        self._bg_thread = threading.Thread(
            target=self._update_loop,
            name="ObstacleMap-updater",
            daemon=True
        )
        self._bg_thread.start()
        self._log("Initialised — background updater running.")

    def _log(self, msg: str):
        """Internal helper to log to file and broadcast to UI."""
        obs_logger.info(msg)
        if self.ui_log_callback:
            self.ui_log_callback("OBS", msg)

    # ── Obstacle management ───────────────────────────────────────────────────

    def add_static(self, obs: StaticObstacle) -> None:
        with self._lock:
            self._static.append(obs)
        self._log(f"Added {obs}")

    def add_wind(self, zone: WindZone) -> None:
        with self._lock:
            self._wind.append(zone)
        self._log(f"Added {zone}")

    def add_dynamic(self, obs: DynamicObstacle) -> None:
        with self._lock:
            self._dynamic.append(obs)
        self._log(f"Added {obs}")

    def add_drone_obstacle(self, obs: DynamicObstacle, adapter) -> None:
        """Register a live drone as an inter-drone obstacle.

        The obstacle position is updated in real-time from the adapter's
        GLOBAL_POSITION_INT telemetry by the background update loop.
        """
        with self._lock:
            self._drone_obs.append((obs, adapter))
        self._log(f"Registered inter-drone obstacle: {obs}")

    def clear_drone_obstacles(self) -> None:
        """Remove all previously registered drone obstacles."""
        with self._lock:
            self._drone_obs.clear()
        self._log("Cleared all inter-drone obstacles.")

    def clear(self) -> None:
        with self._lock:
            self._static.clear()
            self._wind.clear()
            self._dynamic.clear()
        self._log("All obstacles cleared.")

    def snapshot(self) -> dict:
        """Return a JSON-serialisable snapshot of the current obstacle state."""
        with self._lock:
            return {
                "static": [
                    {"lat": o.lat, "lon": o.lon, "radius_m": o.radius_m,
                     "max_alt_m": o.max_alt_m, "label": o.label}
                    for o in self._static
                ],
                "wind": [
                    {"lat": o.lat, "lon": o.lon, "radius_m": o.radius_m,
                     "strength": o.strength, "label": o.label}
                    for o in self._wind
                ],
                "dynamic": [
                    {"lat": o.lat, "lon": o.lon, "alt_m": o.alt_m,
                     "radius_m": o.radius_m, "label": o.label}
                    for o in self._dynamic
                ],
            }

    # ── Core: avoidance vector ────────────────────────────────────────────────

    def get_avoidance_vector(self, lat: float, lon: float,
                             alt: float,
                             goal_lat: float = None,
                             goal_lon: float = None) -> tuple:
        """
        Compute the total repulsive/deviation displacement vector at (lat, lon, alt).

        Parameters
        ----------
        lat, lon, alt  : drone's current GPS position
        goal_lat/lon   : the waypoint the drone is flying toward.
                         Providing this enables goal-biased tangent selection
                         which ELIMINATES circular orbit traps around obstacles.

        Returns
        -------
        (dlat_deg, dlon_deg)
            Degrees to ADD to the commanded target position.
            Returns (0.0, 0.0) when no obstacles are nearby.
        """
        force_lat_m = 0.0  # accumulated force in metres (lat direction)
        force_lon_m = 0.0  # accumulated force in metres (lon direction)

        with self._lock:
            static_snap  = list(self._static)
            wind_snap    = list(self._wind)
            dynamic_snap = list(self._dynamic)
            drone_snap   = list(self._drone_obs)

        # ── Static obstacle repulsion ─────────────────────────────────────────
        for obs in static_snap:
            if alt > obs.max_alt_m + self._ALT_BAND_M:
                continue
            fl, fn = self._apf_repulsion(
                lat, lon, obs.lat, obs.lon,
                obs.radius_m, self._STATIC_INFLUENCE_M, self._STATIC_PEAK_FORCE_M,
                goal_lat=goal_lat, goal_lon=goal_lon
            )
            force_lat_m += fl
            force_lon_m += fn

        # ── Wind zones: lateral drift only (NOT repulsion) ────────────────────
        # Drones fly THROUGH wind zones with lateral drift deviation.
        for zone in wind_snap:
            dist = _dist_meters(lat, lon, zone.lat, zone.lon)
            outer_edge = zone.radius_m + 15.0  # 15m influence radius
            if dist >= outer_edge:
                continue  # Outside wind zone — no effect
            # Depth ratio: 1.0 at centre, 0.0 at outer edge
            depth_ratio = 1.0 - (dist / outer_edge)
            drift_m = self._WIND_MAX_DRIFT_M * zone.strength * depth_ratio
            # Wind vector has both lat (N/S) and lon (E/W) components
            wind_deg = getattr(zone, 'wind_dir_deg', 45.0)  # default 45° NE
            wind_rad = math.radians(wind_deg)
            force_lat_m += drift_m * math.cos(wind_rad)   # N/S component
            force_lon_m += drift_m * math.sin(wind_rad)   # E/W component

        # ── Dynamic obstacle repulsion ────────────────────────────────────────
        for obs in dynamic_snap:
            if abs(alt - obs.alt_m) > self._ALT_BAND_M:
                continue
            fl, fn = self._apf_repulsion(
                lat, lon, obs.lat, obs.lon,
                obs.radius_m, self._DYNAMIC_INFLUENCE_M, self._DYNAMIC_PEAK_FORCE_M,
                goal_lat=goal_lat, goal_lon=goal_lon
            )
            force_lat_m += fl
            force_lon_m += fn

        # ── Inter-drone repulsion ─────────────────────────────────────────────
        for dobs, _ in drone_snap:
            # Skip self-repulsion: if obstacle is within 0.5 m it's this drone
            d_self = math.hypot(
                (dobs.lat - lat) * _METERS_PER_DEG_LAT,
                (dobs.lon - lon) * _m_per_deg_lon(lat)
            )
            if d_self < 0.5:
                continue
            if abs(dobs.alt_m - alt) > self._ALT_BAND_M:
                continue
            fl, fn = self._apf_repulsion(
                lat, lon, dobs.lat, dobs.lon,
                dobs.radius_m, self._DRONE_INFLUENCE_M, self._DRONE_PEAK_FORCE_M,
                goal_lat=goal_lat, goal_lon=goal_lon
            )
            force_lat_m += fl
            force_lon_m += fn

        # Convert from metres to degrees
        dlat = force_lat_m / _METERS_PER_DEG_LAT
        dlon = force_lon_m / _m_per_deg_lon(lat)

        if abs(force_lat_m) > 0.01 or abs(force_lon_m) > 0.01:
            self._log(
                f"Avoidance force: {force_lat_m:.2f}m N/S, "
                f"{force_lon_m:.2f}m E/W  ->  dlat={dlat:.7f}, dlon={dlon:.7f}"
            )

        return dlat, dlon

    # ── APF helper ────────────────────────────────────────────────────────────

    @staticmethod
    def _apf_repulsion(
        drone_lat: float, drone_lon: float,
        obs_lat: float,   obs_lon: float,
        obs_radius_m: float,
        influence_m: float,
        peak_force_m: float,
        goal_lat: float = None,
        goal_lon: float = None
    ) -> tuple:
        """
        Goal-biased Artificial Potential Field repulsion.

        The key improvement over a plain radial APF:
        - Two perpendicular tangent directions are available around any obstacle.
        - If goal_lat/lon is supplied, we choose the tangent that leads TOWARD
          the goal side of the obstacle.  This prevents the drone from orbiting
          endlessly — it always curves to the side that lets it continue forward.

        Returns (force_lat_m, force_lon_m).
        """
        avg_lat = (drone_lat + obs_lat) / 2
        dlat_m = (drone_lat - obs_lat) * _METERS_PER_DEG_LAT
        dlon_m = (drone_lon - obs_lon) * _m_per_deg_lon(avg_lat)
        dist_m = math.hypot(dlat_m, dlon_m)

        outer_edge = obs_radius_m + influence_m

        if dist_m >= outer_edge:
            return 0.0, 0.0  # Out of range — no force

        # Radial unit vector pointing AWAY from obstacle
        if dist_m < 1e-6:
            # Drone is exactly on obstacle centre — push north by default
            ux, uy = 1.0, 0.0
        else:
            ux = dlat_m / dist_m   # north component of repulsion
            uy = dlon_m / dist_m   # east  component of repulsion

        # ── Goal-biased tangential selection ──────────────────────────────────
        # Two perpendicular tangent candidates:
        #   t1 = CCW rotation of radial: (-uy, +ux)
        #   t2 = CW  rotation of radial: (+uy, -ux)
        # Choose the one whose dot-product with the (obstacle→goal) vector
        # is positive — i.e., the tangent that curves toward the goal.
        if goal_lat is not None and goal_lon is not None:
            g_dlat = (goal_lat - obs_lat) * _METERS_PER_DEG_LAT
            g_dlon = (goal_lon - obs_lon) * _m_per_deg_lon((obs_lat + goal_lat) / 2)
            g_dist = math.hypot(g_dlat, g_dlon)
            if g_dist > 0.5:
                gx = g_dlat / g_dist  # unit vector from obstacle toward goal (N)
                gy = g_dlon / g_dist  # unit vector from obstacle toward goal (E)
                # dot products of each tangent with goal direction
                dot1 = (-uy) * gx + ux * gy   # CCW tangent · goal_dir
                dot2 =  uy  * gx - ux * gy    # CW  tangent · goal_dir
                if dot1 >= dot2:
                    tx, ty = -uy, ux   # CCW
                else:
                    tx, ty =  uy, -ux  # CW
            else:
                tx, ty = -uy, ux  # default CCW (goal is behind obstacle)
        else:
            tx, ty = -uy, ux  # default CCW

        # Blend: 60% radial repulsion + 50% tangential escape
        # The tangential component is strong enough to steer the drone AROUND
        # the obstacle rather than merely bouncing off it.
        fx = 0.60 * ux + 0.50 * tx
        fy = 0.60 * uy + 0.50 * ty
        norm = math.hypot(fx, fy)
        if norm > 1e-6:
            fx /= norm
            fy /= norm

        # Emergency push-out when inside obstacle boundary
        if dist_m <= obs_radius_m or dist_m < 1e-6:
            return fx * peak_force_m, fy * peak_force_m

        # Smooth linear falloff: full force at surface, zero at outer_edge
        penetration = (outer_edge - dist_m) / influence_m   # 0 .. 1
        magnitude = peak_force_m * penetration

        return fx * magnitude, fy * magnitude

    # ── Background updater ────────────────────────────────────────────────────

    def _update_loop(self):
        while self._running:
            time.sleep(0.1)  # 10 Hz
            now = time.time()
            dt = now - self._last_tick
            self._last_tick = now

            with self._lock:
                # Advance velocity-based dynamic obstacles
                for obs in self._dynamic:
                    obs.update(dt)

                # Update live drone positions from their adapter telemetry
                for dobs, adapter in self._drone_obs:
                    try:
                        msg = adapter.master.messages.get('GLOBAL_POSITION_INT')
                        if msg:
                            dobs.lat = msg.lat / 1e7
                            dobs.lon = msg.lon / 1e7
                            dobs.alt_m = max(0.0, msg.relative_alt / 1000.0)
                    except Exception:
                        pass  # Adapter may not be connected yet

    def stop(self):
        """Call on shutdown to cleanly stop the background thread."""
        self._running = False


# ── Module-level singleton ────────────────────────────────────────────────────
# Import this anywhere: `from obstacle_map import obstacle_map`

obstacle_map = ObstacleMap()
