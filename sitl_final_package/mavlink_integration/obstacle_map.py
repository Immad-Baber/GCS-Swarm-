"""
obstacle_map.py
───────────────────────────────────────────────────────────────────────────────
Singleton environment model for the autonomous drone obstacle avoidance system.

Design goals:
  • Zero dependencies beyond stdlib (math, threading, time, logging)
  • Purely additive: if no obstacles are loaded, get_avoidance_vector() returns (0, 0)
  • Thread-safe reads via a single RLock
  • Dynamic obstacles move in a background thread (no asyncio, no extra deps)
  • Smooth Artificial Potential Field forces — no jitter, no zig-zagging

Usage (from drone_controller.py):
    from obstacle_map import obstacle_map
    dlat, dlon = obstacle_map.get_avoidance_vector(current_lat, current_lon, current_alt)
    # Add dlat/dlon to the commanded target position
"""

import math
import time
import threading
import logging

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
    A turbulent air region drones should avoid entirely.

    Parameters
    ----------
    lat, lon    : GPS centre
    radius_m    : affected radius in metres
    strength    : force multiplier (1.0 = same as a 10-m static obstacle)
    label       : human-readable name
    """
    def __init__(self, lat: float, lon: float, radius_m: float,
                 strength: float = 1.0, label: str = "wind"):
        self.lat = lat
        self.lon = lon
        self.radius_m = radius_m
        self.strength = strength
        self.label = label

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
    # Influence distance added to each obstacle's radius before force drops to 0.
    _STATIC_INFLUENCE_M  = 25.0   # metres beyond radius → force starts here
    _DYNAMIC_INFLUENCE_M = 40.0   # larger for moving objects (need more reaction time)
    _WIND_INFLUENCE_M    = 20.0

    # Peak force magnitude in metres (converted to degrees when returned).
    # This is the maximum shift applied to the commanded target per update cycle.
    _STATIC_PEAK_FORCE_M  = 12.0
    _DYNAMIC_PEAK_FORCE_M = 18.0
    _WIND_PEAK_FORCE_M    = 10.0

    # Altitude tolerance: how far above/below the obstacle altitude a drone
    # must be before the obstacle is completely ignored.
    _ALT_BAND_M = 15.0

    def __init__(self):
        self._lock = threading.RLock()
        self._static: list[StaticObstacle]  = []
        self._wind:   list[WindZone]         = []
        self._dynamic: list[DynamicObstacle] = []

        # Background thread that advances dynamic obstacles
        self._running = True
        self._last_tick = time.time()
        self._bg_thread = threading.Thread(
            target=self._update_loop,
            name="ObstacleMap-updater",
            daemon=True
        )
        self._bg_thread.start()
        logging.info("[ObstacleMap] Initialised — background updater running.")

    # ── Obstacle management ───────────────────────────────────────────────────

    def add_static(self, obs: StaticObstacle) -> None:
        with self._lock:
            self._static.append(obs)
        logging.info(f"[ObstacleMap] Added {obs}")

    def add_wind(self, zone: WindZone) -> None:
        with self._lock:
            self._wind.append(zone)
        logging.info(f"[ObstacleMap] Added {zone}")

    def add_dynamic(self, obs: DynamicObstacle) -> None:
        with self._lock:
            self._dynamic.append(obs)
        logging.info(f"[ObstacleMap] Added {obs}")

    def clear(self) -> None:
        with self._lock:
            self._static.clear()
            self._wind.clear()
            self._dynamic.clear()
        logging.info("[ObstacleMap] All obstacles cleared.")

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
                             alt: float) -> tuple[float, float]:
        """
        Compute the total repulsive displacement vector at (lat, lon, alt).

        Returns
        -------
        (dlat_deg, dlon_deg)
            Degrees to ADD to the commanded target position.
            Returns (0.0, 0.0) when no obstacles are loaded or nearby.
        """
        force_lat_m = 0.0  # accumulated force in metres (lat direction)
        force_lon_m = 0.0  # accumulated force in metres (lon direction)

        with self._lock:
            static  = list(self._static)
            wind    = list(self._wind)
            dynamic = list(self._dynamic)

        for obs in static:
            # Skip if drone is above obstacle
            if alt > obs.max_alt_m + self._ALT_BAND_M:
                continue
            fl, fn = self._apf_repulsion(
                lat, lon, obs.lat, obs.lon,
                obs.radius_m, self._STATIC_INFLUENCE_M, self._STATIC_PEAK_FORCE_M
            )
            force_lat_m += fl
            force_lon_m += fn

        for zone in wind:
            # Wind zones affect all altitudes
            fl, fn = self._apf_repulsion(
                lat, lon, zone.lat, zone.lon,
                zone.radius_m, self._WIND_INFLUENCE_M,
                self._WIND_PEAK_FORCE_M * zone.strength
            )
            force_lat_m += fl
            force_lon_m += fn

        for obs in dynamic:
            # Only affect drones flying within ALT_BAND of the dynamic obstacle
            if abs(alt - obs.alt_m) > self._ALT_BAND_M:
                continue
            fl, fn = self._apf_repulsion(
                lat, lon, obs.lat, obs.lon,
                obs.radius_m, self._DYNAMIC_INFLUENCE_M, self._DYNAMIC_PEAK_FORCE_M
            )
            force_lat_m += fl
            force_lon_m += fn

        # Convert from metres to degrees
        dlat = force_lat_m / _METERS_PER_DEG_LAT
        dlon = force_lon_m / _m_per_deg_lon(lat)

        if abs(force_lat_m) > 0.01 or abs(force_lon_m) > 0.01:
            logging.info(
                f"[ObstacleMap] Avoidance force: {force_lat_m:.2f}m N/S, "
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
        peak_force_m: float
    ) -> tuple[float, float]:
        """
        Artificial Potential Field repulsion.

        Returns (force_lat_m, force_lon_m).
        Force direction is the unit vector pointing AWAY from the obstacle.
        Force magnitude is zero outside influence_m, rises smoothly to
        peak_force_m at the obstacle surface, then stays at peak_force_m
        if the drone is inside the obstacle (emergency push-out).

        The smooth falloff eliminates zig-zagging: force changes gradually
        so the drone curves around rather than oscillating.
        """
        avg_lat = (drone_lat + obs_lat) / 2
        dlat_m = (drone_lat - obs_lat) * _METERS_PER_DEG_LAT
        dlon_m = (drone_lon - obs_lon) * _m_per_deg_lon(avg_lat)
        dist_m = math.hypot(dlat_m, dlon_m)

        outer_edge = obs_radius_m + influence_m

        if dist_m >= outer_edge:
            return 0.0, 0.0  # out of range — no force

        if dist_m <= obs_radius_m or dist_m < 1e-6:
            # Inside or exactly on obstacle — apply maximum force
            # Direction: push straight away if distance ≠ 0, else push north
            if dist_m > 1e-6:
                ux = dlat_m / dist_m
                uy = dlon_m / dist_m
            else:
                ux, uy = 1.0, 0.0
            return ux * peak_force_m, uy * peak_force_m

        # Between surface and outer edge: smooth quadratic falloff
        penetration = (outer_edge - dist_m) / influence_m   # 0..1
        magnitude = peak_force_m * (penetration ** 2)        # smooth, no kink

        ux = dlat_m / dist_m
        uy = dlon_m / dist_m
        return ux * magnitude, uy * magnitude

    # ── Background updater ────────────────────────────────────────────────────

    def _update_loop(self):
        while self._running:
            time.sleep(0.1)  # 10 Hz
            now = time.time()
            dt = now - self._last_tick
            self._last_tick = now

            with self._lock:
                for obs in self._dynamic:
                    obs.update(dt)

    def stop(self):
        """Call on shutdown to cleanly stop the background thread."""
        self._running = False


# ── Module-level singleton ────────────────────────────────────────────────────
# Import this anywhere: `from obstacle_map import obstacle_map`

obstacle_map = ObstacleMap()
