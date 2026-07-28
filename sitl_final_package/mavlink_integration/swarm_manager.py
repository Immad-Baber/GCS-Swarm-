# swarm_manager.py
# ─────────────────────────────────────────────────────────────────────────────
# Week 4 – Swarm Manager
# Central orchestration layer that holds references to every connected
# SITLAdapter and exposes swarm-wide and individual drone commands.
# ─────────────────────────────────────────────────────────────────────────────

import logging
import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from sitl_adapter import SITLAdapter


class SwarmManager:
    """
    Manages a fleet of drones.

    Usage:
        sm = SwarmManager()
        sm.add_drone("drone_1", "udpin:0.0.0.0:14551")
        sm.add_drone("drone_2", "udpin:0.0.0.0:14552")
        sm.add_drone("drone_3", "udpin:0.0.0.0:14553")
        sm.arm_all()
        sm.takeoff_all(10)
        sm.land_all()
    """

    def __init__(self):
        # {drone_id: SITLAdapter}
        self.drones = {}
        # Lock for thread-safe access to the drones dict
        self._lock = threading.Lock()
        logging.info("[SwarmManager] Initialized (empty fleet)")

    # ── Connection ────────────────────────────────────────────────────────

    def add_drone(self, drone_id: str, connection_str: str) -> bool:
        """
        Connect and initialize a single drone.
        Returns True on success, False on failure.
        """
        try:
            with self._lock:
                old_adapter = self.drones.get(drone_id)
                if old_adapter:
                    old_adapter.abort_mission = True

            logging.info(f"[SwarmManager] Connecting {drone_id} via {connection_str} ...")
            adapter = SITLAdapter(drone_id, connection_str)
            adapter.initialize()
            with self._lock:
                self.drones[drone_id] = adapter
            logging.info(f"[SwarmManager] ✅ {drone_id} connected and initialized")
            return True
        except Exception as e:
            logging.error(f"[SwarmManager] ❌ Failed to add {drone_id}: {e}")
            return False

    def connect_swarm(self, num_drones: int = 3) -> dict:
        """
        Connect to *num_drones* SITL instances on consecutive UDP ports
        starting at 14551.  Returns a summary dict.
        """
        results = {}
        # Connect drones concurrently with a thread pool
        with ThreadPoolExecutor(max_workers=num_drones) as pool:
            futures = {}
            for i in range(num_drones):
                drone_id = f"drone_{i + 1}"
                port = 14551 + i
                connection_str = f"udpin:0.0.0.0:{port}"
                futures[pool.submit(self.add_drone, drone_id, connection_str)] = drone_id

            for future in as_completed(futures):
                drone_id = futures[future]
                try:
                    results[drone_id] = future.result()
                except Exception as e:
                    logging.error(f"[SwarmManager] ❌ {drone_id} connection thread failed: {e}")
                    results[drone_id] = False

        logging.info(f"[SwarmManager] connect_swarm results: {results}")
        return results

    # ── Swarm-wide commands ───────────────────────────────────────────────

    def _run_on_all(self, fn_name: str, *args, **kwargs) -> dict:
        """
        Execute a method on every adapter concurrently.
        *fn_name* is a string attribute name on SITLAdapter.
        Returns {drone_id: True/False}.
        """
        results = {}
        with self._lock:
            drone_items = list(self.drones.items())

        def _run(drone_id, adapter):
            try:
                fn = getattr(adapter, fn_name)
                return fn(*args, **kwargs)
            except Exception as e:
                logging.error(f"[SwarmManager] {fn_name} failed for {drone_id}: {e}")
                return False

        with ThreadPoolExecutor(max_workers=len(drone_items) or 1) as pool:
            futures = {
                pool.submit(_run, did, adp): did for did, adp in drone_items
            }
            for future in as_completed(futures):
                did = futures[future]
                try:
                    results[did] = future.result()
                except Exception as e:
                    results[did] = False
                    logging.error(f"[SwarmManager] {fn_name} thread error for {did}: {e}")

        logging.info(f"[SwarmManager] {fn_name} → {results}")
        return results

    def arm_all(self) -> dict:
        """
        Set GUIDED mode and arm every connected drone concurrently.
        Returns {drone_id: True/False}.
        """
        results = {}
        with self._lock:
            drone_items = list(self.drones.items())

        def _arm_one(drone_id, adapter):
            try:
                if not adapter.set_mode("GUIDED"):
                    logging.error(f"[SwarmManager] {drone_id} failed to set GUIDED mode")
                    return False
                if not adapter.arm_vehicle():
                    logging.error(f"[SwarmManager] {drone_id} failed to arm")
                    return False
                adapter.log_status()
                logging.info(f"[SwarmManager] ✅ {drone_id} armed")
                return True
            except Exception as e:
                logging.error(f"[SwarmManager] arm failed for {drone_id}: {e}")
                return False

        with ThreadPoolExecutor(max_workers=len(drone_items) or 1) as pool:
            futures = {
                pool.submit(_arm_one, did, adp): did for did, adp in drone_items
            }
            for future in as_completed(futures):
                did = futures[future]
                try:
                    results[did] = future.result()
                except Exception as e:
                    results[did] = False

        logging.info(f"[SwarmManager] arm_all → {results}")
        return results

    def takeoff_all(self, altitude: float = 10.0, mission_file: str = "mission1.json") -> dict:
        """Takeoff every drone and start the mission.

        Formation architecture (true leader-following):
        ────────────────────────────────────────────────
        LEADER  (drone_1): flies mission waypoints via WaypointNavigator with APF avoidance.
        FOLLOWERS (others): track leader's live GPS in real-time (~5 Hz), rotating formation
                            offsets by leader heading.  In narrow passages or obstacle proximity,
                            followers squeeze inward behind the leader to fit through safely.
        """
        results = {}
        with self._lock:
            drone_items = list(self.drones.items())

        import os
        import math
        import time
        import threading
        from waypoint_navigator import WaypointNavigator
        from obstacle_map import obstacle_map, DynamicObstacle
        from drone_controller import send_position_target, calculate_distance_meters
        from formation_manager import rotate_offset, LAT_DEG_PER_METER, lon_deg_per_meter
        from pymavlink import mavutil as _mavutil

        SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        WAYPOINT_FILE = os.path.join(SCRIPT_DIR, mission_file)

        temp_navigator = WaypointNavigator(None)
        try:
            base_waypoints = temp_navigator.load_from_json(WAYPOINT_FILE)
        except Exception as e:
            logging.error(f"❌ Failed to load waypoints: {e}")
            base_waypoints = []

        # Determine leader
        all_ids = sorted([did for did, _ in drone_items])
        leader_id = 'drone_1' if any(d == 'drone_1' for d, _ in drone_items) else all_ids[0]
        with self._lock:
            leader_adapter_ref = self.drones.get(leader_id)

        # Body-frame formation offsets: (dx_body [right+], dy_body [fwd+]) in meters
        OFFSETS_BODY = {
            0: (  0.0,   0.0),   # Leader
            1: (-12.0, -12.0),   # Left-1 (12m left, 12m behind)
            2: ( 12.0, -12.0),   # Right-1 (12m right, 12m behind)
            3: (-24.0, -24.0),   # Left-2
            4: ( 24.0, -24.0),   # Right-2
            5: (  0.0, -24.0),   # Centre-rear
            6: (-36.0, -36.0),
            7: ( 36.0, -36.0),
            8: (-12.0, -36.0),
            9: ( 12.0, -36.0),
        }

        # Shift leader waypoints to leader's initial GPS location
        json_origin_lat = base_waypoints[0]["latitude"]  if base_waypoints else 33.665
        json_origin_lon = base_waypoints[0]["longitude"] if base_waypoints else 73.027

        shift_lat = shift_lon = 0.0
        if leader_adapter_ref:
            lp0 = leader_adapter_ref.master.messages.get('GLOBAL_POSITION_INT')
            if lp0:
                shift_lat = lp0.lat / 1e7 - json_origin_lat
                shift_lon = lp0.lon / 1e7 - json_origin_lon

        leader_waypoints = [
            {
                "latitude":  wp["latitude"]  + shift_lat,
                "longitude": wp["longitude"] + shift_lon,
                "altitude":  altitude,
            }
            for wp in base_waypoints
        ]

        leader_started_event = threading.Event()
        obstacle_map.clear_drone_obstacles()

        # ── Worker: LEADER flies waypoints ───────────────────────────────────
        def _leader_worker(drone_id, adapter, waypoints):
            logging.info(f"[{drone_id}] 👑 LEADER: starting mission ({len(waypoints)} wps)")
            leader_started_event.set()
            navigator = WaypointNavigator(adapter)
            if not navigator.execute(waypoints):
                logging.error(f"[{drone_id}] ❌ Leader navigation failed/partial")
            if getattr(adapter, 'abort_mission', False):
                logging.info(f"[{drone_id}] 🛑 Leader aborting.")
                return
            try:
                adapter.land(wait_for_land=True)
                adapter.log_status()
                logging.info(f"[{drone_id}] ✅ Leader landed")
            except Exception as e:
                logging.error(f"[{drone_id}] Leader land error: {e}")

        # ── Worker: FOLLOWER dynamic tracking ────────────────────────────────
        def _follower_worker(drone_id, adapter):
            """Follower continuously tracks leader GPS + rotated offset."""
            try:
                didx = int(drone_id.split('_')[1]) - 1
            except Exception:
                didx = 0
            dx_nominal, dy_nominal = OFFSETS_BODY.get(didx, (-12.0, -12.0))

            logging.info(f"[{drone_id}] ⏳ Awaiting leader signal...")
            leader_started_event.wait(timeout=20)
            time.sleep(2.0)   # Give leader a slight head-start
            logging.info(f"[{drone_id}] 🚁 Following leader in formation...")

            last_send = 0.0
            last_log  = 0.0
            prev_l_lat = None
            prev_l_lon = None
            smoothed_heading = None
            smoothed_target_lat = None
            smoothed_target_lon = None
            last_sent_lat = None   # Last commanded position (for deadband check)
            last_sent_lon = None

            while not getattr(adapter, 'abort_mission', False):
                now = time.time()

                if leader_adapter_ref is None:
                    break

                lp = leader_adapter_ref.master.messages.get('GLOBAL_POSITION_INT')
                if lp is None:
                    time.sleep(0.2)
                    continue

                leader_lat = lp.lat / 1e7
                leader_lon = lp.lon / 1e7
                leader_alt = max(0.0, lp.relative_alt / 1000.0)

                # Check if leader is landing, landed, or disarmed
                hb_l = leader_adapter_ref.master.messages.get('HEARTBEAT')
                if hb_l:
                    armed_l = bool(hb_l.base_mode & _mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode_l  = _mavutil.mode_string_v10(hb_l)
                    if 'LAND' in mode_l.upper() or not armed_l or leader_alt < 1.5:
                        logging.info(
                            f"[{drone_id}] Leader landing/landed "
                            f"(mode={mode_l}, alt={leader_alt:.1f}m) — follower landing now."
                        )
                        break

                # Calculate leader heading from movement direction (Course Over Ground)
                heading = None
                if prev_l_lat is not None:
                    d_lat = (leader_lat - prev_l_lat) * 111320.0
                    d_lon = (leader_lon - prev_l_lon) * 111320.0 * math.cos(math.radians(leader_lat))
                    if math.hypot(d_lat, d_lon) > 0.4:
                        heading = math.degrees(math.atan2(d_lon, d_lat)) % 360.0
                        prev_l_lat, prev_l_lon = leader_lat, leader_lon
                else:
                    prev_l_lat, prev_l_lon = leader_lat, leader_lon

                # Fallback to compass yaw when stationary
                if heading is None and lp.hdg != 65535 and lp.hdg > 0:
                    heading = lp.hdg / 100.0

                # Smooth heading with low-pass filter (alpha = 0.85) to eliminate heading noise while tracking sharp turns
                if heading is not None:
                    if smoothed_heading is None:
                        smoothed_heading = heading
                    else:
                        diff = (heading - smoothed_heading + 180) % 360 - 180
                        smoothed_heading = (smoothed_heading + 0.85 * diff) % 360
                elif smoothed_heading is None:
                    smoothed_heading = 0.0

                # Read follower's current position
                my_pos = adapter.master.messages.get('GLOBAL_POSITION_INT')
                my_lat = my_pos.lat / 1e7 if my_pos else leader_lat
                my_lon = my_pos.lon / 1e7 if my_pos else leader_lon
                my_alt = max(0.0, my_pos.relative_alt / 1000.0) if my_pos else altitude

                # Obstacle proximity check for follower (squeezing in narrow passages)
                dx_eff = dx_nominal
                try:
                    dlat_obs, dlon_obs = obstacle_map.get_avoidance_vector(
                        my_lat, my_lon, my_alt,
                        goal_lat=leader_lat, goal_lon=leader_lon
                    )
                    # If obstacles are nearby, squeeze formation laterally behind leader
                    if abs(dlat_obs) > 1e-6 or abs(dlon_obs) > 1e-6:
                        dx_eff = dx_nominal * 0.25  # collapse laterally 75% behind leader
                except Exception:
                    dlat_obs, dlon_obs = 0.0, 0.0

                # Compute formation target position (direct parallel translation, zero lag)
                rot_east, rot_north = rotate_offset(dx_eff, dy_nominal, smoothed_heading)
                target_lat = leader_lat + rot_north * LAT_DEG_PER_METER
                target_lon = leader_lon + rot_east * lon_deg_per_meter(leader_lat)

                # Add local APF displacement if obstacle repels follower
                eff_target_lat = target_lat + dlat_obs
                eff_target_lon = target_lon + dlon_obs

                if now - last_send >= 0.4:
                    # Deadband: only send if follower is > 1.0m from target or obstacle APF active
                    d_to_target = calculate_distance_meters(
                        my_lat, my_lon,
                        eff_target_lat, eff_target_lon
                    )
                    moved = (d_to_target > 1.0) or (abs(dlat_obs) > 1e-6 or abs(dlon_obs) > 1e-6)

                    if moved:
                        adapter.master.recv_match(blocking=False)
                        send_position_target(
                            adapter.master, adapter.boot_time,
                            eff_target_lat, eff_target_lon, altitude
                        )
                        last_sent_lat = eff_target_lat
                        last_sent_lon = eff_target_lon
                        last_send = now

                    if now - last_log >= 2.5:
                        adapter.log_status()
                        last_log = now

                time.sleep(0.2)

            # Follower landing
            try:
                adapter.land(wait_for_land=True)
                adapter.log_status()
                logging.info(f"[{drone_id}] ✅ Follower landed")
            except Exception as e:
                logging.error(f"[{drone_id}] Follower land error: {e}")

        # ── Per-drone takeoff launcher ───────────────────────────────────────
        def _takeoff_one(drone_id, adapter):
            is_leader = (drone_id == leader_id)
            try:
                adapter.abort_mission = True
                time.sleep(0.5)
                adapter.abort_mission = False

                if not adapter.set_mode("GUIDED"):
                    logging.error(f"[{drone_id}] Failed GUIDED mode")
                    return False
                if not adapter.arm_vehicle():
                    logging.error(f"[{drone_id}] Failed to arm")
                    return False

                ok = adapter.takeoff(altitude)
                adapter.log_status()
                logging.info(
                    f"[SwarmManager] {'✅' if ok else '❌'} {drone_id} "
                    f"takeoff({'ok' if ok else 'fail'})"
                )

                if ok:
                    # Register drone as dynamic obstacle for inter-drone APF
                    pos = adapter.master.messages.get('GLOBAL_POSITION_INT')
                    if pos:
                        dobs = DynamicObstacle(
                            lat=pos.lat / 1e7,
                            lon=pos.lon / 1e7,
                            alt_m=altitude,
                            radius_m=8.0,
                            label=f"drone_{drone_id}"
                        )
                        obstacle_map.add_drone_obstacle(dobs, adapter)

                    if is_leader:
                        threading.Thread(
                            target=_leader_worker,
                            args=(drone_id, adapter, leader_waypoints),
                            daemon=True
                        ).start()
                    else:
                        threading.Thread(
                            target=_follower_worker,
                            args=(drone_id, adapter),
                            daemon=True
                        ).start()

                return ok
            except Exception as e:
                logging.error(f"[SwarmManager] takeoff failed for {drone_id}: {e}")
                return False

        with ThreadPoolExecutor(max_workers=len(drone_items) or 1) as pool:
            futures = {
                pool.submit(_takeoff_one, did, adp): did
                for did, adp in drone_items
            }
            for future in as_completed(futures):
                did = futures[future]
                try:
                    results[did] = future.result()
                except Exception:
                    results[did] = False

        logging.info(f"[SwarmManager] takeoff_all({altitude}m) → {results}")
        return results

    def land_all(self) -> dict:
        """Land every connected drone."""
        results = {}
        with self._lock:
            drone_items = list(self.drones.items())

        def _land_one(drone_id, adapter):
            try:
                adapter.land(wait_for_land=False)
                logging.info(f"[SwarmManager] ✅ {drone_id} land command sent")
                return True
            except Exception as e:
                logging.error(f"[SwarmManager] land failed for {drone_id}: {e}")
                return False

        with ThreadPoolExecutor(max_workers=len(drone_items) or 1) as pool:
            futures = {
                pool.submit(_land_one, did, adp): did for did, adp in drone_items
            }
            for future in as_completed(futures):
                did = futures[future]
                try:
                    results[did] = future.result()
                except Exception as e:
                    results[did] = False

        logging.info(f"[SwarmManager] land_all → {results}")
        return results

    # ── Individual drone commands ─────────────────────────────────────────

    def get_adapter(self, drone_id: str) -> SITLAdapter:
        """Return the SITLAdapter for *drone_id*, or None."""
        with self._lock:
            return self.drones.get(drone_id)

    def arm_drone(self, drone_id: str) -> bool:
        """Arm a single drone by ID."""
        adapter = self.get_adapter(drone_id)
        if adapter is None:
            logging.error(f"[SwarmManager] arm_drone: {drone_id} not found")
            return False
        try:
            if not adapter.set_mode("GUIDED"):
                return False
            if not adapter.arm_vehicle():
                return False
            adapter.log_status()
            logging.info(f"[SwarmManager] ✅ {drone_id} armed (individual)")
            return True
        except Exception as e:
            logging.error(f"[SwarmManager] arm_drone {drone_id} error: {e}")
            return False

    def takeoff_drone(self, drone_id: str, altitude: float = 10.0, mission_file: str = "mission1.json") -> bool:
        """Takeoff a single drone by ID and start mission."""
        adapter = self.get_adapter(drone_id)
        if adapter is None:
            logging.error(f"[SwarmManager] takeoff_drone: {drone_id} not found")
            return False

        import threading
        import os
        import math
        from waypoint_navigator import WaypointNavigator

        SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        WAYPOINT_FILE = os.path.join(SCRIPT_DIR, mission_file)

        temp_navigator = WaypointNavigator(None)
        try:
            base_waypoints = temp_navigator.load_from_json(WAYPOINT_FILE)
        except Exception as e:
            logging.error(f"❌ Failed to load waypoints: {e}")
            base_waypoints = []

        OFFSETS = {
            0: (0,    0),
            1: (-25, -10),
            2: (25,  -10),
            3: (-50, -20),
            4: (50,  -20),
            5: (0,   -20),
            6: (-75, -30),
            7: (75,  -30),
            8: (-25, -30),
            9: (25,  -30),
        }

        def _mission_worker(d_id, adp, waypoints):
            logging.info(f"[{d_id}] Starting waypoint navigation...")
            navigator = WaypointNavigator(adp)
            if not navigator.execute(waypoints):
                logging.error(f"[{d_id}] ❌ Waypoint navigation failed")
            if getattr(adp, 'abort_mission', False):
                logging.info(f"[{d_id}] 🛑 Thread aborting without landing.")
                return

            try:
                adp.land(wait_for_land=True)
                adp.log_status()
                logging.info(f"[{d_id}] ✅ Landed and disarmed")
            except Exception as e:
                logging.error(f"[{d_id}] Land error: {e}")

        try:
            # Cancel any existing mission
            adapter.abort_mission = True
            import time
            time.sleep(0.5)
            adapter.abort_mission = False

            # Ensure the drone is in GUIDED mode and armed before takeoff
            if not adapter.set_mode("GUIDED"):
                logging.error(f"[{drone_id}] Failed to set GUIDED mode for takeoff")
                return False
            if not adapter.arm_vehicle():
                logging.error(f"[{drone_id}] Failed to arm for takeoff")
                return False

            ok = adapter.takeoff(altitude)
            adapter.log_status()
            logging.info(f"[SwarmManager] {'✅' if ok else '❌'} {drone_id} takeoff({altitude}m) individual")
            
            if ok and base_waypoints:
                try:
                    drone_idx = int(drone_id.split('_')[1]) - 1
                except:
                    drone_idx = 0

                json_origin_lat = base_waypoints[0]["latitude"]
                json_origin_lon = base_waypoints[0]["longitude"]

                lat_deg_per_meter = 1.0 / 111320.0
                lon_deg_per_meter = 1.0 / (111320.0 * math.cos(math.radians(json_origin_lat)))

                # Use LEADER's current GPS as the origin reference
                with self._lock:
                    leader_adapter = self.drones.get('drone_1') or adapter
                lp = leader_adapter.master.messages.get('GLOBAL_POSITION_INT')
                if lp:
                    leader_lat = lp.lat / 1e7
                    leader_lon = lp.lon / 1e7
                else:
                    # Fallback: use drone's own position
                    cp = adapter.master.messages.get('GLOBAL_POSITION_INT')
                    leader_lat = cp.lat / 1e7 if cp else json_origin_lat
                    leader_lon = cp.lon / 1e7 if cp else json_origin_lon

                shift_lat = leader_lat - json_origin_lat
                shift_lon = leader_lon - json_origin_lon

                last_wp = base_waypoints[-1]
                dy_path = last_wp["latitude"] - json_origin_lat
                dx_path = (last_wp["longitude"] - json_origin_lon) * math.cos(math.radians(json_origin_lat))
                bearing = math.atan2(dx_path, dy_path) if (abs(dy_path) > 1e-7 or abs(dx_path) > 1e-7) else 0.0

                dx_body, dy_body = OFFSETS.get(drone_idx, (0, 0))
                dx = dx_body * math.cos(bearing) + dy_body * math.sin(bearing)
                dy = -dx_body * math.sin(bearing) + dy_body * math.cos(bearing)

                formation_lat = dy * lat_deg_per_meter
                formation_lon = dx * lon_deg_per_meter

                drone_waypoints = []
                for wp in base_waypoints:
                    drone_waypoints.append({
                        "latitude":  wp["latitude"]  + shift_lat + formation_lat,
                        "longitude": wp["longitude"] + shift_lon + formation_lon,
                        "altitude":  altitude
                    })

                threading.Thread(target=_mission_worker, args=(drone_id, adapter, drone_waypoints), daemon=True).start()

            return ok
        except Exception as e:
            logging.error(f"[SwarmManager] takeoff_drone {drone_id} error: {e}")
            return False

    def land_drone(self, drone_id: str) -> bool:
        """Land a single drone by ID."""
        adapter = self.get_adapter(drone_id)
        if adapter is None:
            logging.error(f"[SwarmManager] land_drone: {drone_id} not found")
            return False
        try:
            adapter.land(wait_for_land=False)
            logging.info(f"[SwarmManager] ✅ {drone_id} land command sent (individual)")
            return True
        except Exception as e:
            logging.error(f"[SwarmManager] land_drone {drone_id} error: {e}")
            return False

    # ── Status ────────────────────────────────────────────────────────────

    def get_drone_status(self, drone_id: str) -> dict:
        """
        Return the latest cached telemetry for a single drone.
        Returns a dict with position, battery, mode, armed status.
        """
        adapter = self.get_adapter(drone_id)
        if adapter is None:
            return {"error": f"{drone_id} not found"}

        status = {"drone_id": drone_id}
        try:
            # Drain pending messages to refresh cache
            while True:
                m = adapter.master.recv_match(blocking=False)
                if m is None:
                    break

            # Position
            pos = adapter.master.messages.get('GLOBAL_POSITION_INT')
            if pos:
                status["position"] = {
                    "lat": pos.lat / 1e7,
                    "lon": pos.lon / 1e7,
                    "alt": max(0.0, pos.relative_alt / 1000.0),
                }

            # Battery
            sys_status = adapter.master.messages.get('SYS_STATUS')
            if sys_status:
                status["battery"] = {
                    "voltage": sys_status.voltage_battery / 1000.0,
                    "remaining": sys_status.battery_remaining,
                    "current": sys_status.current_battery / 100.0,
                }

            # Mode & armed
            from pymavlink import mavutil
            hb = adapter.master.messages.get('HEARTBEAT')
            if hb:
                status["mode"] = mavutil.mode_string_v10(hb)
                status["armed"] = bool(
                    hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
        except Exception as e:
            status["error"] = str(e)

        return status

    def get_swarm_status(self) -> dict:
        """Return status for every connected drone."""
        with self._lock:
            drone_ids = list(self.drones.keys())
        return {did: self.get_drone_status(did) for did in drone_ids}

    def get_connected_drone_ids(self) -> list:
        """Return a list of all connected drone IDs."""
        with self._lock:
            return list(self.drones.keys())
