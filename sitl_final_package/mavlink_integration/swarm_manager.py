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
                connection_str = f"udp:127.0.0.1:{port}"
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

    def takeoff_all(self, altitude: float = 10.0) -> dict:
        """Takeoff every connected drone to *altitude* meters."""
        results = {}
        with self._lock:
            drone_items = list(self.drones.items())

        def _takeoff_one(drone_id, adapter):
            try:
                ok = adapter.takeoff(altitude)
                adapter.log_status()
                logging.info(f"[SwarmManager] {'✅' if ok else '❌'} {drone_id} takeoff({'ok' if ok else 'fail'})")
                return ok
            except Exception as e:
                logging.error(f"[SwarmManager] takeoff failed for {drone_id}: {e}")
                return False

        with ThreadPoolExecutor(max_workers=len(drone_items) or 1) as pool:
            futures = {
                pool.submit(_takeoff_one, did, adp): did for did, adp in drone_items
            }
            for future in as_completed(futures):
                did = futures[future]
                try:
                    results[did] = future.result()
                except Exception as e:
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

    def takeoff_drone(self, drone_id: str, altitude: float = 10.0) -> bool:
        """Takeoff a single drone by ID."""
        adapter = self.get_adapter(drone_id)
        if adapter is None:
            logging.error(f"[SwarmManager] takeoff_drone: {drone_id} not found")
            return False
        try:
            ok = adapter.takeoff(altitude)
            adapter.log_status()
            logging.info(f"[SwarmManager] {'✅' if ok else '❌'} {drone_id} takeoff({altitude}m) individual")
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
