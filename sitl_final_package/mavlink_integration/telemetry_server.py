# telemetry_server.py
# ─────────────────────────────────────────────────────────────────────────────
# GCS Telemetry + Command Server
# Week 4 — Added REST API endpoints for swarm commands, individual drone
# control, and formation management.
# ─────────────────────────────────────────────────────────────────────────────

import asyncio
import json
import threading
import logging
import telemetry_logger
from quart import Quart, websocket, request, make_response
from quart_cors import cors
import time
import datetime

app = Quart(__name__)
app = cors(app, allow_origin="*")

# ── WebSocket broadcast infrastructure ────────────────────────────────────

connected_websockets = set()
broadcast_queue = asyncio.Queue()


@app.websocket("/ws")
async def ws():
    """Accept a WebSocket connection and keep it alive."""
    print("Client connected")
    connected_websockets.add(websocket._get_current_object())
    try:
        while True:
            await websocket.receive()  # Keep alive, ignore content
    except Exception as e:
        print(f"WebSocket connection error: {e}")
    finally:
        connected_websockets.remove(websocket._get_current_object())
        print("Client disconnected")


@app.route("/")
async def index():
    """Serve the GCS dashboard HTML."""
    import os
    html_path = os.path.join(os.path.dirname(__file__), "index.html")
    with open(html_path, "r", encoding="utf-8") as f:
        return await make_response(f.read())


@app.route("/send_telemetry", methods=["POST"])
async def send_telemetry():
    """Receive telemetry from a drone adapter and broadcast via WebSocket.

    If the telemetry payload contains an 'action' field (e.g. 'ARMED',
    'AIRBORNE', 'WAYPOINT_REACHED'), an additional priority action-event
    message is pushed to all WebSocket clients FIRST so the GCS console
    displays it at the exact moment the drone state changed.
    """
    data = await request.get_json()

    # ── Priority action event push ─────────────────────────────────────────
    action = data.get("action") if data else None
    if action:
        import datetime as _dt
        ts = _dt.datetime.utcnow().strftime('%H:%M:%S')
        action_msg = json.dumps({
            "type": "action_event",
            "drone_id": data.get("drone_id", "unknown"),
            "action": action,
            "server_ts": ts,
            # Pass through position so the map icon can update simultaneously
            "position": data.get("position"),
            "mode": data.get("mode"),
            "armed": data.get("armed"),
        })
        n_loop = getattr(app, 'n_loop', None)
        if n_loop and n_loop.is_running():
            asyncio.run_coroutine_threadsafe(broadcast_queue.put(action_msg), n_loop)
        else:
            try:
                await broadcast_queue.put(action_msg)
            except Exception:
                pass
        logging.info(f"[Action] {data.get('drone_id', '?')} → {action}")

    await emit_telemetry(data)
    return {"status": "ok"}


async def broadcast_worker():
    """Background task: pull messages from the queue and send to all clients."""
    while True:
        try:
            message = await broadcast_queue.get()
        except Exception:
            await asyncio.sleep(0.1)
            continue
        if connected_websockets:
            disconnected = set()
            for ws in list(connected_websockets):
                try:
                    await ws.send(message)
                except Exception as e:
                    logging.warning(f"WebSocket send error (removing client): {e}")
                    disconnected.add(ws)
            for ws in disconnected:
                connected_websockets.discard(ws)


async def telemetry_polling_worker():
    """
    Background task: periodically poll all connected drones for telemetry.
    Runs adapter.log_status() in an executor to avoid blocking the event loop.

    Poll interval: 0.25s (4 Hz) so the GCS map/console stays fresh even
    between the 5 Hz MAVLink push cycle.
    """
    while True:
        await asyncio.sleep(0.25)   # was 1s — reduced for real-time display

        def _poll_telemetry():
            for drone_id in swarm_mgr.get_connected_drone_ids():
                adapter = swarm_mgr.get_adapter(drone_id)
                if adapter:
                    try:
                        adapter.log_status()
                    except Exception as e:
                        logging.error(f"[Poll] Telemetry error for {drone_id}: {e}")

        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(None, _poll_telemetry)
        except Exception as e:
            logging.error(f"[Poll] Executor error: {e}")


async def emit_telemetry(data):
    """Push telemetry data into the broadcast queue and log it.

    Called from both async (Quart route) and sync (adapter threads) contexts.
    Uses run_coroutine_threadsafe when called from a thread, direct await otherwise.
    """
    json_message = json.dumps(data)
    n_loop = getattr(app, 'n_loop', None)
    if n_loop and n_loop.is_running():
        # Called from a background thread — schedule coroutine safely
        asyncio.run_coroutine_threadsafe(broadcast_queue.put(json_message), n_loop)
    else:
        # Called from inside the event loop (Quart route)
        try:
            running = asyncio.get_running_loop()
            await broadcast_queue.put(json_message)
        except RuntimeError:
            pass  # No running loop — skip broadcast (server not fully started)
    telemetry_logger.append_log(data.get("drone_id", "unknown"), data)


@app.route("/export_swarm_log", methods=["GET"])
async def export_swarm_log():
    """Export combined swarm telemetry log."""
    loop = asyncio.get_running_loop()
    
    def _export_logs():
        combined_path = telemetry_logger.combine_logs()
        with open(combined_path, "r", encoding="utf-8") as f:
            # Read at most 1MB to prevent JSON bloat/timeout
            return f.read(1024 * 1024)

    try:
        content = await loop.run_in_executor(None, _export_logs)
        return {"status": "ok", "combined_log": content}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ── Fake telemetry (for testing without SITL) ────────────────────────────

telemetry_points = [
    {"lat": 33.665137, "lon": 73.027023},
    {"lat": 33.6660379009009, "lon": 73.027023},
    {"lat": 33.665960013925805, "lon": 73.0274632656695},
    {"lat": 33.66573982036609, "lon": 73.02782740540503},
    {"lat": 33.665415393688626, "lon": 73.02805245613827},
    {"lat": 33.665042830213274, "lon": 73.02809950455287},
    {"lat": 33.66468654954955, "lon": 73.02796041555054},
    {"lat": 33.664408155860926, "lon": 73.02765923888337},
    {"lat": 33.66425578594529, "lon": 73.02724805073322},
    {"lat": 33.66425578594529, "lon": 73.02679794926678},
    {"lat": 33.664408155860926, "lon": 73.02638676111663},
    {"lat": 33.66468654954955, "lon": 73.02608558444946},
    {"lat": 33.665042830213274, "lon": 73.02594649544713},
    {"lat": 33.665415393688626, "lon": 73.02599354386173},
    {"lat": 33.66573982036609, "lon": 73.02621859459497},
    {"lat": 33.67129993962286, "lon": 73.04784421693864},
]

async def generate_fake_telemetry():
    while True:
        for point in telemetry_points:
            data = {
                "battery": {
                    "voltage": 12.6,
                    "remaining": 0,
                    "current": 15.15
                },
                "mode": "GUIDED",
                "armed": "true",
                "position": {
                    "lat": point["lat"],
                    "lon": point["lon"],
                    "alt": 10.083,
                    "timestamp": datetime.datetime.utcnow().isoformat()
                }
            }

            await emit_telemetry(data)
            await asyncio.sleep(1)


# ═══════════════════════════════════════════════════════════════════════════
# WEEK 4 — SWARM COMMAND & CONTROL API
# ═══════════════════════════════════════════════════════════════════════════

from swarm_manager import SwarmManager
from formation_manager import FormationManager
import formation_logger

# Global SwarmManager and FormationManager instances
swarm_mgr = SwarmManager()
formation_mgr = FormationManager(swarm_mgr)


def _run_in_thread(fn, *args, **kwargs):
    """
    Run a blocking function in a background thread and return
    an asyncio Future so the Quart endpoint can await it.
    """
    loop = asyncio.get_event_loop()
    return loop.run_in_executor(None, lambda: fn(*args, **kwargs))


# ── Swarm Connection ──────────────────────────────────────────────────────

@app.route("/api/swarm/connect", methods=["POST"])
async def api_swarm_connect():
    """
    Connect to N SITL drone instances.
    Body: {"num_drones": 3}
    """
    data = await request.get_json(force=True, silent=True) or {}
    num_drones = data.get("num_drones", 3)
    logging.info(f"[API] /api/swarm/connect — num_drones={num_drones}")
    results = await _run_in_thread(swarm_mgr.connect_swarm, int(num_drones))
    # Convert bool values to strings for JSON serialization
    return {"status": "ok", "results": {k: bool(v) for k, v in results.items()}}


# ── Swarm-wide Commands ──────────────────────────────────────────────────

@app.route("/api/swarm/arm_all", methods=["POST"])
async def api_swarm_arm_all():
    """Arm all connected drones."""
    logging.info("[API] /api/swarm/arm_all")
    results = await _run_in_thread(swarm_mgr.arm_all)
    return {"status": "ok", "results": {k: bool(v) for k, v in results.items()}}


@app.route("/api/swarm/takeoff_all", methods=["POST"])
async def api_swarm_takeoff_all():
    """
    Takeoff all connected drones.
    Body: {"altitude": 10}
    """
    data = await request.get_json(force=True, silent=True) or {}
    altitude = float(data.get("altitude", 10))
    mission = data.get("mission", "mission1.json")
    logging.info(f"[API] /api/swarm/takeoff_all — altitude={altitude}, mission={mission}")
    results = await _run_in_thread(swarm_mgr.takeoff_all, altitude, mission)
    return {"status": "ok", "results": {k: bool(v) for k, v in results.items()}}


@app.route("/api/swarm/land_all", methods=["POST"])
async def api_swarm_land_all():
    """Land all connected drones."""
    logging.info("[API] /api/swarm/land_all")
    results = await _run_in_thread(swarm_mgr.land_all)
    return {"status": "ok", "results": {k: bool(v) for k, v in results.items()}}


# ── Automated Test Runner ──────────────────────────────────────────────────

import test_swarm_scenarios
import threading

def log_to_ui(module, message):
    """
    Broadcasting helper to send logs to GCS Dashboard console.
    Safe to call from any thread. Adds a server-side timestamp so the
    GCS console always shows when the event *actually happened*, not
    when the WebSocket message was received by the browser.
    """
    import datetime as _dt
    n_loop = getattr(app, 'n_loop', None)
    if n_loop is not None and n_loop.is_running():
        ts = _dt.datetime.utcnow().strftime('%H:%M:%S')
        data = {"type": "log", "module": module, "message": message, "server_ts": ts}
        try:
            asyncio.run_coroutine_threadsafe(broadcast_queue.put(json.dumps(data)), n_loop)
        except Exception as e:
            logging.error(f"[log_to_ui] Failed to enqueue log: {e}")

@app.route("/api/test/run", methods=["POST"])
async def api_test_run():
    """Run an automated test scenario in the background.

    Body JSON:
      { "test_id": 1-18, "mode": "pass"|"fail", "num_drones": 3 }

    num_drones (default 3, max 10) sets how many SITL instances to connect
    for the test.  The value is injected into the test_swarm_scenarios module
    before the scenario function is called.
    """
    data = await request.get_json(force=True, silent=True) or {}
    test_id = int(data.get("test_id", 1))
    mode = data.get("mode", "pass")      # "pass" or "fail"
    force_fail = (mode == "fail")
    num_drones = int(data.get("num_drones", 3))   # NEW: dynamic drone count
    logging.info(f"[API] /api/test/run — test_id={test_id}, mode={mode}, num_drones={num_drones}")

    def _run_scenario():
        try:
            # Propagate drone count to test module global
            test_swarm_scenarios.NUM_DRONES = num_drones

            fn = getattr(test_swarm_scenarios, f"scenario_{test_id}", None)
            if fn is not None:
                fn(force_fail=force_fail, log_callback=log_to_ui)
            else:
                logging.error(f"Unknown test id: {test_id}")
                log_to_ui(f"TEST-{test_id}", f"❌ Unknown test scenario: {test_id}")
        except Exception as e:
            logging.error(f"Scenario {test_id} error: {e}")
            log_to_ui(f"TEST-{test_id}", f"❌ Scenario error: {e}")

    t = threading.Thread(target=_run_scenario, daemon=True)
    t.start()

    return {
        "status": "ok",
        "message": f"Test {test_id} started in background [{mode.upper()}] with {num_drones} drones"
    }


# ── Individual Drone Commands ─────────────────────────────────────────────

@app.route("/api/drone/<drone_id>/arm", methods=["POST"])
async def api_drone_arm(drone_id):
    """Arm a specific drone by ID."""
    logging.info(f"[API] /api/drone/{drone_id}/arm")
    result = await _run_in_thread(swarm_mgr.arm_drone, drone_id)
    return {"status": "ok", "drone_id": drone_id, "armed": bool(result)}


@app.route("/api/drone/<drone_id>/takeoff", methods=["POST"])
async def api_drone_takeoff(drone_id):
    """
    Takeoff a specific drone.
    Body: {"altitude": 10}
    """
    data = await request.get_json(force=True, silent=True) or {}
    altitude = float(data.get("altitude", 10))
    mission = data.get("mission", "mission1.json")
    mode = data.get("mode", "follow")
    logging.info(f"[API] /api/drone/{drone_id}/takeoff — altitude={altitude}, mission={mission}, mode={mode}")
    result = await _run_in_thread(swarm_mgr.takeoff_drone, drone_id, altitude, mission, mode)
    return {"status": "ok", "drone_id": drone_id, "takeoff": bool(result)}


@app.route("/api/drone/<drone_id>/land", methods=["POST"])
async def api_drone_land(drone_id):
    """Land a specific drone."""
    logging.info(f"[API] /api/drone/{drone_id}/land")
    result = await _run_in_thread(swarm_mgr.land_drone, drone_id)
    return {"status": "ok", "drone_id": drone_id, "landed": bool(result)}


@app.route("/api/drone/<drone_id>/goto", methods=["POST"])
async def api_drone_goto(drone_id):
    """
    Command a specific drone to fly to a GPS position.
    Body: {"lat": float, "lon": float, "alt": float}
    """
    data = await request.get_json(force=True, silent=True) or {}
    lat = float(data.get("lat", 0))
    lon = float(data.get("lon", 0))
    alt = float(data.get("alt", 10))
    logging.info(f"[API] /api/drone/{drone_id}/goto — lat={lat}, lon={lon}, alt={alt}")

    def _goto():
        adapter = swarm_mgr.get_adapter(drone_id)
        if adapter is None:
            return False
        try:
            adapter.goto_position(lat, lon, alt)
            return True
        except Exception as e:
            logging.error(f"[API] goto failed for {drone_id}: {e}")
            return False

    result = await _run_in_thread(_goto)
    return {"status": "ok", "drone_id": drone_id, "goto": bool(result),
            "target": {"lat": lat, "lon": lon, "alt": alt}}


# ── Status ────────────────────────────────────────────────────────────────


@app.route("/api/swarm/status", methods=["GET"])
async def api_swarm_status():
    """Return status of all connected drones."""
    status = swarm_mgr.get_swarm_status()
    connected = swarm_mgr.get_connected_drone_ids()
    return {"status": "ok", "connected_drones": connected, "drones": status}


@app.route("/api/drone/<drone_id>/status", methods=["GET"])
async def api_drone_status(drone_id):
    """Return status of a single drone."""
    status = swarm_mgr.get_drone_status(drone_id)
    return {"status": "ok", "drone": status}


# ── Formation ─────────────────────────────────────────────────────────────

@app.route("/api/swarm/formation", methods=["POST"])
async def api_swarm_formation():
    """
    Move the swarm into a formation.
    Body: {"type": "triangle", "spacing": 10}
    """
    data = await request.get_json(force=True, silent=True) or {}
    formation_type = data.get("type", "triangle")
    spacing = float(data.get("spacing", 10))
    logging.info(
        f"[API] /api/swarm/formation — type={formation_type}, spacing={spacing}"
    )

    def _do_formation():
        # Compute target positions
        targets = formation_mgr.compute_formation_positions(
            formation_type, spacing
        )
        # Move followers to formation
        results = formation_mgr.move_to_formation(formation_type, spacing)
        # Log distances
        distances = formation_mgr.log_formation_distances()
        # Gather actual positions
        actual_positions = {}
        for did in swarm_mgr.get_connected_drone_ids():
            s = swarm_mgr.get_drone_status(did)
            if "position" in s:
                actual_positions[did] = s["position"]
        # Log formation state
        formation_logger.log_formation_state(
            formation_type=formation_type,
            target_positions=targets,
            actual_positions=actual_positions,
            inter_drone_distances=distances,
            extra={"spacing": spacing},
        )
        return results, targets, distances

    results, targets, distances = await _run_in_thread(_do_formation)

    return {
        "status": "ok",
        "formation_type": formation_type,
        "spacing": spacing,
        "results": {k: bool(v) for k, v in results.items()},
        "target_positions": targets,
        "inter_drone_distances": distances,
    }


@app.route("/api/swarm/formation/distances", methods=["GET"])
async def api_formation_distances():
    """Return current inter-drone distances."""
    distances = formation_mgr.log_formation_distances()
    return {"status": "ok", "distances": distances}


@app.route("/api/swarm/formation/log", methods=["GET"])
async def api_formation_log():
    """Return the full formation log."""
    entries = formation_logger.read_formation_log()
    return {"status": "ok", "entries": entries}


# ═══════════════════════════════════════════════════════════════════════════
# OBSTACLE MAP API
# ═══════════════════════════════════════════════════════════════════════════
# These endpoints let tests (and the GCS UI) inject or remove obstacles at
# runtime without restarting the server.  The navigation layer in
# drone_controller.py reads from the same singleton automatically.

from obstacle_map import obstacle_map, StaticObstacle, WindZone, DynamicObstacle

# Route obstacle map internal logs to the live UI console
obstacle_map.ui_log_callback = log_to_ui


@app.route("/api/obstacles/add_static", methods=["POST"])
async def api_obstacle_add_static():
    """Add a fixed obstacle (wall, building, no-fly zone).
    Body: { "lat": float, "lon": float, "radius_m": float,
            "max_alt_m": float (opt, default 100), "label": str (opt) }
    """
    d = await request.get_json(force=True, silent=True) or {}
    obs = StaticObstacle(
        lat=float(d["lat"]),
        lon=float(d["lon"]),
        radius_m=float(d.get("radius_m", 10)),
        max_alt_m=float(d.get("max_alt_m", 100)),
        label=str(d.get("label", "static")),
    )
    obstacle_map.add_static(obs)
    return {"status": "ok", "added": repr(obs)}


@app.route("/api/obstacles/add_wind", methods=["POST"])
async def api_obstacle_add_wind():
    """Add a wind / turbulence zone.
    Body: { "lat": float, "lon": float, "radius_m": float,
            "strength": float (opt, default 1.0), "label": str (opt) }
    """
    d = await request.get_json(force=True, silent=True) or {}
    zone = WindZone(
        lat=float(d["lat"]),
        lon=float(d["lon"]),
        radius_m=float(d.get("radius_m", 20)),
        strength=float(d.get("strength", 1.0)),
        label=str(d.get("label", "wind")),
    )
    obstacle_map.add_wind(zone)
    return {"status": "ok", "added": repr(zone)}


@app.route("/api/obstacles/add_dynamic", methods=["POST"])
async def api_obstacle_add_dynamic():
    """Add a moving obstacle (bird, unknown drone, etc.).
    Body: { "lat": float, "lon": float, "alt_m": float,
            "radius_m": float, "vel_lat_dps": float, "vel_lon_dps": float,
            "label": str (opt) }
    """
    d = await request.get_json(force=True, silent=True) or {}
    obs = DynamicObstacle(
        lat=float(d["lat"]),
        lon=float(d["lon"]),
        alt_m=float(d.get("alt_m", 10)),
        radius_m=float(d.get("radius_m", 5)),
        vel_lat_dps=float(d.get("vel_lat_dps", 0)),
        vel_lon_dps=float(d.get("vel_lon_dps", 0)),
        label=str(d.get("label", "dynamic")),
    )
    obstacle_map.add_dynamic(obs)
    return {"status": "ok", "added": repr(obs)}


@app.route("/api/obstacles/clear", methods=["POST"])
async def api_obstacle_clear():
    """Remove all obstacles from the environment."""
    obstacle_map.clear()
    return {"status": "ok", "message": "All obstacles cleared"}


@app.route("/api/obstacles/status", methods=["GET"])
async def api_obstacle_status():
    """Return the current obstacle environment snapshot."""
    return {"status": "ok", "obstacles": obstacle_map.snapshot()}


# ═══════════════════════════════════════════════════════════════════════════
# SERVER STARTUP
# ═══════════════════════════════════════════════════════════════════════════

@app.route("/api/health", methods=["GET"])
async def api_health():
    """
    Health check endpoint. Returns server status, connected drone count,
    and WebSocket client count. Used by testers and monitoring tools.
    """
    connected_ids = swarm_mgr.get_connected_drone_ids()
    return {
        "status": "ok",
        "server": "GCS Swarm Commander",
        "connected_drones": connected_ids,
        "drone_count": len(connected_ids),
        "websocket_clients": len(connected_websockets),
        "obstacle_count": len(obstacle_map.snapshot().get("static", [])
                              + obstacle_map.snapshot().get("wind", [])
                              + obstacle_map.snapshot().get("dynamic", [])),
    }


@app.before_serving
async def startup():
    app.n_loop = asyncio.get_running_loop()
    app.add_background_task(broadcast_worker)
    app.add_background_task(telemetry_polling_worker)
    logging.info("="*60)
    logging.info("✅ GCS Swarm Commander server started on port 5000")
    logging.info("   Endpoints: /, /ws, /api/swarm/*, /api/drone/*, /api/health")
    logging.info("   Telemetry broadcast: active")
    logging.info("="*60)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )
    logging.info("Starting GCS Swarm Commander on 0.0.0.0:5000 ...")
    app.run(host="0.0.0.0", port=5000, use_reloader=False)