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
    """Receive telemetry from a drone adapter and broadcast via WebSocket."""
    data = await request.get_json()
    await emit_telemetry(data)
    return {"status": "ok"}


async def broadcast_worker():
    """Background task: pull messages from the queue and send to all clients."""
    while True:
        message = await broadcast_queue.get()
        if connected_websockets:
            disconnected = set()
            for ws in connected_websockets:
                try:
                    await ws.send(message)
                except Exception as e:
                    print(f"Error sending to client: {e}")
                    disconnected.add(ws)
            for ws in disconnected:
                connected_websockets.remove(ws)


async def telemetry_polling_worker():
    """Background task: periodically poll all connected drones for telemetry."""
    while True:
        await asyncio.sleep(1)
        # We run this in an executor because adapter.log_status() uses requests.post
        # and pymavlink recv_match which are blocking operations.
        def _poll_telemetry():
            for drone_id in swarm_mgr.get_connected_drone_ids():
                adapter = swarm_mgr.get_adapter(drone_id)
                if adapter:
                    try:
                        # This fetches telemetry and posts it to our own /send_telemetry endpoint
                        adapter.log_status()
                    except Exception as e:
                        logging.error(f"Error polling telemetry for {drone_id}: {e}")
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, _poll_telemetry)


async def emit_telemetry(data):
    """Push telemetry data into the broadcast queue and log it."""
    json_message = json.dumps(data)
    print(f"[DEBUG] Emitting telemetry: {json_message}")
    loop = asyncio.get_event_loop()
    if loop.is_running():
        asyncio.run_coroutine_threadsafe(broadcast_queue.put(json_message), loop)
    else:
        loop.run_until_complete(broadcast_queue.put(json_message))
    telemetry_logger.append_log(data.get("drone_id", "unknown"), data)


@app.route("/export_swarm_log", methods=["GET"])
async def export_swarm_log():
    """Export combined swarm telemetry log."""
    combined_path = telemetry_logger.combine_logs()
    try:
        with open(combined_path, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception as e:
        return {"status": "error", "message": str(e)}
    return {"status": "ok", "combined_log": content}


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

@app.route("/api/test/run", methods=["POST"])
async def api_test_run():
    """Run an automated test scenario in the background."""
    data = await request.get_json(force=True, silent=True) or {}
    test_id = int(data.get("test_id", 1))
    logging.info(f"[API] /api/test/run — test_id={test_id}")
    
    def _run_scenario():
        try:
            if test_id == 1:
                test_swarm_scenarios.scenario_1()
            elif test_id == 2:
                test_swarm_scenarios.scenario_2()
            elif test_id == 3:
                test_swarm_scenarios.scenario_3()
            elif test_id == 4:
                test_swarm_scenarios.scenario_4()
            else:
                logging.error(f"Unknown test id: {test_id}")
        except Exception as e:
            logging.error(f"Scenario {test_id} error: {e}")
            
    t = threading.Thread(target=_run_scenario, daemon=True)
    t.start()
    
    return {"status": "ok", "message": f"Test {test_id} started in background"}


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
    logging.info(f"[API] /api/drone/{drone_id}/takeoff — altitude={altitude}, mission={mission}")
    result = await _run_in_thread(swarm_mgr.takeoff_drone, drone_id, altitude, mission)
    return {"status": "ok", "drone_id": drone_id, "takeoff": bool(result)}


@app.route("/api/drone/<drone_id>/land", methods=["POST"])
async def api_drone_land(drone_id):
    """Land a specific drone."""
    logging.info(f"[API] /api/drone/{drone_id}/land")
    result = await _run_in_thread(swarm_mgr.land_drone, drone_id)
    return {"status": "ok", "drone_id": drone_id, "landed": bool(result)}


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
# SERVER STARTUP
# ═══════════════════════════════════════════════════════════════════════════

@app.before_serving
async def startup():
    app.add_background_task(broadcast_worker)
    app.add_background_task(telemetry_polling_worker)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, use_reloader=False)