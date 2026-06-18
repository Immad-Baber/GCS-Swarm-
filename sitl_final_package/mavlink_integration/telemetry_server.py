import asyncio
import json
from quart import Quart, websocket, request
import random
import time
import datetime

app = Quart(__name__)

connected_websockets = set()
broadcast_queue = asyncio.Queue()

@app.websocket("/ws")
async def ws():
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

from quart import Quart, websocket, request, make_response

@app.route("/")
async def index():
    import os
    html_path = os.path.join(os.path.dirname(__file__), "index.html")
    with open(html_path, "r", encoding="utf-8") as f:
        return await make_response(f.read())


@app.route("/send_telemetry", methods=["POST"])
async def send_telemetry():
    data = await request.get_json()
    await emit_telemetry(data)
    return {"status": "ok"}



async def broadcast_worker():
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

async def emit_telemetry(data):
    json_message = json.dumps(data)
    print(f"[DEBUG] Emitting telemetry: {json_message}")  # ADD THIS LINE
    loop = asyncio.get_event_loop()
    if loop.is_running():
        asyncio.run_coroutine_threadsafe(broadcast_queue.put(json_message), loop)
    else:
        loop.run_until_complete(broadcast_queue.put(json_message))


@app.before_serving
async def startup():
    app.add_background_task(broadcast_worker)
    #app.add_background_task(generate_fake_telemetry) #this is to test the server, websocket with fake data


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

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, use_reloader=False)