from quart import Quart, websocket
import asyncio

app = Quart(__name__)
clients = set()

@app.websocket("/ws")
async def ws():
    clients.add(websocket._get_current_object())
    try:
        while True:
            await websocket.send("Hello from server!")
            await asyncio.sleep(1)
    except:
        clients.remove(websocket._get_current_object())

@app.route("/")
async def index():
    return "WebSocket server running"

if __name__ == "__main__":
    import hypercorn.asyncio
    import hypercorn.config
    config = hypercorn.config.Config()
    config.bind = ["0.0.0.0:5000"]
    asyncio.run(hypercorn.asyncio.serve(app, config))