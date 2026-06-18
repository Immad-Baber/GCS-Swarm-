import websocket
import json
import time

def on_message(ws, message):
    print("Received:", message)

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection closed")

def on_open(ws):
    print("Connection opened")
    # Send a valid launch message
    launch_message = {
        "type": "launch",
        "message": "Test launch from Python client"
    }
    ws.send(json.dumps(launch_message))

if __name__ == "__main__":
    websocket.enableTrace(False)  # Optional: set to True to debug
    ws = websocket.WebSocketApp("ws://172.17.23.169:5001",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.run_forever()
