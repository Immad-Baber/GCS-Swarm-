import requests
import datetime
import time

url = "http://localhost:5000/send_telemetry"  # Change if your server runs elsewhere

def send_telemetry():
    data = {
        "battery": {
            "voltage": 12.5,
            "remaining": 85,
            "current": 10.0
        },
        "mode": "AUTO",
        "armed": "false",
        "position": {
            "lat": 40.7128,
            "lon": -74.0060,
            "alt": 20,
            "timestamp": datetime.datetime.utcnow().isoformat()
        }
    }
    response = requests.post(url, json=data)
    print(f"Status: {response.status_code}, Response: {response.json()}")

if __name__ == "__main__":
    while True:
        send_telemetry()
        time.sleep(2)  # send every 2 seconds
