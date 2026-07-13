import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14552")
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}")

# FORCE GUIDED MODE
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4 # GUIDED
)
time.sleep(1)

master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
pos = master.messages.get("GLOBAL_POSITION_INT")
if pos:
    lat = pos.lat / 1e7 + 0.0002
    lon = pos.lon / 1e7 + 0.0002
    alt = 15.0
    print(f"Moving to lat={lat}, lon={lon}, alt={alt}")
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0, 0,
        -1.0, 0, 0, float("nan"),
        int(lat * 1e7), int(lon * 1e7), alt
    )
    time.sleep(5)
    # read fresh position
    while True:
        m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if not m: break
    
    pos2 = master.messages.get("GLOBAL_POSITION_INT")
    if pos2:
        print(f"New pos: lat={pos2.lat}, lon={pos2.lon}")
