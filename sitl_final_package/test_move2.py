import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14552")
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}")

master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
pos = master.messages.get("GLOBAL_POSITION_INT")
if pos:
    lat = pos.lat + 1000
    lon = pos.lon + 1000
    alt = pos.relative_alt / 1000.0
    print(f"Moving to lat={lat}, lon={lon}, alt={alt}")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x0DF8,
        lat, lon, alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(5)
    # read fresh position
    while True:
        m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if not m: break
    
    pos2 = master.messages.get("GLOBAL_POSITION_INT")
    if pos2:
        print(f"New pos: lat={pos2.lat}, lon={pos2.lon}")
