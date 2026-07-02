import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14553")
master.wait_heartbeat()

msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
lat = msg.lat / 1e7 + 0.005
lon = msg.lon / 1e7 + 0.005
alt = 15.0

print(f"Moving to lat={lat}, lon={lon}, alt={alt}")
master.mav.command_int_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_DO_REPOSITION,
    0, 0,
    -1.0, 0, 0, 0,  # Try 0 instead of nan for yaw
    int(lat * 1e7), int(lon * 1e7), alt
)

time.sleep(5)
msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
print(f"New pos: lat={msg.lat}, lon={msg.lon}, alt={msg.relative_alt/1000.0}")
