import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14552')
master.wait_heartbeat()
print(f'Connected! System ID: {master.target_system}')

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4 # GUIDED mode
)
time.sleep(1)

# Takeoff to 10m
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10.0
)
time.sleep(5)

# Move to a new position (e.g. lat + 0.0001, lon + 0.0001)
master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
pos = master.messages.get('GLOBAL_POSITION_INT')
if pos:
    lat = pos.lat + 1000
    lon = pos.lon + 1000
    alt = pos.relative_alt / 1000.0
    print(f'Moving to lat={lat}, lon={lon}, alt={alt}')
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        lat, lon, alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(5)
    pos2 = master.messages.get('GLOBAL_POSITION_INT')
    print(f'New pos: lat={pos2.lat}, lon={pos2.lon}')
