from pymavlink import mavutil
import time
import math

boot_time = time.time()

# Connect to SITL
master = mavutil.mavlink_connection('udp:localhost:14551')

# Wait for heartbeat
master.wait_heartbeat()
print("✅ Heartbeat received")

# Set mode to GUIDED
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("🔁 Set mode to GUIDED")
time.sleep(2)

# Arm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("⚙️ Arming drone")
master.motors_armed_wait()
print("✅ Drone armed")

# Takeoff to 10 meters
takeoff_alt = 10
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, takeoff_alt
)
print("🚀 Takeoff initiated")
time.sleep(10)

# Function to send position target
def send_goto(lat, lon, alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)

    master.mav.set_position_target_global_int_send(
        int((time.time() - boot_time) * 1000),  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # type_mask: only position
        lat_int,
        lon_int,
        alt,
        0, 0, 0,       # vx, vy, vz
        0, 0, 0,       # afx, afy, afz
        0, 0           # yaw, yaw_rate
    )

# Function to compute distance in meters between lat/lon
def distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# === TARGET GPS COORDINATES ===
target_lat = 33.636857467637135  # adjust this
target_lon = 73.06503694535758   # adjust this
target_alt = 10                  # relative altitude in meters
# ==============================

print(f"📍 Moving toward lat={target_lat}, lon={target_lon}, alt={target_alt} ...")

# Send GPS commands until drone reaches destination
while True:
    send_goto(target_lat, target_lon, target_alt)

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0  # in meters

        dist = distance_meters(current_lat, current_lon, target_lat, target_lon)
        print(f"📡 Current: lat={current_lat:.6f}, lon={current_lon:.6f}, alt={current_alt:.1f} → Distance to target: {dist:.2f}m")

        if dist < 3.0:
            print("✅ Target location reached")
            break
    time.sleep(0.2)

# Hover for a few seconds
print("⏸️ Hovering at target location...")
time.sleep(5)

# Land the drone
print("🛬 Landing...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)
