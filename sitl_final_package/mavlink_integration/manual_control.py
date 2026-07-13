from pymavlink import mavutil
import time
import threading

# Connect to SITL
master = mavutil.mavlink_connection('udp:localhost:14551')
master.wait_heartbeat()
print("✅ Heartbeat received")

# Set mode to GUIDED
mode_id = master.mode_mapping()['GUIDED']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("🔁 Mode set to GUIDED")
time.sleep(1)

# Arm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
master.motors_armed_wait()
print("⚙️ Drone armed")

# Takeoff to 5m
altitude = 5
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, altitude
)
print("🚀 Taking off to 5m")
time.sleep(8)

# Send local NED movement commands
def move_relative_ned(vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # relative to current heading
        0b0000111111000111,  # enable only velocity
        0, 0, 0,             # x, y, z positions (ignored)
        vx, vy, vz,          # velocities (m/s)
        0, 0, 0,             # accelerations (not used)
        0, 0                 # yaw, yaw_rate
    )

# Keyboard control thread
def control_loop():
    print("""
Controls:
    w = forward
    s = backward
    a = left
    d = right
    r = up
    f = down
    l = land
    q = quit
""")
    while True:
        key = input(">> ").strip().lower()
        if key == 'w':
            move_relative_ned(1, 0, 0)
        elif key == 's':
            move_relative_ned(-1, 0, 0)
        elif key == 'a':
            move_relative_ned(0, -1, 0)
        elif key == 'd':
            move_relative_ned(0, 1, 0)
        elif key == 'r':
            move_relative_ned(0, 0, -0.5)  # Up is negative Z
        elif key == 'f':
            move_relative_ned(0, 0, 0.5)   # Down is positive Z
        elif key == 'l':
            print("🛬 Landing...")
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            break
        elif key == 'q':
            print("❌ Exiting and landing...")
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            break
        else:
            print("❓ Invalid command")
        time.sleep(1)

control_loop()
