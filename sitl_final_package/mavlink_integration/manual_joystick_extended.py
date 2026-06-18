import time
from pymavlink import mavutil
from evdev import InputDevice, categorize, ecodes, list_devices
import threading

# -- SETTINGS --
UDP_CONNECTION = "udp:127.0.0.1:14551"
TAKEOFF_ALTITUDE = 10  # meters
VELOCITY_SCALE = 2.0   # max m/s velocity from joystick input [-1..1]
ALTITUDE_STEP = 1.0    # meters per button press

# Globals
boot_time = None  # Will hold program start time for time_boot_ms

# Find joystick device dynamically
def find_joystick_device():
    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        if 'Thrustmaster' in dev.name or 'HOTAS' in dev.name:
            return dev
    raise RuntimeError("Joystick device not found")

# Normalize joystick axis values from [min, max] to [-1, 1]
def normalize(value, absinfo):
    center = (absinfo.max + absinfo.min) / 2
    span = (absinfo.max - absinfo.min) / 2
    norm = (value - center) / span
    return max(min(norm, 1), -1)

# Send local velocity command to drone
def send_velocity(master, vx, vy, vz):
    global boot_time
    if boot_time is None:
        boot_time = time.time()
    time_boot_ms = int((time.time() - boot_time) * 1000)  # ms since program start

    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # ignore pos, accel, yaw
        0, 0, 0,  # x, y, z positions (ignored)
        vx, vy, vz,  # vx, vy, vz velocity in m/s
        0, 0, 0,  # afx, afy, afz acceleration (ignored)
        0, 0  # yaw, yaw_rate (ignored)
    )

def wait_for_ack(master, command, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        ack = master.recv_match(type='COMMAND_ACK', blocking=False)
        if ack and ack.command == command:
            return ack
        time.sleep(0.1)
    return None



def main():
    global boot_time
    print("🔌 Connecting to SITL...")
    master = mavutil.mavlink_connection(UDP_CONNECTION)
    master.wait_heartbeat()
    print(f"✅ Heartbeat received from system {master.target_system}, component {master.target_component}")

    # Set GUIDED mode
    mode = 'GUIDED'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    ack = wait_for_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    if ack is None:
        print("⚠️ Warning: No ACK received for mode set command")
    else:
        print(f"🚁 Mode set ACK: {ack}")

    # Arm vehicle
    master.arducopter_arm()
    ack = wait_for_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    if ack is None:
        print("⚠️ Warning: No ACK received for arm command")
    else:
        print("🟢 Vehicle armed")

    # Takeoff
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALTITUDE
    )
    print(f"🚀 Taking off to {TAKEOFF_ALTITUDE}m...")
    time.sleep(10)  # wait for takeoff

    # Open joystick device
    dev = find_joystick_device()
    print(f"🎮 Using joystick: {dev.name}")

    # Get absinfo for normalization
    abs_ranges = {}
    for abs_code in (ecodes.ABS_X, ecodes.ABS_Y):
        try:
            abs_ranges[abs_code] = dev.absinfo(abs_code)
        except Exception:
            pass  # Some axes may not exist

    # Current target altitude controlled by buttons
    target_altitude = TAKEOFF_ALTITUDE

    # Speed multiplier for strafing velocity
    speed_multiplier = 1.0

    # Thread-safe storage of velocity and altitude changes
    control_state = {
        'vx': 0.0,
        'vy': 0.0,
        'vz': 0.0,
        'target_altitude': target_altitude,
        'speed_multiplier': speed_multiplier
    }

    # Send manual control command to drone
    def send_manual_control(master, x, y, z, r):
        master.mav.manual_control_send(
            master.target_system,
            int(x),  # pitch/roll (x)
            int(y),  # pitch/roll (y)
            int(z),  # throttle (0 to 1000; lower means higher throttle)
            int(r),  # yaw
            0        # buttons (bitmask)
        )


    def joystick_loop():
        nonlocal control_state
        for event in dev.read_loop():
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X and ecodes.ABS_X in abs_ranges:
                    control_state['vy'] = -normalize(event.value, abs_ranges[ecodes.ABS_X]) * VELOCITY_SCALE * control_state['speed_multiplier']
                elif event.code == ecodes.ABS_Y and ecodes.ABS_Y in abs_ranges:
                    control_state['vx'] = -normalize(event.value, abs_ranges[ecodes.ABS_Y]) * VELOCITY_SCALE * control_state['speed_multiplier']
            elif event.type == ecodes.EV_KEY and event.value == 1:  # on button press only
                if event.code == 289:  # Increase altitude
                    control_state['target_altitude'] += ALTITUDE_STEP
                    print(f"⬆️ Increasing altitude to {control_state['target_altitude']:.1f}m")
                elif event.code == 705:  # Decrease speed multiplier
                    control_state['speed_multiplier'] = max(0.1, control_state['speed_multiplier'] - 1.0)
                    print(f"🐢 Speed multiplier: {control_state['speed_multiplier']:.1f}x")
                elif event.code == 303:  # Increase speed multiplier
                    control_state['speed_multiplier'] += 1.0
                    print(f"⚡ Speed multiplier: {control_state['speed_multiplier']:.1f}x")

                # Strafing buttons (slide left/right) — using manual control
                elif event.code == 298:  # Strafe Left forward
                    send_manual_control(master, int(-500 * control_state['speed_multiplier']), 0, 500, 0)
                    print("⏪ Strafing Left Forward")
                elif event.code == 299:  # Strafe Left backward
                    send_manual_control(master, int(-500 * control_state['speed_multiplier']), 0, int(-500 * control_state['speed_multiplier']), 0)
                    print("⏪ Strafing Left Backward")
                elif event.code == 300:  # Strafe Right forward
                    send_manual_control(master, int(500 * control_state['speed_multiplier']), 0, 500, 0)
                    print("⏩ Strafing Right Forward")
                elif event.code == 301:  # Strafe Right backward
                    send_manual_control(master, int(500 * control_state['speed_multiplier']), 0, int(-500 * control_state['speed_multiplier']), 0)
                    print("⏩ Strafing Right Backward")


    # Start joystick event thread
    threading.Thread(target=joystick_loop, daemon=True).start()

    print("🕹️ Manual control active — use joystick and buttons")

    boot_time = time.time()  # Set boot time here before control loop

    # Main control loop
    while True:
        vx = control_state['vx']
        vy = control_state['vy']

        # Get current altitude from drone telemetry
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # meters
        else:
            current_alt = control_state['target_altitude']

        # Simple P controller for vertical speed
        alt_error = control_state['target_altitude'] - current_alt
        vz = -alt_error * 0.5  # gain of 0.5
        vz = max(min(vz, 1.0), -1.0)  # limit vertical speed

        send_velocity(master, vx, vy, vz)
        #print(f"Sending velocity vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} | Target alt={control_state['target_altitude']:.1f}m, Current alt={current_alt:.1f}m")

        time.sleep(0.1)

if __name__ == "__main__":
    main()