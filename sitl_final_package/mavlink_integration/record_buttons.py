import select
from evdev import InputDevice, categorize, ecodes

# Device paths
stick_path = '/dev/input/by-id/usb-Thustmaster_Joystick_-_HOTAS_Warthog-event-joystick'
throttle_path = '/dev/input/by-id/usb-Thrustmaster_Throttle_-_HOTAS_Warthog-event-joystick'

# Open devices
stick = InputDevice(stick_path)
throttle = InputDevice(throttle_path)

print(f"Reading from:")
print(f"  Joystick: {stick.name} ({stick_path})")
print(f"  Throttle: {throttle.name} ({throttle_path})")
print("\nPress buttons or move axes... (Ctrl+C to stop)\n")

# Device lookup
devices = {
    stick.fd: ('Joystick', stick),
    throttle.fd: ('Throttle', throttle)
}

# === Pretty button names (custom mapping based on common HOTAS Warthog config) ===
BUTTON_NAMES = {
    # Joystick buttons
    288: "TRIGGER FIRST STAGE",
    289: "TRIGGER SECOND STAGE",
    290: "PINKY SWITCH",
    291: "COOLIE HAT UP",
    292: "COOLIE HAT DOWN",
    293: "COOLIE HAT LEFT",
    294: "COOLIE HAT RIGHT",
    295: "TMS UP",
    296: "TMS DOWN",
    297: "TMS LEFT",
    298: "TMS RIGHT",
    299: "DMS UP",
    300: "DMS DOWN",
    301: "DMS LEFT",
    302: "DMS RIGHT",
    303: "TRIGGER",
    704: "PINKY PADDLE",
    705: "CMS LEFT",
    706: "CMS RIGHT",
    707: "CMS UP",
    708: "CMS DOWN",

    # Throttle buttons
    709: "BOAT SWITCH UP",
    710: "BOAT SWITCH DN",
    711: "BOAT SWITCH CTR",
    712: "CHINA HAT FWD",
    713: "CHINA HAT AFT",
    714: "CHINA HAT CTR",
    715: "SPEED BRAKE FWD",
    718: "SPEED BRAKE AFT",
    719: "SPEED BRAKE CTR",
}

# Axis names (common HOTAS mappings)
AXIS_NAMES = {
    ecodes.ABS_X: "STICK X (Roll)",
    ecodes.ABS_Y: "STICK Y (Pitch)",
    ecodes.ABS_RX: "STICK TWIST (Yaw)",
    ecodes.ABS_Z: "THROTTLE LEFT",
    ecodes.ABS_RZ: "THROTTLE RIGHT",
    ecodes.ABS_THROTTLE: "THROTTLE",
    ecodes.ABS_HAT0X: "COOLIE LEFT/RIGHT",
    ecodes.ABS_HAT0Y: "COOLIE UP/DOWN",
}

# Event loop
try:
    while True:
        r, _, _ = select.select(devices.keys(), [], [])
        for fd in r:
            name, dev = devices[fd]
            for event in dev.read():
                if event.type == ecodes.EV_KEY:
                    button_name = BUTTON_NAMES.get(event.code, f"BUTTON {event.code}")
                    state = 'PRESSED' if event.value == 1 else 'RELEASED'
                    print(f"[{name}] [BUTTON] {button_name} - {state}")
                elif event.type == ecodes.EV_ABS:
                    axis_name = AXIS_NAMES.get(event.code, f"AXIS {event.code}")
                    print(f"[{name}] [AXIS] {axis_name} => {event.value}")
except KeyboardInterrupt:
    print("\nStopped.")
