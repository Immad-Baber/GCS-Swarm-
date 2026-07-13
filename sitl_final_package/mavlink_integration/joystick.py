from evdev import InputDevice, categorize, ecodes

device_path = '/dev/input/by-id/usb-Thustmaster_Joystick_-_HOTAS_Warthog-event-joystick'
dev = InputDevice(device_path)
print(f"Using device: {dev.name}")

max_values = {'ABS_X': 0, 'ABS_Y': 0}
min_values = {'ABS_X': 65535, 'ABS_Y': 65535}  # Assuming 16-bit max as starting min
exit_key_code = 289  # Change this to the button code you want to use to exit

print(f"Press button with code {exit_key_code} to exit and show stats.")

try:
    for event in dev.read_loop():
        if event.type == ecodes.EV_KEY:
            print(f"[BUTTON] Code: {event.code} | State: {event.value}")
            if event.code == exit_key_code and event.value == 1:  # Button pressed
                print("\nExit button pressed. Exiting...\n")
                break

        elif event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            axis_name = ecodes.ABS.get(absevent.event.code, 'UNKNOWN')
            value = absevent.event.value
            print(f"[AXIS] {axis_name} => {value}")

            if axis_name in max_values:
                if value > max_values[axis_name]:
                    max_values[axis_name] = value
                    print(f"*** New max {axis_name}: {value} ***")

                if value < min_values[axis_name]:
                    min_values[axis_name] = value
                    print(f"*** New min {axis_name}: {value} ***")

except KeyboardInterrupt:
    print("\nKeyboardInterrupt received. Exiting...\n")

# Print summary
print("Final Min/Max values:")
for axis in max_values.keys():
    print(f"{axis}: min = {min_values[axis]}, max = {max_values[axis]}")
