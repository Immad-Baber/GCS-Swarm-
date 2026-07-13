import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14553")
master.wait_heartbeat()

msg = master.recv_match(type="HEARTBEAT", blocking=True)
armed = master.motors_armed()
mode = mavutil.mode_string_v10(msg)
print(f"Drone 3 is in mode: {mode}, Armed: {armed}")

# Check failsafes
sys_status = master.recv_match(type="SYS_STATUS", blocking=True)
if sys_status:
    print(f"Sys Status - Battery: {sys_status.voltage_battery/1000.0}V, Current: {sys_status.current_battery/100.0}A, Remaining: {sys_status.battery_remaining}%")
