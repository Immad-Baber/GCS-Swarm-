import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14552')
print('Waiting for heartbeat...')
master.wait_heartbeat()
print(f'Connected! System ID: {master.target_system}')

print('Sending LAND command...')
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)

ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack:
    print(f'ACK: command={ack.command}, result={ack.result}')
else:
    print('No ACK received!')
