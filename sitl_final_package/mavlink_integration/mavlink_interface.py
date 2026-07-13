# mavlink_interface.py
from pymavlink import mavutil

class MAVLinkInterface:
    def __init__(self, connection_str: str):
        self.connection_str = connection_str
        self.master = None

    def connect(self):
        print(f"[INFO] Connecting to {self.connection_str}...")
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.wait_heartbeat()

    def wait_heartbeat(self):
        print("[INFO] Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"[INFO] Heartbeat received from system {self.master.target_system}, component {self.master.target_component}")

    def send_command_long(self, command, params):
        print(f"[DEBUG] Sending command {command} with params {params}")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command,
            0,  # confirmation
            *params
        )

    def recv_msg(self, msg_type="COMMAND_ACK", blocking=True):
        msg = self.master.recv_match(type=msg_type, blocking=blocking)
        if msg:
            print(f"[RECV] {msg}")
        return msg

    def get_master(self):
        return self.master
