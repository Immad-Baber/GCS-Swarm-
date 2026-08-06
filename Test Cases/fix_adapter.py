import glob
import re

new_adapter = '''class TestAdapter(SITLAdapter):
    def __init__(self, drone_id: str, connection_str: str):
        super().__init__(drone_id, connection_str)
        self.initialize()
    @property
    def mav(self): return self.master.mav if self.master else None
    @property
    def target_system(self): return self.master.target_system if self.master else None
    @property
    def target_component(self): return self.master.target_component if self.master else None
    def mode_mapping(self):
        if self.master: return self.master.mode_mapping()
        return {}
    def wait_heartbeat(self):
        if self.master: return self.master.wait_heartbeat()
    def recv_match(self, *args, **kwargs):
        if self.master: return self.master.recv_match(*args, **kwargs)
        
    def set_mode(self, mode="GUIDED"):
        if not self.master: return
        mm = self.master.mode_mapping()
        if mode in mm:
            self.master.mav.set_mode_send(self.master.target_system, 1, mm[mode])
            
    def arm_vehicle(self):
        if not self.master: return
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
            400, 0, 1, 0, 0, 0, 0, 0, 0)
            
    def takeoff(self, altitude):
        if not self.master: return
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
            22, 0, 0, 0, 0, 0, 0, 0, altitude)

    def send_ned_pos(self, t0, north, east, down):
        from pymavlink import mavutil
        import time
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        tb = int((time.time() - t0) * 1000) & 0xFFFFFFFF
        self.mav.set_position_target_local_ned_send(
            tb, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            float(north), float(east), float(down),
            0, 0, 0, 0, 0, 0, 0, 0
        )'''

for f in glob.glob('*.py'):
    if f in ['refactor.py', 'fix_adapter.py']: continue
    with open(f, 'r') as file:
        data = file.read()
    
    # We will replace the existing class TestAdapter(...) block with the new one
    data = re.sub(r'class TestAdapter\(SITLAdapter\):.*?def send_ned_pos\(self, t0, north, east, down\):.*?0, 0, 0, 0, 0, 0, 0, 0\n        \)', new_adapter, data, flags=re.DOTALL)
    
    with open(f, 'w') as file:
        file.write(data)
