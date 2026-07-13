import time
import math
import os
from pymavlink import mavutil
from model.mission import Mission
from model.waypoint import Waypoint
from views.display import MissionView
from utils.mission_planner import MissionPlanner
from utils.waypoints_exporter import WaypointsExporter
from controller.mission_controller import MissionController
def send_goto(lat, lon, alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    master.mav.set_position_target_global_int_send(
        int((time.time() - boot_time) * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        lat_int,
        lon_int,
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
def distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c
def wait_for_heartbeat(master):
    print(":white_check_mark: Waiting for heartbeat...")
    master.wait_heartbeat()
    print(":white_check_mark: Heartbeat received")
def disable_arming_checks(master):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    print(":shield: Arming checks disabled")
    time.sleep(1)
def set_guided_mode(master):
    mode = 'GUIDED'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(":repeat: Setting mode to GUIDED...")
    timeout = time.time() + 10
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and master.flightmode == 'GUIDED':
            print(":white_check_mark: GUIDED mode confirmed")
            return True
        time.sleep(0.5)
    print(":x: Failed to set GUIDED mode")
    return False
def arm_uav(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print(":gear: Arming drone...")
    timeout = time.time() + 10
    while time.time() < timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0:
            master.motors_armed_wait()
            print(":white_check_mark: Drone armed")
            return True
        time.sleep(0.5)
    print(":x: Failed to arm drone")
    return False
def takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, alt
    )
    print(f":rocket: Takeoff initiated to {alt} meters")
    timeout = time.time() + 15
    while time.time() < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg and abs(msg.relative_alt / 1000.0 - alt) < 0.5:
            print(f":white_check_mark: Reached takeoff altitude: {msg.relative_alt / 1000.0:.1f}m")
            return True
        time.sleep(0.5)
    print(":x: Failed to reach takeoff altitude")
    return False
def goto_location(master, lat, lon, alt):
    print(f":round_pushpin: Moving toward lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}...")
    while True:
        send_goto(lat, lon, alt)
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            dist = distance_meters(current_lat, current_lon, lat, lon)
            print(f":satellite_antenna: Current: lat={current_lat:.6f}, lon={current_lon:.6f}, alt={current_alt:.1f} → Distance to waypoint: {dist:.2f}m")
            if dist < 1.0:
                print(f":white_check_mark: Waypoint reached: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}")
                return True
        time.sleep(0.2)
def return_to_launch(master, home_lat, home_lon, home_alt):
    print(f":house: Returning to home: lat={home_lat:.6f}, lon={home_lon:.6f}, alt={home_alt:.1f}...")
    if goto_location(master, home_lat, home_lon, home_alt):
        print(":white_check_mark: Home location reached")
    else:
        print(":x: Failed to return to home")
def land(master):
    print(":airplane_arriving: Landing...")
    if master.flightmode != 'GUIDED':
        print(f":x: Not in GUIDED mode, current mode: {master.flightmode}")
        return False
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    # Wait for acknowledgment
    ack_timeout = time.time() + 5
    while time.time() < ack_timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            if msg.result == 0:
                print(":white_check_mark: Landing command accepted")
            else:
                print(f":x: Landing command failed with result: {msg.result}")
            break
    timeout = time.time() + 30
    while time.time() < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f"Current altitude: {current_alt:.2f}m")
            if current_alt < 0.2:
                print(":white_check_mark: Drone landed")
                return True
        else:
            print(":warning: No GLOBAL_POSITION_INT message received")
        time.sleep(0.5)
    print(":x: Failed to land")
    return False
def main():
    global master, boot_time
    boot_time = time.time()
    print(":electric_plug: Connecting to SITL (udp:127.0.0.1:14551)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    #master = mavutil.mavlink_connection('udp:localhost:14551')
    mission = Mission()
    view = MissionView()
    planner = MissionPlanner(mission)
    exporter = WaypointsExporter()
    controller = MissionController(mission, view, planner, exporter)
    try:
        controller.run()
        home_lat = 33.65613704754941
        home_lon = 73.0160236189347
        home_alt = 10
        wait_for_heartbeat(master)
        disable_arming_checks(master)
        if not set_guided_mode(master):
            raise Exception("Failed to set GUIDED mode")
        if not arm_uav(master):
            raise Exception("Failed to arm UAV")
        if not takeoff(master, home_alt):
            raise Exception("Failed to complete takeoff")
        waypoints = mission.get_waypoints()
        print(f":globe_with_meridians: Waypoints: {[str(wp) for wp in waypoints]}")
        if waypoints:
            print(f":globe_with_meridians: Executing mission with {len(waypoints)} waypoints")
            for i, wp in enumerate(waypoints):
                print(f"Starting waypoint {i+1}/{len(waypoints)}: {wp}")
                if not goto_location(master, wp.latitude, wp.longitude, wp.altitude):
                    raise Exception(f"Failed to reach waypoint: lat={wp.latitude}, lon={wp.longitude}, alt={wp.altitude}")
                print(":double_vertical_bar: Hovering at waypoint...")
                time.sleep(5)
        else:
            print(":x: No waypoints defined. Aborting mission.")
            raise Exception("No waypoints defined")
        return_to_launch(master, home_lat, home_lon, home_alt)
        if not land(master):
            raise Exception("Failed to land")
        print(":white_check_mark: Mission completed.")
    except Exception as e:
        print(f":x: ERROR: {e}")
        exit(1)
    finally:
        master.close()
if __name__ == "__main__":
    main()