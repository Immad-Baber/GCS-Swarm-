import glob
import os

def refactor_test_case(filepath):
    with open(filepath, 'r') as f:
        data = f.read()

    # 1. Add imports
    if 'from simulation_test_adapter import SimulationTestAdapter' not in data:
        data = data.replace('from pymavlink import mavutil', 
                            'from pymavlink import mavutil\nimport sys, os\nsys.path.append(os.path.abspath(\'../sitl_final_package/mavlink_integration\'))\nfrom simulation_test_adapter import SimulationTestAdapter')

    # 2. Refactor connect_all
    old_connect = 'conns = [mavutil.mavlink_connection(f"udpin:0.0.0.0:{p}") for p in PORTS]'
    new_connect = 'conns = [SimulationTestAdapter(f"drone_{i+1}", f"udpin:0.0.0.0:{p}") for i, p in enumerate(PORTS)]'
    data = data.replace(old_connect, new_connect)

    # 3. Refactor prep_vehicles
    old_prep = '''def prep_vehicles(conns, takeoff_alt=15):
    for c in conns:
        set_mode(c, "GUIDED")
        time.sleep(0.2)
        arm(c)
        time.sleep(0.2)
        takeoff(c, takeoff_alt)
        time.sleep(0.2)
    time.sleep(6)'''
    
    new_prep = '''def prep_vehicles(conns, takeoff_alt=15):
    for c in conns:
        c.set_mode("GUIDED")
        time.sleep(0.2)
        c.arm_vehicle()
        time.sleep(0.2)
        c.takeoff(takeoff_alt)
        time.sleep(0.2)
    time.sleep(6)'''
    data = data.replace(old_prep, new_prep)

    # 4. Refactor send_ned_pos calls
    data = data.replace('send_ned_pos(c, ', 'c.send_ned_pos(T0, ')
    data = data.replace('send_ned_pos(conn, ', 'conn.send_ned_pos(T0, ')
    data = data.replace('send_ned_pos(conns[idx], ', 'conns[idx].send_ned_pos(T0, ')
    data = data.replace('send_ned_pos(conns[i], ', 'conns[i].send_ned_pos(T0, ')

    with open(filepath, 'w') as f:
        f.write(data)
    print(f'Refactored {filepath}')

for f in glob.glob('*.py'):
    if f != 'refactor.py':
        refactor_test_case(f)
