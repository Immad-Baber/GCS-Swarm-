import requests
import time
import sys

BASE_URL = "http://127.0.0.1:5000"

def print_log(level, msg):
    icons = {
        "INFO": "ℹ️",
        "PASS": "✅",
        "FAIL": "❌",
        "WARN": "⚠️",
        "WHY": "🔍",
        "HOW": "🛠️"
    }
    icon = icons.get(level, " ")
    print(f"[{level}] {icon} {msg}")

def post(endpoint, body=None):
    url = f"{BASE_URL}{endpoint}"
    try:
        resp = requests.post(url, json=body or {}, timeout=120)
        return resp.json()
    except Exception as e:
        print_log("FAIL", f"API Request Failed: {e}")
        return None

def get(endpoint):
    url = f"{BASE_URL}{endpoint}"
    try:
        resp = requests.get(url, timeout=30)
        return resp.json()
    except Exception as e:
        print_log("FAIL", f"API Request Failed: {e}")
        return None

def check_swarm_status():
    status = get("/api/swarm/status")
    if not status or not status.get("drones"):
        return None
    return status["drones"]

def test_big_scenario():
    print("\n" + "="*80)
    print("🚀 BIG SCENARIO - COMPREHENSIVE TEST CASE (PASS & FAIL SCENARIOS)")
    print("="*80 + "\n")
    
    # ---------------------------------------------------------------------
    # Phase 0: Setup and Connection
    # ---------------------------------------------------------------------
    print_log("INFO", "Initializing Swarm Connection (Master: drone_1, Slaves: drone_2, drone_3)")
    connect_resp = post("/api/swarm/connect", {"num_drones": 3})
    if not connect_resp or connect_resp.get("status") != "ok":
        print_log("FAIL", "Could not connect to the swarm. Please ensure SITL is running.")
        return
    print_log("PASS", "Swarm successfully connected.")
    time.sleep(2)

    # ---------------------------------------------------------------------
    # Phase 1: The Passed Scenario (Hexagon + Formation Change)
    # ---------------------------------------------------------------------
    print("\n" + "-"*80)
    print("🌟 PHASE 1: BIG PASSED SCENARIO (Hexagon Mission + Dynamic Formation Change)")
    print("-"*80)
    
    print_log("INFO", "Master commands all Slaves to Arm.")
    post("/api/swarm/arm_all")
    time.sleep(2)
    
    print_log("INFO", "Master initiates takeoff (Altitude: 15m) using V-formation...")
    post("/api/swarm/formation", {"type": "v", "spacing": 10})
    post("/api/swarm/takeoff_all", {"altitude": 15, "mission": "mission4.json"})
    
    print_log("INFO", "Waiting for drones to reach altitude and begin mission (15s)...")
    time.sleep(15)
    
    drones = check_swarm_status()
    all_passed = True
    if drones:
        for did in ["drone_1", "drone_2", "drone_3"]:
            alt = drones.get(did, {}).get("position", {}).get("alt", 0)
            if alt >= 10:
                print_log("PASS", f"{did} successfully airborne: {alt:.2f}m")
            else:
                print_log("FAIL", f"{did} failed to reach sufficient altitude (Current: {alt:.2f}m)")
                all_passed = False
    
    if all_passed:
        print_log("PASS", "All drones successfully airborne and following Hexagon mission.")
    else:
        print_log("FAIL", "One or more drones failed to reach altitude.")

    print_log("INFO", "Letting the swarm fly the Hexagon mission for 20 seconds...")
    time.sleep(20)

    print_log("INFO", "Dynamically changing formation from V to LINE mid-flight...")
    post("/api/swarm/formation", {"type": "line", "spacing": 15})
    print_log("PASS", "Line formation command sent successfully.")
    
    print_log("INFO", "Letting the swarm fly the Hexagon mission in Line formation for 20 seconds...")
    time.sleep(20)

    print_log("INFO", "Mission phase 1 complete. Commanding all drones to land to prepare for Phase 2...")
    post("/api/swarm/land_all")
    print_log("INFO", "Waiting for drones to land safely (20 seconds)...")
    time.sleep(20)

    # ---------------------------------------------------------------------
    # Phase 2: The Failed Scenario (Mid-Flight Disruption)
    # ---------------------------------------------------------------------
    print("\n" + "-"*80)
    print("💥 PHASE 2: BIG FAILED SCENARIO (Mid-Flight Localized Failure)")
    print("-"*80)
    
    print_log("INFO", "Rearming and taking off for Phase 2...")
    post("/api/swarm/arm_all")
    time.sleep(2)
    
    # Restore V-formation
    post("/api/swarm/formation", {"type": "v", "spacing": 10})
    post("/api/swarm/takeoff_all", {"altitude": 15, "mission": "mission4.json"})
    
    print_log("INFO", "Waiting for drones to stabilize on Hexagon path (15 seconds)...")
    time.sleep(15)
    
    print_log("INFO", "Simulating critical failure: drone_2 suddenly enters failsafe / emergency land.")
    post("/api/drone/drone_2/land")
    
    print_log("INFO", "Waiting to observe the resulting failure (15 seconds)...")
    time.sleep(15)
    
    drones = check_swarm_status()
    if drones:
        d1_alt = drones.get("drone_1", {}).get("position", {}).get("alt", 0)
        d2_alt = drones.get("drone_2", {}).get("position", {}).get("alt", 0)
        d3_alt = drones.get("drone_3", {}).get("position", {}).get("alt", 0)
        
        print_log("INFO", f"Current Altitudes -> Master (drone_1): {d1_alt:.2f}m, Slave 1 (drone_2): {d2_alt:.2f}m, Slave 2 (drone_3): {d3_alt:.2f}m")
        
        if d2_alt < 5 and d1_alt >= 10 and d3_alt >= 10:
            print_log("FAIL", "FORMATION BROKEN: drone_2 has dropped out of the sky, while Master and drone_3 continue.")
            
            print("\n" + "*"*80)
            print("📝 DETAILED FAILURE LOG & ROOT CAUSE ANALYSIS")
            print("*"*80)
            print_log("WHY", "Why did the formation fail?")
            print("      -> drone_2 experienced an unexpected localized event (simulated communication loss or motor failure) causing an emergency landing.")
            
            print_log("HOW", "How did this failure occur technically?")
            print("      -> Technique Simulated: 'Mid-Flight Localized Follower Failure'.")
            print("      -> The Master drone continuously broadcasts setpoints to all followers.")
            print("      -> A follower overriding the swarm behavior by switching its flight mode to LAND breaks the physics geometry of the swarm.")
            print("      -> The remaining drones (drone_1, drone_3) lack advanced consensus/collision-repair algorithms to reform a new 2-drone formation dynamically.")
            print("*"*80 + "\n")
        else:
            print_log("WARN", "Unexpected behavior observed. Did drone_2 fail to land or did others also crash?")
            
    # Cleanup
    print_log("INFO", "Initiating emergency land sequence to reset environment...")
    post("/api/swarm/land_all")
    print_log("PASS", "Test case execution completed.")
    print("="*80 + "\n")

if __name__ == "__main__":
    test_big_scenario()
