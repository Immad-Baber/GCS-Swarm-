import requests
import time
import json
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

def test_leader_master_slave():
    print("\n" + "="*80)
    print("🚀 LEADER / MASTER / SLAVE - COMPREHENSIVE TEST CASE (PASS & FAIL SCENARIOS)")
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
    # Phase 1: The Passed Scenario (Perfect Execution)
    # ---------------------------------------------------------------------
    print("\n" + "-"*80)
    print("🌟 PHASE 1: PASSED SCENARIO (Ideal Leader-Follower Execution)")
    print("-"*80)
    
    print_log("INFO", "Master commands all Slaves to Arm.")
    post("/api/swarm/arm_all")
    time.sleep(2)
    
    print_log("INFO", "Master initiates takeoff for the entire formation (Altitude: 10m)")
    post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
    
    print_log("INFO", "Waiting for formation to stabilize (15 seconds)...")
    time.sleep(15)
    
    drones = check_swarm_status()
    all_passed = True
    if drones:
        for did in ["drone_1", "drone_2", "drone_3"]:
            alt = drones.get(did, {}).get("position", {}).get("alt", 0)
            if alt >= 5:
                print_log("PASS", f"{did} successfully reached operational altitude: {alt:.2f}m")
            else:
                print_log("FAIL", f"{did} failed to reach altitude (Current: {alt:.2f}m)")
                all_passed = False
    
    if all_passed:
        print_log("PASS", "PHASE 1 SUCCESS: Master and Slaves successfully executed the coordinated takeoff.")
    else:
        print_log("FAIL", "PHASE 1 FAILURE: One or more drones failed to reach altitude.")

    print_log("INFO", "Master commanding all drones to land to prepare for Phase 2...")
    post("/api/swarm/land_all")
    print_log("INFO", "Waiting for drones to land safely (20 seconds)...")
    time.sleep(20)

    # ---------------------------------------------------------------------
    # Phase 2: The Failed Scenario (Intentional Disruption)
    # ---------------------------------------------------------------------
    print("\n" + "-"*80)
    print("💥 PHASE 2: FAILED SCENARIO (Intentional Leader-Follower Disruption)")
    print("-"*80)
    
    print_log("INFO", "Simulating a communication failure: Master fails to arm the Slaves before takeoff.")
    
    # We explicitly only arm drone_1 (Master), leaving Slaves unarmed
    print_log("INFO", "Master (drone_1) arms itself, but fails to transmit arm signal to Slaves.")
    post("/api/drone/drone_1/arm")
    time.sleep(2)
    
    print_log("INFO", "Master attempts to execute a formation takeoff command.")
    post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
    
    print_log("INFO", "Waiting to observe the resulting failure (10 seconds)...")
    time.sleep(10)
    
    drones = check_swarm_status()
    if drones:
        d1_alt = drones.get("drone_1", {}).get("position", {}).get("alt", 0)
        d2_alt = drones.get("drone_2", {}).get("position", {}).get("alt", 0)
        d3_alt = drones.get("drone_3", {}).get("position", {}).get("alt", 0)
        
        print_log("INFO", f"Current Altitudes -> Master (drone_1): {d1_alt:.2f}m, Slave 1 (drone_2): {d2_alt:.2f}m, Slave 2 (drone_3): {d3_alt:.2f}m")
        
        if d2_alt < 2 and d3_alt < 2 and d1_alt >= 5:
            print_log("FAIL", "FORMATION BROKEN: Master took off, but Slaves remained grounded.")
            
            print("\n" + "*"*80)
            print("📝 DETAILED FAILURE LOG & ROOT CAUSE ANALYSIS")
            print("*"*80)
            print_log("WHY", "Why did the formation fail?")
            print("      -> The Slaves (drone_2, drone_3) did not acknowledge the takeoff command and remained on the ground, leaving the Master (drone_1) isolated in the air.")
            
            print_log("HOW", "How did this failure occur technically?")
            print("      -> Technique Simulated: 'Pre-flight State Mismatch'.")
            print("      -> In a Leader/Master-Slave architecture, synchronized state transitions are mandatory.")
            print("      -> The flight controller firmware strictly requires a vehicle to be in an 'ARMED' state before accepting a 'TAKEOFF' command.")
            print("      -> By skipping the arming sequence for the Slaves, the Master's takeoff command was rejected by the Slaves' internal safety checks.")
            print("      -> As a result, the decentralised / swarm takeoff loop executed partially, breaking the physical coherence of the Master-Slave formation.")
            print("*"*80 + "\n")
        else:
            print_log("WARN", "Unexpected behavior observed. Did all drones manage to take off anyway?")
            
    # Cleanup
    print_log("INFO", "Initiating emergency land sequence to reset environment...")
    post("/api/swarm/land_all")
    print_log("PASS", "Test case execution completed.")
    print("="*80 + "\n")

if __name__ == "__main__":
    test_leader_master_slave()
