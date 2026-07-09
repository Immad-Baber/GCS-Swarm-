# test_swarm_scenarios.py
# ─────────────────────────────────────────────────────────────────────────────
# Week 4 – Behavior-Based Swarm Test Scenarios
#
# This script exercises the swarm commands through the REST API exposed by
# telemetry_server.py.  It is meant to be run while the server and SITL
# instances are already up.
#
# Prerequisites:
#   1. Start 3 SITL drones in WSL:   bash start_sitl.sh 3
#   2. Start the telemetry server:   python telemetry_server.py
#   3. Run this script:              python test_swarm_scenarios.py
#
# Each scenario prints a clear PASS / FAIL result.
# ─────────────────────────────────────────────────────────────────────────────

import requests
import time
import json
import sys

BASE_URL = "http://127.0.0.1:5000"

# ── Helpers ───────────────────────────────────────────────────────────────

def post(endpoint, body=None):
    """POST to an API endpoint and return the parsed JSON response."""
    url = f"{BASE_URL}{endpoint}"
    print(f"  → POST {url}  body={json.dumps(body) if body else '{}'}")
    try:
        resp = requests.post(url, json=body or {}, timeout=120)
        data = resp.json()
        print(f"  ← {resp.status_code}: {json.dumps(data, indent=2)}")
        return data
    except Exception as e:
        print(f"  ← ERROR: {e}")
        return None


def get(endpoint):
    """GET from an API endpoint and return the parsed JSON response."""
    url = f"{BASE_URL}{endpoint}"
    print(f"  → GET {url}")
    try:
        resp = requests.get(url, timeout=30)
        data = resp.json()
        print(f"  ← {resp.status_code}: {json.dumps(data, indent=2)}")
        return data
    except Exception as e:
        print(f"  ← ERROR: {e}")
        return None


def check_all_ok(results: dict) -> bool:
    """Check that every value in a results dict is True."""
    return all(v is True for v in results.values())


def separator(title):
    print()
    print("=" * 70)
    print(f"  {title}")
    print("=" * 70)
    print()


# ── Scenario 1: Swarm Arm + Takeoff + Land ───────────────────────────────

def scenario_1():
    """
    SCENARIO 1: Full swarm lifecycle
    Steps:
      1. Connect 3 drones
      2. Arm all drones
      3. Takeoff all drones to 10m
      4. Wait 15 seconds (hover)
      5. Land all drones
    """
    separator("SCENARIO 1: Swarm Arm → Takeoff → Land")
    passed = True

    # Step 1: Connect
    print("[Step 1] Connecting 3 drones...")
    data = post("/api/swarm/connect", {"num_drones": 3})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: Could not connect swarm")
        return False
    if not check_all_ok(data.get("results", {})):
        print("⚠ WARNING: Not all drones connected successfully")
        passed = False
    print("✅ Swarm connected\n")

    # Step 2: Arm all
    print("[Step 2] Arming all drones...")
    data = post("/api/swarm/arm_all")
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: arm_all failed")
        return False
    if not check_all_ok(data.get("results", {})):
        print("⚠ WARNING: Not all drones armed")
        passed = False
    print("✅ All drones armed\n")

    # Step 3: Takeoff all
    print("[Step 3] Taking off all drones to 10m...")
    data = post("/api/swarm/takeoff_all", {"altitude": 10})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: takeoff_all failed")
        return False
    if not check_all_ok(data.get("results", {})):
        print("⚠ WARNING: Not all drones took off")
        passed = False
    print("✅ All drones taking off\n")

    # Step 4: Hover
    print("[Step 4] Hovering for 15 seconds...")
    time.sleep(15)
    print("✅ Hover complete\n")

    # Check status
    print("[Check] Getting swarm status...")
    status = get("/api/swarm/status")
    if status and status.get("drones"):
        for did, info in status["drones"].items():
            pos = info.get("position", {})
            armed = info.get("armed", "?")
            mode = info.get("mode", "?")
            print(f"  {did}: mode={mode}, armed={armed}, "
                  f"alt={pos.get('alt', '?')}m")

    # Step 5: Land all
    print("\n[Step 5] Landing all drones...")
    data = post("/api/swarm/land_all")
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: land_all failed")
        return False
    print("✅ All drones landing\n")

    return passed


# ── Scenario 2: Individual Drone Control ──────────────────────────────────

def scenario_2():
    """
    SCENARIO 2: Control a single drone independently
    Steps:
      1. Connect 3 drones (if not already connected)
      2. Arm ONLY drone_2
      3. Takeoff drone_2 to 10m
      4. Wait 10 seconds
      5. Land drone_2
      6. Verify drone_1 and drone_3 did NOT change state
    """
    separator("SCENARIO 2: Individual Drone Control (drone_2)")
    passed = True

    # Step 1: Connect (may already be connected from scenario 1)
    print("[Step 1] Connecting swarm...")
    data = post("/api/swarm/connect", {"num_drones": 3})
    if data is None or data.get("status") != "ok":
        print("⚠ Connect returned non-ok (drones may already be connected)")
    print()

    # Step 2: Arm drone_2
    print("[Step 2] Arming drone_2 only...")
    data = post("/api/drone/drone_2/arm")
    if data is None or not data.get("armed"):
        print("❌ FAIL: drone_2 arm failed")
        return False
    print("✅ drone_2 armed\n")

    # Step 3: Takeoff drone_2
    print("[Step 3] Taking off drone_2 to 10m...")
    data = post("/api/drone/drone_2/takeoff", {"altitude": 10})
    if data is None or not data.get("takeoff"):
        print("❌ FAIL: drone_2 takeoff failed")
        return False
    print("✅ drone_2 taking off\n")

    # Step 4: Wait
    print("[Step 4] Waiting 10 seconds...")
    time.sleep(10)

    # Step 5: Verify drone_1 status (should NOT be armed)
    print("[Step 5] Verifying drone_1 is NOT armed...")
    status = get("/api/drone/drone_1/status")
    if status and status.get("drone", {}).get("armed") is True:
        print("⚠ WARNING: drone_1 is unexpectedly armed!")
        passed = False
    else:
        print("✅ drone_1 is not armed (correct)\n")

    # Step 6: Land drone_2
    print("[Step 6] Landing drone_2...")
    data = post("/api/drone/drone_2/land")
    if data is None or not data.get("landed"):
        print("❌ FAIL: drone_2 land failed")
        return False
    print("✅ drone_2 landed\n")

    return passed


# ── Scenario 3: Formation Flight ─────────────────────────────────────────

def scenario_3():
    """
    SCENARIO 3: Triangle formation flight
    Steps:
      1. Connect 3 drones
      2. Arm all
      3. Takeoff all to 10m
      4. Move swarm to triangle formation (10m spacing)
      5. Log formation distances
      6. Wait 10 seconds
      7. Land all
    """
    separator("SCENARIO 3: Triangle Formation Flight")
    passed = True

    # Step 1: Connect
    print("[Step 1] Connecting 3 drones...")
    data = post("/api/swarm/connect", {"num_drones": 3})
    if data is None or data.get("status") != "ok":
        print("⚠ Connect returned non-ok (may be already connected)")
    print()

    # Step 2: Arm all
    print("[Step 2] Arming all drones...")
    data = post("/api/swarm/arm_all")
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: arm_all failed")
        return False
    print("✅ All drones armed\n")

    # Step 3: Takeoff all
    print("[Step 3] Taking off all drones to 10m...")
    data = post("/api/swarm/takeoff_all", {"altitude": 10})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: takeoff_all failed")
        return False
    print("✅ All drones taking off\n")

    # Wait for drones to reach altitude
    print("[Wait] Waiting 15 seconds for altitude stabilization...")
    time.sleep(15)

    # Step 4: Formation
    print("[Step 4] Moving to TRIANGLE formation (10m spacing)...")
    data = post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: formation command failed")
        return False
    print("✅ Formation command sent\n")

    # Print target positions
    if data.get("target_positions"):
        print("  Target positions:")
        for did, pos in data["target_positions"].items():
            print(f"    {did}: lat={pos['lat']:.6f}, lon={pos['lon']:.6f}, alt={pos['alt']:.1f}m")
        print()

    # Print inter-drone distances
    if data.get("inter_drone_distances"):
        print("  Inter-drone distances:")
        for pair, dist in data["inter_drone_distances"].items():
            print(f"    {pair}: {dist}m")
        print()

    # Step 5: Get live distances
    print("[Step 5] Fetching live formation distances...")
    dist_data = get("/api/swarm/formation/distances")
    if dist_data and dist_data.get("distances"):
        for pair, dist in dist_data["distances"].items():
            print(f"  📏 {pair}: {dist}m")
    print()

    # Step 6: Wait
    print("[Step 6] Hovering in formation for 10 seconds...")
    time.sleep(10)

    # Step 7: Land all
    print("[Step 7] Landing all drones...")
    data = post("/api/swarm/land_all")
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: land_all failed")
        return False
    print("✅ All drones landing\n")

    # Check formation log
    print("[Check] Reading formation log...")
    log_data = get("/api/swarm/formation/log")
    if log_data and log_data.get("entries"):
        print(f"  📄 Formation log has {len(log_data['entries'])} entries")
    else:
        print("  ⚠ No formation log entries found")

    return passed


# ── Scenario 4: Mission Robustness & Disconnect Handling ───────────────────

def scenario_4():
    """
    SCENARIO 4: Runtime Mission Selection & Failure Recovery
    Steps:
      1. Connect 3 drones
      2. Arm all
      3. Takeoff all to 15m using mission2.json
      4. Wait 15 seconds (start following waypoints)
      5. Force land drone_2 to simulate failure/disconnect
      6. Wait 15 seconds. Verify drone_1 and drone_3 continue.
      7. Recover drone_2 (Takeoff individually with mission2.json)
      8. Wait 15 seconds. Verify drone_2 is flying again.
      9. Land all drones.
    """
    separator("SCENARIO 4: Mission Selection & Failure Recovery")
    passed = True

    print("[Step 1] Connecting 3 drones...")
    post("/api/swarm/connect", {"num_drones": 3})
    
    print("\n[Step 2] Arming all drones...")
    post("/api/swarm/arm_all")

    print("\n[Step 3] Taking off all to 15m with mission2.json...")
    data = post("/api/swarm/takeoff_all", {"altitude": 15, "mission": "mission2.json"})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: takeoff_all with mission failed")
        return False

    print("\n[Step 4] Waiting 20 seconds for drones to take off and start mission...")
    time.sleep(20)

    print("\n[Step 5] Simulating failure: Forcing drone_2 to land...")
    post("/api/drone/drone_2/land")

    print("\n[Step 6] Waiting 15 seconds. Verifying swarm handles the failure...")
    time.sleep(15)
    
    status = get("/api/swarm/status")
    if status and status.get("drones"):
        d1 = status["drones"].get("drone_1", {})
        d2 = status["drones"].get("drone_2", {})
        d3 = status["drones"].get("drone_3", {})
        
        if not (d1.get("armed") and d3.get("armed")):
            print("❌ FAIL: drone_1 or drone_3 stopped when drone_2 failed.")
            passed = False
        else:
            print("✅ drone_1 and drone_3 continued successfully.")
            
        if d2.get("armed") and d2.get("position", {}).get("alt", 0) > 2:
            print("⚠ WARNING: drone_2 is still armed and high. Did it land?")
        else:
            print("✅ drone_2 correctly aborted and landed.")

    print("\n[Step 7] Recovering drone_2 with mission2.json...")
    post("/api/drone/drone_2/arm")
    post("/api/drone/drone_2/takeoff", {"altitude": 15, "mission": "mission2.json"})

    print("\n[Step 8] Waiting 20 seconds for drone_2 to catch up...")
    time.sleep(20)

    status = get("/api/swarm/status")
    if status and status.get("drones"):
        d2 = status["drones"].get("drone_2", {})
        if d2.get("armed") and d2.get("position", {}).get("alt", 0) > 10:
            print("✅ drone_2 successfully recovered and rejoined the mission.")
        else:
            print("❌ FAIL: drone_2 failed to recover.")
            passed = False

    print("\n[Step 9] Landing all drones...")
    post("/api/swarm/land_all")
    
    return passed


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    print()
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║          WEEK 4 — SWARM BEHAVIOR TEST SCENARIOS                ║")
    print("╠══════════════════════════════════════════════════════════════════╣")
    print("║  Prerequisites:                                                ║")
    print("║    1. Start SITL:   bash start_sitl.sh 3                       ║")
    print("║    2. Start server: python telemetry_server.py                 ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()

    # Parse which scenario to run
    if len(sys.argv) > 1:
        scenario_num = int(sys.argv[1])
        scenarios = {1: scenario_1, 2: scenario_2, 3: scenario_3, 4: scenario_4}
        if scenario_num in scenarios:
            result = scenarios[scenario_num]()
            print()
            print(f"{'✅ PASSED' if result else '❌ FAILED'} — Scenario {scenario_num}")
        else:
            print(f"Unknown scenario: {scenario_num}. Choose 1, 2, 3, or 4.")
        return

    # Run all scenarios
    results = {}

    results[1] = scenario_1()
    # Wait between scenarios
    print("\n⏳ Waiting 10 seconds between scenarios...\n")
    time.sleep(10)

    results[2] = scenario_2()
    print("\n⏳ Waiting 10 seconds between scenarios...\n")
    time.sleep(10)

    results[3] = scenario_3()
    
    print("\n⏳ Waiting 10 seconds between scenarios...\n")
    time.sleep(10)
    
    results[4] = scenario_4()

    # Summary
    separator("TEST RESULTS SUMMARY")
    for num, passed in results.items():
        icon = "✅ PASSED" if passed else "❌ FAILED"
        print(f"  Scenario {num}: {icon}")

    all_passed = all(results.values())
    print()
    print(f"Overall: {'✅ ALL PASSED' if all_passed else '❌ SOME FAILED'}")
    print()


if __name__ == "__main__":
    main()
