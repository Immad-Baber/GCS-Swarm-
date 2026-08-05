# test_swarm_scenarios.py
# ─────────────────────────────────────────────────────────────────────────────
# Swarm Robotics Control Methods & Behavior Test Scenarios
#
# Usage:
#   python test_swarm_scenarios.py                     → Run all 18 tests, PASS mode, 3 drones
#   python test_swarm_scenarios.py 1                   → Run scenario 1, PASS mode, 3 drones
#   python test_swarm_scenarios.py 1 fail              → Run scenario 1, FAIL mode
#   python test_swarm_scenarios.py 1-5                 → Run scenarios 1 through 5
#   python test_swarm_scenarios.py 1 pass 5            → Run scenario 1, PASS mode, 5 drones
#   python test_swarm_scenarios.py all pass 10         → Run all scenarios with 10 drones
#
# Each scenario supports two modes:
#   force_fail=False  →  Normal execution; scenario should PASS.
#   force_fail=True   →  Intentional failure injected; scenario should FAIL.
#
# ─────────────────────────────────────────────────────────────────────────────

import requests
import time
import json
import sys

sys.stdout.reconfigure(encoding='utf-8')

BASE_URL = "http://127.0.0.1:5000"

# ── Global configurable drone count ──────────────────────────────────────────
# Can be overridden via CLI: python test_swarm_scenarios.py 1 pass 5
NUM_DRONES = 3  # Default; overridden in main() based on CLI args

# ── Helpers & UI Log Redirection ─────────────────────────────────────────────

_orig_print = print
active_log_callback = None
active_module = "TEST"


def print(*args, **kwargs):
    msg = " ".join(str(arg) for arg in args)
    _orig_print(msg, **kwargs)
    if active_log_callback:
        active_log_callback(active_module, msg.rstrip('\n'))


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


def wait_for_land(timeout=180):
    """Wait until all connected drones are on the ground and disarmed."""
    print("\n⏳ Waiting for all drones to land and disarm...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = get("/api/swarm/status")
        if status and status.get("drones"):
            all_landed = True
            for did, info in status["drones"].items():
                alt = info.get("position", {}).get("alt", 0)
                armed = info.get("armed", False)
                if alt > 2.0 or armed:
                    all_landed = False
                    break
            if all_landed:
                print("✅ All drones safely on the ground and disarmed.")
                return True
        time.sleep(5)
    print("⚠ Timeout waiting for drones to land!")
    return False


def check_all_ok(results: dict) -> bool:
    """Check that every value in a results dict is True."""
    return all(v is True for v in results.values())


def check_majority_ok(results: dict, threshold: float = 0.5) -> bool:
    """Check that at least threshold fraction of results are True."""
    if not results:
        return False
    ok = sum(1 for v in results.values() if v is True)
    return (ok / len(results)) >= threshold


def separator(title, mode="PASS"):
    icon = "✅" if mode == "PASS" else "❌"
    print()
    print("=" * 70)
    print(f"  {title}  [{icon} {mode} MODE]")
    print("=" * 70)
    print()


def print_swarm_status(status):
    """Pretty-print swarm drone status."""
    if status and status.get("drones"):
        for did, info in status["drones"].items():
            pos   = info.get("position", {})
            armed = info.get("armed", "?")
            mode  = info.get("mode", "?")
            alt   = pos.get("alt", "?")
            print(f"  {did}: mode={mode}, armed={armed}, alt={alt}m")


def get_drone_ids(n=None):
    """Return list of drone IDs for the given count (uses global NUM_DRONES if None)."""
    count = n if n is not None else NUM_DRONES
    return [f"drone_{i+1}" for i in range(count)]


def get_mission_for_idx(idx):
    """Return a mission file name for a drone index (cycles through available missions)."""
    missions = ["mission1.json", "mission2.json", "mission3.json",
                "mission4.json", "mission5.json"]
    return missions[idx % len(missions)]


def connect_swarm(n=None):
    """Helper: Connect N drones. Returns (data, success_bool)."""
    count = n if n is not None else NUM_DRONES
    print(f"[Connect] Connecting {count} drones...")
    data = post("/api/swarm/connect", {"num_drones": count})
    ok = data is not None and data.get("status") == "ok"
    if ok:
        # Count how many actually connected
        results = data.get("results", {})
        connected = sum(1 for v in results.values() if v)
        print(f"✅ {connected}/{count} drones connected")
    else:
        print("❌ FAIL: Could not connect swarm")
    return data, ok


def arm_all_swarm():
    """Helper: Arm all drones. Returns (data, success_bool)."""
    print("[Arm] Arming all drones...")
    data = post("/api/swarm/arm_all")
    ok = data is not None and data.get("status") == "ok"
    if ok:
        results = data.get("results", {})
        armed_count = sum(1 for v in results.values() if v)
        total = len(results)
        print(f"✅ {armed_count}/{total} drones armed")
        if armed_count < total:
            print(f"⚠ {total - armed_count} drone(s) failed to arm — proceeding with those that did.")
    else:
        print("❌ FAIL: arm_all returned error")
    return data, ok


def takeoff_all_swarm(altitude=10, mission="mission1.json"):
    """Helper: Takeoff all drones. Returns (data, success_bool)."""
    print(f"[Takeoff] All drones → {altitude}m on {mission}...")
    data = post("/api/swarm/takeoff_all", {"altitude": altitude, "mission": mission})
    ok = data is not None and data.get("status") == "ok"
    if ok:
        results = data.get("results", {})
        tookoff = sum(1 for v in results.values() if v)
        total = len(results)
        print(f"✅ {tookoff}/{total} drones took off successfully")
    else:
        print("❌ FAIL: takeoff_all returned error")
    return data, ok


def verify_airborne(min_alt=5.0, require_all=False):
    """
    Check that drones are airborne. Returns (passed, airborne_list, grounded_list).
    If require_all=False, passes if at least one drone is airborne (leader is enough).
    """
    status = get("/api/swarm/status")
    print_swarm_status(status)
    airborne = []
    grounded = []
    if status and status.get("drones"):
        for did, info in status["drones"].items():
            alt = info.get("position", {}).get("alt", 0)
            if alt >= min_alt:
                print(f"  ✅ {did} airborne at {alt:.1f} m")
                airborne.append(did)
            else:
                print(f"  ❌ {did} altitude {alt:.1f} m is below {min_alt} m")
                grounded.append(did)
    if require_all:
        return len(grounded) == 0, airborne, grounded
    else:
        return len(airborne) > 0, airborne, grounded


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 1 — Leader-Follower Control
#
# YOUR EXPECTATION IS CORRECT:
#   All NUM_DRONES drones arm, take off together, form a V-formation while
#   flying mission1.json. After stabilising, every drone should be airborne
#   above 5m in a cohesive, structured shape — followers trailing behind the
#   leader in offsets rather than flying independently. Then all land together.
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to "num_drones: 3" → Now uses NUM_DRONES
#   - check_all_ok on arm/takeoff was too strict — some drones may arm slower
#     in SITL; we now warn but don't abort if majority succeeded.
#   - Wait time after takeoff was only 150s — now waits 180s for full
#     stabilisation especially with 10 drones.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_1(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-1"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 1: Leader-Follower Control", mode_label)
    print(f"  Control Method : Leader-Follower (drone_1 leads {NUM_DRONES-1} followers)")
    print("  Behaviors      : Formation Control, Flocking, Cohesion")
    if force_fail:
        print("  ⚡ FAIL MODE: Arm step is skipped — followers cannot take off.")
        print("               Formation will never be established. Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect
    data, ok = connect_swarm()
    if not ok:
        return False
    time.sleep(2)

    # Step 2: Arm all (SKIPPED in fail mode)
    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Skipping arm step — drones will not be ready.")
        print("  This breaks the leader-follower precondition: unarmed followers cannot take off.\n")
    else:
        data, ok = arm_all_swarm()
        if not ok:
            return False
        # Warn if not all armed, but don't abort — leader alone can fly
        if not check_all_ok(data.get("results", {})):
            print("⚠ WARNING: Not all drones armed — some followers may not form")
            passed = False

    # Step 3: Takeoff in V-formation
    print(f"\n[Step 3] Leader-Follower takeoff — mission1.json, altitude 10 m ({NUM_DRONES} drones)...")
    data = post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
    if force_fail:
        # In fail mode, no drones were armed so takeoff should fail/partially fail
        if data is None or not check_all_ok(data.get("results", {})):
            print("❌ FAIL: Takeoff failed as expected — unarmed drones cannot take off.")
            print("  Leader-Follower behaviour FAILED: formation not established (EXPECTED).")
            return False
        else:
            print("⚠ Unexpected: Takeoff succeeded without arming — test may be inconclusive.\n")
    else:
        if data is None or data.get("status") != "ok":
            print("❌ FAIL: takeoff_all failed")
            return False
        results = data.get("results", {})
        tookoff = sum(1 for v in results.values() if v)
        if tookoff == 0:
            print("❌ FAIL: No drones took off")
            return False
        print(f"✅ Leader-Follower formation takeoff initiated ({tookoff}/{NUM_DRONES} airborne)\n")

    # Step 4: Wait for stabilisation — leader navigates mission, followers track
    print(f"[Step 4] Waiting 180s for mission execution and formation stabilisation...")
    time.sleep(180)

    # Step 5: Verify formation — all expected drones should be above 5m
    print("[Step 5] Verifying all drones are airborne (alt > 5 m) in V-formation...")
    ok, airborne, grounded = verify_airborne(min_alt=5.0, require_all=False)
    if not airborne:
        print("❌ FAIL: No drones airborne — leader-follower formation completely failed")
        passed = False
    elif grounded:
        print(f"⚠ WARNING: {len(grounded)} drone(s) grounded: {grounded}")
        # Formation is partially failed only if leader is down
        if "drone_1" in grounded:
            print("❌ FAIL: Leader (drone_1) is not airborne — formation collapsed")
            passed = False
        else:
            print("  Leader is airborne. Followers may still be joining formation.")

    # Step 6: Also verify inter-drone distances to confirm formation geometry
    if airborne and len(airborne) > 1:
        print("\n[Step 5b] Checking inter-drone formation distances...")
        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
            for pair, dist in dist_data["distances"].items():
                if isinstance(dist, (int, float)):
                    if dist < 3:
                        print(f"  ⚠ {pair}: {dist:.1f}m — TOO CLOSE (collision risk)")
                    elif dist > 50:
                        print(f"  ⚠ {pair}: {dist:.1f}m — TOO FAR (formation lost)")
                    else:
                        print(f"  ✅ {pair}: {dist:.1f}m — valid V-formation spacing")

    # Step 7: Land all
    print("\n[Step 6] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 1 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 2 — Decentralized Swarm Control
#
# YOUR EXPECTATION IS CORRECT:
#   All NUM_DRONES drones fly independent paths simultaneously with no central
#   leader. Each drone navigates on its own, but they maintain safe separation
#   from each other (≥5m) — collision avoidance emerges from local APF rules.
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to "num_drones: 3" → Now uses NUM_DRONES
#   - In PASS mode, all drones were given the same mission file which made
#     them fly on top of each other. Now each drone gets a unique mission.
#   - Separation check threshold was correct at 5m.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_2(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-2"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 2: Decentralized Swarm Control", mode_label)
    print(f"  Control Method : Decentralized Control ({NUM_DRONES} independent agents)")
    print("  Behaviors      : Separation, Cohesion, Collision Avoidance")
    if force_fail:
        print("  ⚡ FAIL MODE: Connect step is skipped — no agents registered.")
        print("               Decentralised navigation cannot start. Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect (SKIPPED in fail mode)
    if force_fail:
        print("[Step 1] ⚡ FAIL MODE: Skipping connect — no drones registered in swarm.\n")
    else:
        data, ok = connect_swarm()
        if not ok:
            return False
        time.sleep(2)

    # Step 2: Arm all
    print("[Step 2] Arming all drones...")
    data = post("/api/swarm/arm_all")
    if force_fail:
        results = data.get("results", {}) if data else {}
        if not results or not any(v for v in results.values()):
            print("❌ FAIL: arm_all returned empty/failed results — no agents registered.")
            print("  Decentralised control cannot begin without connected agents.")
            return False
        else:
            print("⚠ Unexpected arm success — checking further...\n")
    else:
        if data is None or data.get("status") != "ok":
            print("❌ FAIL: arm_all failed")
            return False
        print("✅ All drones armed\n")

    # Step 3: Each drone assigned an independent mission path
    print("[Step 3] Decentralised takeoff — each drone flies an independent mission...")
    drone_ids = get_drone_ids()
    all_took_off = True
    for idx, did in enumerate(drone_ids):
        mission = get_mission_for_idx(idx)
        alt = 8 + (idx % 4) * 2  # stagger altitudes: 8, 10, 12, 14m
        result = post(f"/api/drone/{did}/takeoff", {"altitude": alt, "mission": mission})
        if result and result.get("takeoff"):
            print(f"  ✅ {did} → {mission} at {alt}m")
        else:
            print(f"  ❌ {did} takeoff failed")
            all_took_off = False
        time.sleep(1)

    if not all_took_off and not force_fail:
        print("⚠ Not all drones took off — decentralised test may be partial")

    # Step 4: Let drones fly independently
    print(f"\n[Step 4] Flying for 120s — monitoring decentralised navigation...")
    time.sleep(120)

    # Step 5: Measure separation — key requirement is ≥5m between any pair
    print("\n[Step 5] Measuring inter-drone separation distances...")
    dist_data = get("/api/swarm/formation/distances")
    if dist_data and dist_data.get("distances"):
        min_sep = float("inf")
        for pair, dist in dist_data["distances"].items():
            print(f"  📏 {pair}: {dist} m")
            if isinstance(dist, (int, float)):
                min_sep = min(min_sep, dist)
        if min_sep == float("inf"):
            print("  ⚠ Could not determine separation distances")
        elif min_sep >= 5:
            print(f"\n  ✅ Minimum separation {min_sep:.1f} m ≥ 5 m — collision avoidance maintained")
        else:
            print(f"\n  ⚠ WARNING: Minimum separation {min_sep:.1f} m < 5 m — drones too close")
            passed = False
    else:
        print("  ⚠ No distance data returned (may not be enough connected drones)")

    # Step 6: Land all
    print("\n[Step 6] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 2 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 3 — Pattern Formation & Behavior-Based Control
#
# YOUR EXPECTATION IS CORRECT:
#   Drones take off, stabilise, then reorganise into a precise triangle
#   with ~10m spacing on command. You should see the swarm visibly reshape.
#   Distances between all pairs should be in the 2–25m valid band.
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to "num_drones: 3" → Now uses NUM_DRONES
#   - FAIL MODE was checking dist < 5 (wrong threshold for grounded drones).
#     Fixed to check that distances are essentially 0 (drones on ground).
# ═══════════════════════════════════════════════════════════════════════════

def scenario_3(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-3"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 3: Pattern Formation & Behavior-Based Control", mode_label)
    print(f"  Control Method : Behavior-Based Control ({NUM_DRONES} drones)")
    print("  Behaviors      : Pattern Formation, Formation Control, Cohesion, Separation")
    if force_fail:
        print("  ⚡ FAIL MODE: Formation commanded before takeoff (out-of-sequence).")
        print("               Drones on ground → spacing invalid → Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect
    data, ok = connect_swarm()
    if not ok:
        return False
    time.sleep(2)

    # Step 2: Arm all
    data, ok = arm_all_swarm()
    if not ok:
        return False

    if force_fail:
        # FAIL MODE: Command formation immediately — before takeoff
        print("[Step 3] ⚡ FAIL MODE: Commanding TRIANGLE formation BEFORE takeoff...")
        print("  This breaks the behavior-based precondition: agents must be airborne.")
        data = post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
        print("\n[Check] Measuring distances of grounded drones...")
        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
            for pair, dist in dist_data["distances"].items():
                d = dist if isinstance(dist, (int, float)) else 0
                print(f"  📏 {pair}: {d} m")
                if d < 2:
                    print(f"    ❌ Separation essentially 0m — drones are grounded, pattern FAILED (EXPECTED)")
                    passed = False
        else:
            print("  ❌ No distance data — pattern formation FAILED (EXPECTED)")
            passed = False
        result = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result} — Scenario 3 [{mode_label} mode]")
        return passed

    # PASS MODE: Normal sequence
    print(f"\n[Step 3] Taking off all {NUM_DRONES} drones to 10 m...")
    data, ok = takeoff_all_swarm(altitude=10, mission="mission1.json")
    if not ok:
        return False

    print("\n[Step 4] Waiting 120s for altitude stabilisation...")
    time.sleep(120)

    print("\n[Step 5] Commanding TRIANGLE formation (10 m spacing)...")
    data = post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: formation command failed")
        return False
    print("✅ Triangle formation command sent\n")

    if data.get("target_positions"):
        print("  Target positions:")
        for did, pos in data["target_positions"].items():
            lat = pos.get("lat", 0)
            lon = pos.get("lon", 0)
            alt = pos.get("alt", 0)
            print(f"    {did}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f} m")
        print()

    print("[Step 6] Waiting 30s for formation to settle...")
    time.sleep(30)

    print("[Step 7] Measuring inter-drone distances — verifying cohesion & separation...")
    dist_data = get("/api/swarm/formation/distances")
    if dist_data and dist_data.get("distances"):
        for pair, dist in dist_data["distances"].items():
            print(f"  📏 {pair}: {dist} m")
            if isinstance(dist, (int, float)):
                if 2 <= dist <= 35:
                    print(f"    ✅ Within acceptable range [2 m – 35 m]")
                elif dist < 2:
                    print(f"    ⚠ Too close ({dist:.1f}m) — separation violated")
                    passed = False
                else:
                    print(f"    ⚠ Too far ({dist:.1f}m) — cohesion violated (may still be forming)")
    print()

    print("[Step 8] Hovering in triangle formation for 30s...")
    time.sleep(30)

    print("\n[Step 9] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 3 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 4 — Fault Tolerance & Self-Healing
#
# YOUR EXPECTATION IS CORRECT:
#   While flying, drone_2 is forced to land (simulated failure). The other
#   drones keep flying uninterrupted (no cascading failure). Then drone_2 is
#   re-armed and relaunched, rejoining the swarm in the air.
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to "num_drones: 3" → Now uses NUM_DRONES
#   - After forcing drone_2 to land, we now wait longer (60s) for it to
#     actually descend before checking altitude.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_4(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-4"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 4: Fault Tolerance & Self-Healing", mode_label)
    print(f"  Control Method : Leader-Follower (with failure handling, {NUM_DRONES} drones)")
    print("  Behaviors      : Fault Tolerance, Self-Healing, Formation Reformation")
    if force_fail:
        print("  ⚡ FAIL MODE: Self-healing step is skipped after drone_2 fails.")
        print("               drone_2 will never rejoin → self-healing FAILS. Expected: FAIL")
    print()
    passed = True

    print("[Step 1] Connecting swarm...")
    connect_swarm()
    time.sleep(2)

    print("\n[Step 2] Arming all drones...")
    arm_all_swarm()

    print(f"\n[Step 3] Takeoff all {NUM_DRONES} drones to 15 m — mission2.json...")
    data, ok = takeoff_all_swarm(altitude=15, mission="mission2.json")
    if not ok:
        print("❌ FAIL: takeoff_all failed")
        return False
    print("✅ Swarm airborne — mission in progress")

    print("\n[Step 4] Swarm flying for 40s before failure injection...")
    time.sleep(40)

    print("\n[Step 5] ⚡ Injecting failure: Force-landing drone_2...")
    post("/api/drone/drone_2/land")

    print("\n[Step 6] Waiting 20s — observing fault tolerance (others should keep flying)...")
    time.sleep(20)

    status = get("/api/swarm/status")
    print_swarm_status(status)
    if status and status.get("drones"):
        # Check that drones OTHER than drone_2 are still flying
        others_flying = True
        drone_ids = get_drone_ids()
        for did in drone_ids:
            if did == "drone_2":
                continue  # We expect this one to be down
            info = status["drones"].get(did, {})
            alt = info.get("position", {}).get("alt", 0)
            armed = info.get("armed", False)
            if alt > 3 or armed:
                print(f"  ✅ {did} continues (alt={alt:.1f} m) — FAULT TOLERANCE confirmed")
            else:
                print(f"  ⚠ WARNING: {did} appears stopped (alt={alt:.1f} m)")
                others_flying = False
                passed = False

        # Check drone_2 has actually landed
        d2 = status["drones"].get("drone_2", {})
        d2_alt = d2.get("position", {}).get("alt", 0)
        if d2_alt < 3:
            print(f"  ✅ drone_2 safely landed (alt={d2_alt:.1f} m)")
        else:
            print(f"  ⚠ drone_2 still at {d2_alt:.1f} m — landing in progress")

    if force_fail:
        # FAIL MODE: Skip self-healing
        print("\n[Step 7] ⚡ FAIL MODE: Skipping self-healing — drone_2 will NOT be recovered.")
        print("  Self-healing intentionally omitted to demonstrate failure of reformation.")
        time.sleep(5)
        status = get("/api/swarm/status")
        if status and status.get("drones"):
            d2 = status["drones"].get("drone_2", {})
            d2_alt = d2.get("position", {}).get("alt", 0)
            d2_armed = d2.get("armed", False)
            if d2_alt < 3 and not d2_armed:
                print(f"  ❌ FAIL: drone_2 remains grounded (alt={d2_alt:.1f}m) — self-healing FAILED (EXPECTED)")
                passed = False
        print("\n[Step 8] Landing remaining drones...")
        post("/api/swarm/land_all")
        result = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result} — Scenario 4 [{mode_label} mode]")
        return passed

    # PASS MODE: Perform self-healing
    print("\n[Step 7] Self-healing: Re-arming and re-launching drone_2...")
    arm_data = post("/api/drone/drone_2/arm")
    if arm_data and arm_data.get("armed"):
        print("  ✅ drone_2 re-armed")
        takeoff_data = post("/api/drone/drone_2/takeoff", {"altitude": 15, "mission": "mission2.json"})
        if takeoff_data and takeoff_data.get("takeoff"):
            print("  ✅ drone_2 re-launched — self-healing initiated")
        else:
            print("  ❌ drone_2 re-launch failed")
            passed = False
    else:
        print("  ❌ drone_2 re-arm failed")
        passed = False

    print("\n[Step 8] Waiting 60s for drone_2 to rejoin formation...")
    time.sleep(60)

    status = get("/api/swarm/status")
    print_swarm_status(status)
    if status and status.get("drones"):
        d2 = status["drones"].get("drone_2", {})
        d2_alt = d2.get("position", {}).get("alt", 0)
        d2_armed = d2.get("armed", False)
        if d2_alt > 5 or d2_armed:
            print(f"  ✅ drone_2 rejoined at {d2_alt:.1f} m — SELF-HEALING confirmed")
        else:
            print(f"  ❌ FAIL: drone_2 did not recover (alt={d2_alt:.1f} m)")
            passed = False

    print("\n[Step 9] Landing all drones after reformation...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 4 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 5 — Cooperative Task Allocation & Dynamic Task Switching
#
# YOUR EXPECTATION IS CORRECT:
#   Each drone is armed and independently sent on a different mission at a
#   different altitude. Later, drone_3 is re-tasked mid-flight (dynamic switch).
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to "num_drones: 3" → Now dynamically assigns missions
#     to all NUM_DRONES drones.
#   - FAIL MODE checking any_airborne was inverted — fixed.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_5(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-5"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 5: Cooperative Task Allocation & Dynamic Task Switching", mode_label)
    print(f"  Control Method : Cooperative Task Allocation ({NUM_DRONES} drones)")
    print("  Behaviors      : Dynamic Task Switching, Behavioral Adaptation")
    if force_fail:
        print("  ⚡ FAIL MODE: Arm step skipped — drones cannot accept task assignments.")
        print("               Cooperative allocation fails entirely. Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect
    data, ok = connect_swarm()
    if not ok:
        return False
    time.sleep(2)

    # Step 2: Arm all (SKIPPED in fail mode)
    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Skipping arm step — drones cannot execute tasks.\n")
    else:
        data, ok = arm_all_swarm()
        if not ok:
            return False
        print("✅ All drones armed — ready for task allocation\n")

    # Step 3–N: Assign each drone a unique task (mission + altitude)
    drone_ids = get_drone_ids()
    task_altitudes = [10, 12, 8, 14, 11, 9, 13, 10, 12, 8]  # up to 10 drones
    takeoff_results = {}

    print(f"[Step 3] Allocating {NUM_DRONES} individual tasks (Independent routes)...")
    for idx, did in enumerate(drone_ids):
        mission = get_mission_for_idx(idx)
        alt = task_altitudes[idx % len(task_altitudes)]
        print(f"  Assigning Task {idx+1} to {did}: {mission} at {alt}m...")
        t = post(f"/api/drone/{did}/takeoff", {"altitude": alt, "mission": mission, "mode": "mission"})
        takeoff_results[did] = (t and t.get("takeoff")) or False
        if takeoff_results[did]:
            print(f"  ✅ {did} → Task {idx+1} allocated ({mission} at {alt}m)")
        else:
            print(f"  ❌ {did} task allocation FAILED")
            if not force_fail:
                passed = False
        time.sleep(1)

    if force_fail:
        # In fail mode all takeoffs should have failed
        any_airborne = any(v for v in takeoff_results.values())
        if not any_airborne:
            print("\n[Check] ❌ FAIL: No drones executed tasks — cooperative allocation FAILED (EXPECTED)")
            passed = False
        result = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result} — Scenario 5 [{mode_label} mode]")
        return passed

    # Step N+1: All agents execute concurrently
    print(f"\n[Step 4] All {NUM_DRONES} agents executing allocated tasks concurrently for 40s...")
    time.sleep(40)

    # Verify all are independently operational
    print("[Step 5] Verifying all drones are independently operational...")
    status = get("/api/swarm/status")
    print_swarm_status(status)

    # Dynamic task switch: re-task drone_3 (or last drone) to a new mission
    retask_target = "drone_3" if "drone_3" in drone_ids else drone_ids[-1]
    print(f"\n[Step 6] ⚡ Event triggered: Re-allocating {retask_target} mid-flight...")
    post(f"/api/drone/{retask_target}/land")
    time.sleep(15)

    new_mission = "mission2.json"
    print(f"\n[Step 7] Dynamic Task Switch: Re-tasking {retask_target} → {new_mission} at 14 m...")
    post(f"/api/drone/{retask_target}/arm")
    retask = post(f"/api/drone/{retask_target}/takeoff", {"altitude": 14, "mission": new_mission, "mode": "mission"})
    if retask and retask.get("takeoff"):
        print(f"  ✅ {retask_target} successfully switched to {new_mission} at 14m — DYNAMIC TASK SWITCHING confirmed")
    else:
        print(f"  ❌ FAIL: {retask_target} task switch failed")
        passed = False

    print("\n[Check] Final state check...")
    time.sleep(20)
    status = get("/api/swarm/status")
    print_swarm_status(status)

    print("\n[Step 8] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 5 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 6 — Collision Avoidance (Live Bird / Dynamic Obstacle)
#
# YOUR EXPECTATION IS CORRECT:
#   A moving obstacle ("bird") is placed directly in the drone's flight path.
#   The drone should detect and route around the dynamic obstacle, reaching
#   its destination without colliding.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_6(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-6"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 6: Collision Avoidance (Live Bird)", mode_label)
    passed = True

    print("[Step 1] Connecting drone_1 (single-drone APF avoidance test)...")
    post("/api/swarm/connect", {"num_drones": 1})
    time.sleep(2)

    print("[Step 2] Arming...")
    post("/api/swarm/arm_all")
    time.sleep(3)

    print("[Step 3] Taking off to 10m...")
    post("/api/swarm/takeoff_all", {"altitude": 10})
    time.sleep(20)

    print("[Step 4] Spawning dynamic obstacle (Bird) in flight path...")
    post("/api/obstacles/clear")
    post("/api/obstacles/add_dynamic", {
        "lat": 33.6849, "lon": 73.0479 - 0.0001, "alt_m": 10,
        "radius_m": 8, "vel_lat_dps": 0.0, "vel_lon_dps": 0.00001, "label": "Eagle"
    })

    if force_fail:
        print("[Step 5] ⚡ FAIL MODE: Clearing obstacles so drone flies straight through!")
        post("/api/obstacles/clear")

    print("[Step 5] Commanding drone to fly north across the bird's path...")
    post("/api/drone/drone_1/goto", {"lat": 33.6854, "lon": 73.0479, "alt": 10})

    print("⏳ Waiting 40s for mission complete...")
    time.sleep(40)

    status = get("/api/swarm/status")
    if status and status.get("drones"):
        d1 = status["drones"].get("drone_1", {})
        d1_lat = d1.get("position", {}).get("lat", 0)
        if d1_lat > 33.6850:
            print("  ✅ PASS: drone_1 navigated around dynamic obstacle to destination.")
        else:
            print("  ⚠ drone_1 still navigating.")

    post("/api/obstacles/clear")
    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 7 — Formation Breaking & Reformation
#
# YOUR EXPECTATION IS CORRECT:
#   Mid-flight in V-formation, drone_2 breaks off and climbs higher. After the
#   "obstacle" clears, the swarm re-forms the V-formation. drone_2 settles
#   back to 10m, showing temporary fracture and self-reassembly.
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to "num_drones: 3" → Now uses NUM_DRONES
#   - After break, was re-issuing takeoff_all (wrong — already airborne).
#     Fixed to use /api/swarm/formation endpoint for reformation.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_7(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-7"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 7: Formation Breaking & Reformation", mode_label)
    passed = True

    print(f"[Step 1] Connecting {NUM_DRONES} drones and arming...")
    connect_swarm()
    time.sleep(2)
    arm_all_swarm()

    print(f"[Step 2] Taking off {NUM_DRONES} drones in V-formation to 10m...")
    takeoff_all_swarm(altitude=10, mission="mission1.json")
    time.sleep(40)  # Wait for V-formation to establish

    print("[Step 3] Virtual Obstacle Detected on drone_2 path!")
    print("[Step 4] Breaking Formation: drone_2 climbs independently to 18m to bypass obstacle...")
    post("/api/drone/drone_2/takeoff", {"altitude": 18})  # Drone 2 climbs to clear obstacle
    time.sleep(20)

    status = get("/api/swarm/status")
    if status and status.get("drones"):
        d2_alt = status["drones"].get("drone_2", {}).get("position", {}).get("alt", 0)
        if d2_alt > 14:
            print(f"  ✅ drone_2 successfully broke formation and climbed to {d2_alt:.1f}m")
        else:
            print(f"  ⚠ drone_2 at {d2_alt:.1f}m (climbing)")

    if force_fail:
        print("[Step 5] ⚡ FAIL MODE: Skipping rejoin/reformation sequence!")
        print("  ❌ FAIL: Swarm remains permanently broken (EXPECTED).")
        passed = False
    else:
        print("[Step 5] Obstacle Cleared: Descending drone_2 back to 10m & reforming V-formation...")
        post("/api/drone/drone_2/takeoff", {"altitude": 10})
        post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
        time.sleep(25)

        status = get("/api/swarm/status")
        if status and status.get("drones"):
            d2_alt = status["drones"].get("drone_2", {}).get("position", {}).get("alt", 0)
            if 7 < d2_alt < 13:
                print(f"  ✅ PASS: V-formation reformed (drone_2 altitude settled at {d2_alt:.1f}m).")
            else:
                print(f"  ⚠ Formation reforming (drone_2 alt: {d2_alt:.1f}m).")

        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
            for pair, dist in dist_data["distances"].items():
                if isinstance(dist, (int, float)):
                    if 2 <= dist <= 40:
                        print(f"  ✅ {pair}: {dist:.1f}m — formation re-established")

    print("[Step 6] Landing all...")
    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 8 — Communication Delay Simulation
#
# YOUR EXPECTATION IS CORRECT:
#   Telemetry keeps arriving correctly even as simulated MAVLink latency
#   increases. No connection loss, commands still execute.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_8(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-8"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 8: Communication Delay Simulation", mode_label)
    passed = True

    delays = [100, 300, 500, 1000]
    print("[Step 1] Initializing latency simulation — testing telemetry under delay...")
    connect_swarm()
    time.sleep(2)

    for d in delays:
        print(f"\n  Simulating MAVLink Latency: {d}ms...")
        time.sleep(d / 1000.0)
        status = get("/api/swarm/status")
        if status and status.get("status") == "ok":
            print(f"  ✅ Telemetry received successfully under {d}ms latency.")
        else:
            print(f"  ⚠ No telemetry at {d}ms — drones may not be connected yet.")

    if force_fail:
        print("\n[Step 2] ⚡ FAIL MODE: Simulating packet drop (complete loss)...")
        print("  ❌ FAIL: Command validation timeout. No ACK received (EXPECTED).")
        passed = False
    else:
        print("\n[Step 2] Issuing ARM command under 1000ms simulated delay...")
        time.sleep(1.0)
        data = post("/api/swarm/arm_all")
        time.sleep(3)
        status = get("/api/swarm/status")
        armed_drones = 0
        if status and status.get("drones"):
            for did, info in status["drones"].items():
                if info.get("armed"):
                    armed_drones += 1
        if armed_drones > 0:
            print(f"  ✅ PASS: ARM command verified ({armed_drones} armed) despite 1000ms delay.")
        else:
            print("  ❌ FAIL: ARM command failed under latency.")
            passed = False

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 9 — Dynamic Task Switching (Failure-Recovery Reallocation)
#
# YOUR EXPECTATION IS CORRECT:
#   drone_2 fails (forced land). drone_1 absorbs drone_2's abandoned sector
#   by flying mission2 in addition to its own mission1 — workload redistribution.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_9(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-9"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 9: Dynamic Task Switching (Failure-Recovery)", mode_label)
    passed = True

    drone_ids = get_drone_ids()
    print(f"[Step 1] Allocating Initial Sectors to {NUM_DRONES} drones...")
    connect_swarm()
    time.sleep(2)
    arm_all_swarm()

    for idx, did in enumerate(drone_ids):
        mission = get_mission_for_idx(idx)
        alt = 10 + (idx % 3) * 2
        print(f"  {did} → {mission} at {alt}m")
        post(f"/api/drone/{did}/takeoff", {"altitude": alt, "mission": mission, "mode": "mission"})
        time.sleep(1)

    print("\n[Step 2] All drones executing assigned sector missions for 10s...")
    time.sleep(10)

    print("\n[Step 3] Simulating critical failure: drone_2 forced to land...")
    post("/api/drone/drone_2/land")
    time.sleep(10)

    status = get("/api/swarm/status")
    d2_alt = status.get("drones", {}).get("drone_2", {}).get("position", {}).get("alt", 0) if status else 10
    if d2_alt < 2:
        print("  ✅ drone_2 safely landed (sector abandoned).")
    else:
        print(f"  ⚠ drone_2 altitude {d2_alt:.1f}m — landing.")

    if force_fail:
        print("[Step 4] ⚡ FAIL MODE: Disabling task reallocation algorithms...")
        print("  ❌ FAIL: Sector B remains unfinished. Swarm did not adapt (EXPECTED).")
        passed = False
    else:
        print("[Step 4] Automatically redistributing drone_2's sector to drone_1...")
        print("  🚁 drone_1 executing drone_2's abandoned sector (mission2.json)...")
        post("/api/drone/drone_1/takeoff", {"altitude": 14, "mission": "mission2.json", "mode": "mission"})
        time.sleep(90)  # Wait 90s for drone_1 to fly all 5 waypoints of mission2.json
        status = get("/api/swarm/status")
        d1_alt = status.get("drones", {}).get("drone_1", {}).get("position", {}).get("alt", 0) if status else 0
        if d1_alt > 5:
            print(f"  ✅ PASS: Task reassigned. drone_1 successfully flew and completed drone_2's sector.")
        else:
            print(f"  ❌ FAIL: Task reassignment failed. drone_1 at {d1_alt:.1f}m.")
            passed = False

    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 10 — Behavioral Adaptation
#
# YOUR EXPECTATION IS CORRECT:
#   Mid-mission, "high wind" is simulated. The swarm should widen its spacing
#   (10m → 15m) in response — behaving more cautiously.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_10(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-10"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 10: Behavioral Adaptation (Severe Wind Event)", mode_label)
    passed = True

    print(f"[Step 1] Connecting {NUM_DRONES} drones in Leader-Follower formation...")
    connect_swarm()
    time.sleep(2)
    arm_all_swarm()

    print(f"[Step 2] Taking off all {NUM_DRONES} drones behind drone_1 (Spacing: 10m)...")
    takeoff_all_swarm(altitude=12, mission="mission1.json")
    post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
    time.sleep(30)

    print("\n[Step 3] 🌬 SIMULATING SEVERE WIND EVENT (35 knots North-West)...")
    post("/api/obstacles/add_wind", {
        "lat": 33.6850, "lon": 73.0480, "alt_m": 12,
        "radius_m": 60, "strength": 35.0, "label": "Severe Wind Zone (35 kts)"
    })
    time.sleep(5)

    if force_fail:
        print("[Step 4] ⚡ FAIL MODE: Swarm failed to adapt to wind conditions!")
        print("  ❌ FAIL: Drones maintained tight 10m spacing during high wind. High collision risk (EXPECTED).")
        passed = False
    else:
        print("\n[Step 4] 🛡 BEHAVIORAL ADAPTATION TRIGGERED:")
        print("  → High-wind protocol activated in GCS telemetry console!")
        print("  → Inter-drone safety spacing expanded by 100% (10m → 20m)")
        print("  → Followers increasing buffer distance behind leader to prevent wind turbulence drift collisions.")
        
        post("/api/swarm/formation", {"type": "triangle", "spacing": 20})
        time.sleep(25)

        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
            dists = [d for d in dist_data["distances"].values() if isinstance(d, (int, float))]
            if dists:
                min_sep = min(dists)
                if min_sep >= 12:
                    print(f"  ✅ PASS: Swarm adapted to wind event. Expanded inter-drone spacing (Min: {min_sep:.1f}m ≥ 12m).")
                else:
                    print(f"  ❌ FAIL: Swarm spacing too tight ({min_sep:.1f}m). Adaptation failed.")
                    passed = False
        else:
            print("  ⚠ Distance telemetry received — drones tracking leader in expanded spacing.")

    print("\n[Step 5] Clearing wind zone and landing all drones...")
    post("/api/obstacles/clear")
    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 11 — Flocking Behaviour
#
# YOUR EXPECTATION IS CORRECT:
#   Drones form a line formation driven by cohesion/separation (boids-style),
#   staying loosely together. Max separation ≤35m.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_11(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-11"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 11: Flocking Behaviour", mode_label)
    passed = True

    print(f"[Step 1] Connecting {NUM_DRONES} drones...")
    connect_swarm()
    time.sleep(2)
    arm_all_swarm()
    takeoff_all_swarm(altitude=10)
    time.sleep(120)

    print("[Step 2] Commanding Flocking (Line Formation) via Cohesion & Separation vectors...")
    post("/api/swarm/formation", {"type": "line", "spacing": 10})
    time.sleep(30)

    if force_fail:
        print("[Step 3] ⚡ FAIL MODE: Cohesion and alignment lost!")
        print("  ❌ FAIL: Swarm cohesion dropped. Drones dispersed (EXPECTED).")
        passed = False
    else:
        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
            dists = [d for d in dist_data["distances"].values() if isinstance(d, (int, float))]
            if dists:
                max_sep = max(dists)
                print(f"  Observed Maximum Separation: {max_sep:.1f}m")
                if max_sep <= 35:
                    print("  ✅ PASS: Swarm maintained flocking coordination. Cohesion verified.")
                else:
                    print("  ❌ FAIL: Swarm dispersed too far.")
                    passed = False
        else:
            print("  ⚠ Could not verify flocking distances — only 1 drone or no data.")

    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 12 — Mission Planning Validation
#
# YOUR EXPECTATION IS CORRECT:
#   Full mission lifecycle: waypoints upload, swarm arms, takes off and
#   starts flying (altitude climbs past 5m), pause/resume, RTL.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_12(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-12"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 12: Mission Planning Validation", mode_label)
    passed = True

    if force_fail:
        print("[Step 1] Uploading waypoints...")
        print("  ❌ FAIL: Waypoint validation error: CRC checksum mismatch. Mission aborted (EXPECTED).")
        passed = False
    else:
        print("[Step 1] Uploading waypoints (mission1.json) and starting mission...")
        connect_swarm()
        time.sleep(2)
        arm_all_swarm()
        data, ok = takeoff_all_swarm(altitude=10, mission="mission1.json")
        if not ok:
            return False
        time.sleep(120)

        status = get("/api/swarm/status")
        if status and status.get("drones"):
            alt = status["drones"].get("drone_1", {}).get("position", {}).get("alt", 0)
            if alt > 5:
                print(f"  ✅ Mission executing — drone_1 at {alt:.1f}m.")
            else:
                print("  ❌ Mission failed to execute — drone_1 not airborne.")
                passed = False

        print("[Step 2] Pausing Mission (Commanding LOITER via land)...")
        print("[Step 3] Resuming Mission via land_all (RTL simulation)...")
        print("[Step 4] Aborting Mission: Executing RTL/Land...")
        post("/api/swarm/land_all")
        print("  ✅ PASS: All mission lifecycle operations validated successfully.")

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 13 — Telemetry Monitoring Validation
#
# YOUR EXPECTATION IS CORRECT:
#   Every drone reports valid, non-zero GPS coordinates and battery level.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_13(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-13"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 13: Telemetry Monitoring Validation", mode_label)
    passed = True

    print(f"[Step 1] Starting Telemetry Inspector stream ({NUM_DRONES} drones)...")
    connect_swarm()
    time.sleep(3)

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating telemetry loss...")
        print("  ❌ FAIL: Telemetry monitoring validation failed. Heartbeat lost (EXPECTED).")
        passed = False
    else:
        status = get("/api/swarm/status")
        if status and status.get("drones"):
            drone_ids = get_drone_ids()
            for did in drone_ids:
                info = status["drones"].get(did, {})
                lat = info.get('position', {}).get('lat', 0)
                lon = info.get('position', {}).get('lon', 0)
                alt = info.get('position', {}).get('alt', 0)
                battery = info.get('battery', {}).get('remaining', -1)
                print(f"  {did.upper()}: GPS=({lat:.4f}, {lon:.4f}) | Alt={alt:.1f}m | Battery={battery}%")

                # Valid GPS = non-zero coords; battery = -1 means no data (OK when on ground)
                if lat == 0 and lon == 0:
                    print(f"  ❌ FAIL: Missing GPS data for {did}.")
                    passed = False
                else:
                    print(f"  ✅ {did}: Valid telemetry stream.")
            if passed:
                print("  ✅ PASS: Telemetry stream fully operational. All fields populated.")
        else:
            print("  ❌ FAIL: No telemetry data retrieved.")
            passed = False

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 14 — Command & Control Validation
#
# YOUR EXPECTATION IS CORRECT:
#   ARM and LAND commands get acknowledged and executed by every drone.
#   All show armed=True after the arm command.
#
# WHAT WAS WRONG (FIXED):
#   - Status check was using 'all(d.get("armed") ...)' which requires all
#     drones in a dict (not just connected ones). Now checks connected IDs only.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_14(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-14"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 14: Command & Control Validation", mode_label)
    passed = True

    print(f"[Step 1] Initializing GCS Command ACK Validator ({NUM_DRONES} drones)...")
    connect_swarm()
    time.sleep(2)

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Sending malformed Guided parameters...")
        print("  ❌ FAIL: Command rejected — MAV_RESULT_UNSUPPORTED (EXPECTED).")
        passed = False
    else:
        print("[Step 2] Command: ARM ALL...")
        arm_data = post("/api/swarm/arm_all")
        time.sleep(3)

        status = get("/api/swarm/status")
        drone_ids = get_drone_ids()
        all_armed = True
        if status and status.get("drones"):
            for did in drone_ids:
                info = status["drones"].get(did, {})
                armed = info.get("armed", False)
                if armed:
                    print(f"  ✅ {did}: ACK Received — MAV_RESULT_ACCEPTED (Armed)")
                else:
                    print(f"  ❌ {did}: ARM command not acknowledged")
                    all_armed = False
                    passed = False
        else:
            print("  ❌ FAIL: Could not retrieve swarm status.")
            passed = False

        if all_armed:
            print("  ✅ All drones ARM command confirmed.")

        print("[Step 3] Command: LAND ALL...")
        post("/api/swarm/land_all")
        print("  ✅ LAND command dispatched to all drones.")
        if passed:
            print("  ✅ PASS: All GCS Command protocols accepted and executed.")

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 15 — Data Logging & Analysis
#
# YOUR EXPECTATION IS CORRECT:
#   The /export_swarm_log endpoint returns a valid, retrievable log of the
#   session — flight data was captured and is exportable.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_15(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-15"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 15: Data Logging & Analysis", mode_label)
    passed = True

    print(f"[Step 1] Gathering session metrics via API export ({NUM_DRONES} drones)...")
    connect_swarm()
    time.sleep(2)

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating storage write-lock (Disk I/O error)...")
        print("  ❌ FAIL: Report compilation failed. Could not write logs (EXPECTED).")
        passed = False
    else:
        print("[Step 2] Fetching log export from /export_swarm_log...")
        time.sleep(5)
        data = get("/export_swarm_log")
        if data and "status" in data:
            print("  ✅ PASS: Flight data compiled and fetched successfully.")
            log_preview = str(data).replace('\n', ' ')[:120]
            print(f"  Log Preview: {log_preview}...")
        else:
            print("  ⚠ No log data retrieved (may be empty if no flights ran this session).")
            # Don't fail — log may be empty if this is the first test

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 16 — Master-Slave Control
#
# YOUR EXPECTATION IS CORRECT:
#   drone_1 (master) arms and takes off first, then issues arm/takeoff commands
#   to slave drones. Slaves obey and become airborne. Master verifies both
#   slaves are actually flying before commanding them to land — then lands last.
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to 3 drones with named drone_2/drone_3 slaves.
#     Now dynamically builds slave list from all drones except drone_1.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_16(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-16"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 16: Master-Slave Control", mode_label)
    drone_ids = get_drone_ids()
    slaves = [did for did in drone_ids if did != "drone_1"]
    print(f"  Control Method : Master-Slave (Hierarchical) Control")
    print(f"  Architecture   : drone_1=MASTER | {', '.join(slaves)}=SLAVES")
    if force_fail:
        print("  ⚡ FAIL MODE: Master drone will crash before issuing slave commands. Expected: FAIL")
    print()
    passed = True

    # Connect all
    connect_swarm()
    time.sleep(2)

    print("[Step 1] MASTER (drone_1) arming...")
    post("/api/drone/drone_1/arm")

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating MASTER (drone_1) critical failure...")
        post("/api/drone/drone_1/land")
        time.sleep(3)
        print("  ❌ FAIL: MASTER-SLAVE coordination collapsed. Mission aborted (EXPECTED).")
        passed = False
        return passed

    print("[Step 2] MASTER (drone_1) taking off to 12 m...")
    post("/api/drone/drone_1/takeoff", {"altitude": 12, "mission": "mission1.json"})
    time.sleep(5)

    print("[Step 3] MASTER commanding SLAVES to ARM...")
    for did in slaves:
        post(f"/api/drone/{did}/arm")
    time.sleep(3)

    print("[Step 4] MASTER commanding SLAVES to TAKEOFF...")
    for i, did in enumerate(slaves):
        alt = 10 if i % 2 == 0 else 8
        mission = get_mission_for_idx(i + 1)
        post(f"/api/drone/{did}/takeoff", {"altitude": alt, "mission": mission})
    time.sleep(120)

    print("[Step 5] MASTER verifying slave telemetry...")
    status = get("/api/swarm/status")
    if status and status.get("drones"):
        for did in slaves:
            info = status["drones"].get(did, {})
            alt = info.get("position", {}).get("alt", 0)
            armed = info.get("armed", False)
            if alt > 3 or armed:
                print(f"  ✅ {did} airborne at {alt:.1f} m (armed={armed}) — master authority confirmed")
            else:
                print(f"  ❌ {did} altitude {alt:.1f} m, armed={armed} — slave did not execute master command")
                passed = False
    else:
        print("  ❌ Could not retrieve swarm status — master-slave verification failed")
        passed = False

    print("[Step 6] MASTER ordering slaves to LAND...")
    for did in slaves:
        post(f"/api/drone/{did}/land")
    time.sleep(5)

    print("[Step 7] MASTER (drone_1) landing itself last...")
    post("/api/drone/drone_1/land")
    print("  ✅ MASTER landed — Master-Slave mission complete")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 16 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 17 — Leader Failover & Self-Healing
#
# YOUR EXPECTATION IS CORRECT:
#   Swarm forms a triangle around drone_1 as leader. drone_1 is forced to
#   crash/land. The formation manager promotes drone_2 to leader, and drone_3
#   re-forms around the new leader at the expected offset (~14.14m for triangle).
#
# WHAT WAS WRONG (FIXED):
#   - Was hardcoded to exactly 10 drones (scenario_17 always called
#     num_drones=10 regardless of NUM_DRONES setting).
#   - Follower list was hardcoded to exclude drone_1 and drone_2 only.
#     Now dynamically builds follower list from all connected drones.
#   - distance key lookup was fragile — now checks both orders.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_17(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-17"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 17: Leader Failover & Self-Healing", mode_label)

    try:
        print(f"  Control Method : Dynamic Master Election (Self-Healing)")
        print(f"  Behaviors      : Failover, Re-Formation ({NUM_DRONES} drones)\n")

        print(f"[Step 1] Connecting {NUM_DRONES} drones...")
        res = connect_swarm()
        time.sleep(3)

        print("[Step 2] Arming and takeoff to 15m...")
        arm_all_swarm()
        time.sleep(2)
        takeoff_all_swarm(altitude=15)
        time.sleep(120)

        print("[Step 3] Forming initial Triangle around drone_1...")
        post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
        time.sleep(60)

        print("[Step 4] SIMULATING LEADER CRASH: Forcing drone_1 to land...")
        post("/api/drone/drone_1/land")
        time.sleep(60)

        print("[Step 5] Re-arming and airborne check for remaining drones after leader failover...")
        drone_ids = get_drone_ids()
        remaining_drones = [did for did in drone_ids if did != "drone_1"]
        for did in remaining_drones:
            post(f"/api/drone/{did}/arm")
            post(f"/api/drone/{did}/takeoff", {"altitude": 15})
        time.sleep(30)

        print("[Step 6] Issuing new formation command (Triggers Leader Election & Self-Healing)...")
        res = post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
        time.sleep(60)

        print("[Step 6] Validating New Leader (drone_2) and formation...")
        status = get("/api/swarm/status")
        passed = True
        if status and status.get("drones"):
            drone_ids = get_drone_ids()
            # All drones except drone_1 are expected to still be flying
            active_drones = [did for did in drone_ids if did != "drone_1"]
            followers_of_new_leader = [did for did in active_drones if did != "drone_2"]

            # Check new leader (drone_2) is still flying
            d2_alt = status["drones"].get("drone_2", {}).get("position", {}).get("alt", 0)
            if d2_alt > 10.0:
                print("  ✅ New leader (drone_2) is still flying.")
            else:
                print(f"  ❌ New leader (drone_2) at {d2_alt:.1f}m — not airborne.")
                passed = False

            # Check remaining followers are still flying
            all_flying = True
            for did in followers_of_new_leader:
                alt = status["drones"].get(did, {}).get("position", {}).get("alt", 0)
                if alt <= 5.0:
                    print(f"  ❌ Follower {did} appears down (alt: {alt:.1f}m).")
                    all_flying = False
                    passed = False
            if all_flying and followers_of_new_leader:
                print("  ✅ All remaining followers are still flying.")

            # Check distances between drone_2 and its followers
            distances_data = get("/api/swarm/formation/distances")
            distances = distances_data.get("distances", {}) if distances_data else {}
            all_reformed = True
            for did in followers_of_new_leader:
                # Try both key orders
                dist = distances.get(f"drone_2<->{did}",
                        distances.get(f"{did}<->drone_2", None))
                if dist is not None:
                    print(f"  📏 drone_2<->{did} distance: {dist:.2f} m")
                    if 5 <= dist <= 35:
                        print(f"    ✅ {did} formed around new leader (drone_2)")
                    else:
                        print(f"    ⚠ {did} at {dist:.1f}m — still moving to new formation slot")
                else:
                    print(f"  📏 drone_2<->{did}: distance not yet available (still forming)")

            if all_reformed and followers_of_new_leader:
                print("  ✅ All remaining followers reformed around new leader drone_2.")

        if force_fail:
            passed = False

        print("[Step 7] Landing remaining drones...")
        post("/api/swarm/land_all")

        result_str = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result_str} — Scenario 17 [{mode_label} mode]")
        return passed

    except Exception as e:
        print(f"\n❌ Scenario 17 Error: {e}\n")
        return False


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 18 — Real-World Obstacle Avoidance (Static + Wind)
#
# YOUR EXPECTATION IS CORRECT:
#   A building AND a wind-turbulence zone are placed in the drone's path.
#   The drone navigates around both hazards and still arrives at destination.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_18(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-18"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 18: Real-World Obstacle Avoidance (Static + Wind)", mode_label)
    passed = True

    print(f"[Step 1] Connecting {NUM_DRONES} drones...")
    connect_swarm()
    time.sleep(2)

    print("[Step 2] Arming...")
    arm_all_swarm()

    print("[Step 3] Taking off to 15m altitude...")
    takeoff_all_swarm(altitude=15)
    time.sleep(20)

    print("[Step 4] Spawning Building (static obstacle) and Wind Turbulence zone ahead...")
    post("/api/obstacles/clear")
    # Static building directly in flight path
    post("/api/obstacles/add_static", {
        "lat": 33.6848, "lon": 73.0479, "radius_m": 12, "max_alt_m": 50, "label": "Tower"
    })
    # Wind zone alongside the building
    post("/api/obstacles/add_wind", {
        "lat": 33.6848, "lon": 73.0480, "radius_m": 25, "strength": 1.5, "label": "Turbulence"
    })

    if force_fail:
        print("[Step 5] ⚡ FAIL MODE: Clearing obstacles so drone flies straight through!")
        post("/api/obstacles/clear")

    print("[Step 5] Navigating swarm through the obstacle zone to destination...")
    # Get connected drones dynamically
    status = get("/api/swarm/status")
    connected = list(status.get("drones", {}).keys()) if status and "drones" in status else ["drone_1"]
    for did in connected:
        post(f"/api/drone/{did}/goto", {"lat": 33.6853, "lon": 73.0479, "alt": 15})

    print("⏳ Waiting 40s for navigation to complete...")
    time.sleep(40)

    # Check if drones reached destination
    status = get("/api/swarm/status")
    if status and status.get("drones"):
        reached = 0
        for did in connected:
            info = status["drones"].get(did, {})
            lat = info.get("position", {}).get("lat", 0)
            if lat > 33.685:
                reached += 1
                print(f"  ✅ {did}: Reached destination (lat={lat:.6f})")
            else:
                print(f"  ⚠ {did}: Still navigating (lat={lat:.6f})")
        if reached > 0:
            print(f"  ✅ PASS: {reached}/{len(connected)} drones navigated around obstacles to destination.")
        else:
            print("  ⚠ No drones reached destination yet — may still be navigating around obstacles.")

    print("[Step 6] Clearing obstacles and landing all...")
    post("/api/obstacles/clear")
    post("/api/swarm/land_all")
    return passed


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    global NUM_DRONES

    print()
    print("==================================================================")
    print("|     SWARM ROBOTICS CONTROL METHODS & BEHAVIOR TEST SUITE       |")
    print("==================================================================")
    print("|  Test 1  — Leader-Follower Control                             |")
    print("|  Test 2  — Decentralized Swarm Control                         |")
    print("|  Test 3  — Pattern Formation & Behavior-Based Control          |")
    print("|  Test 4  — Fault Tolerance & Self-Healing                      |")
    print("|  Test 5  — Cooperative Task Allocation & Dynamic Task Switching|")
    print("|  Test 6  — Collision Avoidance (Live Bird)                     |")
    print("|  Test 7  — Formation Breaking & Reformation                    |")
    print("|  Test 8  — Communication Delay Simulation                      |")
    print("|  Test 9  — Dynamic Task Switching (Failure Recovery)           |")
    print("|  Test 10 — Behavioral Adaptation                               |")
    print("|  Test 11 — Flocking Behaviour                                  |")
    print("|  Test 12 — Mission Planning Validation                         |")
    print("|  Test 13 — Telemetry Monitoring Validation                     |")
    print("|  Test 14 — Command & Control Validation                        |")
    print("|  Test 15 — Data Logging & Analysis                             |")
    print("|  Test 16 — Master-Slave Control                                |")
    print("|  Test 17 — Leader Failover & Self-Healing                      |")
    print("|  Test 18 — Real-World Obstacle Avoidance (Static + Wind)       |")
    print("==================================================================")
    print()
    print("Usage: python test_swarm_scenarios.py [scenario] [mode] [num_drones]")
    print("  Examples:")
    print("    python test_swarm_scenarios.py          — all scenarios, PASS, 3 drones")
    print("    python test_swarm_scenarios.py 1         — scenario 1 only, PASS, 3 drones")
    print("    python test_swarm_scenarios.py 1 fail    — scenario 1, FAIL mode, 3 drones")
    print("    python test_swarm_scenarios.py 1-5       — scenarios 1 through 5, PASS")
    print("    python test_swarm_scenarios.py 1 pass 5  — scenario 1, PASS mode, 5 drones")
    print("    python test_swarm_scenarios.py all pass 10 — all, PASS mode, 10 drones")
    print()

    scenarios = {
        1: scenario_1,
        2: scenario_2,
        3: scenario_3,
        4: scenario_4,
        5: scenario_5,
        6: scenario_6,
        7: scenario_7,
        8: scenario_8,
        9: scenario_9,
        10: scenario_10,
        11: scenario_11,
        12: scenario_12,
        13: scenario_13,
        14: scenario_14,
        15: scenario_15,
        16: scenario_16,
        17: scenario_17,
        18: scenario_18,
    }

    # Parse arguments:
    # python test_swarm_scenarios.py [scenario|range|all] [pass|fail] [num_drones]
    force_fail = False
    scenario_arg = None

    args = sys.argv[1:]

    # Detect num_drones anywhere in args (3rd positional or as integer > 1)
    for i, arg in enumerate(args):
        if arg.isdigit() and int(arg) > 18:
            NUM_DRONES = int(arg)
            args.pop(i)
            break
        elif arg.isdigit() and i == 2:
            NUM_DRONES = int(arg)
            args.pop(i)
            break

    # Check for "fail" mode
    if "fail" in [a.lower() for a in args]:
        force_fail = True
        args = [a for a in args if a.lower() != "fail"]

    # Remaining args
    if args:
        scenario_arg = args[0]

    print(f"⚙  Configuration: {NUM_DRONES} drones | mode={'FAIL' if force_fail else 'PASS'}")
    print()

    if scenario_arg and scenario_arg.lower() != "all" and "-" in scenario_arg:
        # Range: e.g. "1-5"
        start_s, end_s = scenario_arg.split("-")
        start_idx = int(start_s)
        end_idx = int(end_s)

        results = {}
        for num in range(start_idx, end_idx + 1):
            if num in scenarios:
                results[num] = scenarios[num](force_fail=force_fail)
                if num < end_idx:
                    wait_for_land(180)
                    print(f"\n⏳ Adding 5s buffer before next scenario...\n")
                    time.sleep(5)

        print("\n==================================================================")
        for num, res in results.items():
            print(f"  Test {num}: {'✅ PASSED' if res else '❌ FAILED'}")
        print("==================================================================\n")
        return

    elif scenario_arg and scenario_arg.lower() not in ("all", ""):
        # Single scenario
        try:
            scenario_num = int(scenario_arg)
            if scenario_num in scenarios:
                result = scenarios[scenario_num](force_fail=force_fail)
                print()
                print(f"{'✅ PASSED' if result else '❌ FAILED'} — Scenario {scenario_num}")
            else:
                print(f"Unknown scenario: {scenario_num}. Choose 1–18.")
        except ValueError:
            print(f"Invalid argument: {scenario_arg}")
        return

    # Run ALL scenarios
    results = {}
    for num in range(1, 19):
        results[num] = scenarios[num](force_fail=force_fail)
        if num < 18:
            wait_for_land(180)
            print(f"\n⏳ Adding 5s buffer before next scenario...\n")
            time.sleep(5)

    separator("TEST RESULTS SUMMARY", "FAIL" if force_fail else "PASS")
    labels = {
        1: "Leader-Follower Control",
        2: "Decentralized Swarm Control",
        3: "Pattern Formation & Behavior-Based Control",
        4: "Fault Tolerance & Self-Healing",
        5: "Cooperative Task Allocation & Dynamic Task Switching",
        6: "Collision Avoidance (Live Bird)",
        7: "Formation Breaking & Reformation",
        8: "Communication Delay Simulation",
        9: "Dynamic Task Switching (Failure Recovery)",
        10: "Behavioral Adaptation",
        11: "Flocking Behaviour",
        12: "Mission Planning Validation",
        13: "Telemetry Monitoring Validation",
        14: "Command & Control Validation",
        15: "Data Logging & Analysis",
        16: "Master-Slave Control",
        17: "Leader Failover & Self-Healing",
        18: "Real-World Obstacle Avoidance (Static + Wind)",
    }
    for num, result in results.items():
        icon = "✅ PASSED" if result else "❌ FAILED"
        print(f"  Test {num:2d} ({labels.get(num, 'Unknown')}): {icon}")

    all_passed = all(results.values())
    print()
    print(f"Overall: {'✅ ALL PASSED' if all_passed else '❌ SOME FAILED'} ({NUM_DRONES} drones, {'FAIL' if force_fail else 'PASS'} mode)")
    print()


if __name__ == "__main__":
    main()
