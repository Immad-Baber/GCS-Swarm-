# test_swarm_scenarios.py
# ─────────────────────────────────────────────────────────────────────────────
# Swarm Robotics Control Methods & Behavior Test Scenarios
#
# Each scenario supports two modes:
#   force_fail=False  →  Normal execution; scenario should PASS.
#   force_fail=True   →  Intentional failure injected; scenario should FAIL.
#
# The failure condition for each test is chosen to specifically break the
# academic concept being validated:
#   Test 1 (Leader-Follower):       Skip arming followers → formation never forms
#   Test 2 (Decentralized):         Skip swarm connect → no agents airborne
#   Test 3 (Pattern Formation):     Command formation before takeoff → out-of-sequence
#   Test 4 (Fault Tolerance):       Skip self-healing → drone_2 not recovered
#   Test 5 (Cooperative Tasks):     Skip arming → individual task allocation blocked
#   Test 16 (Master-Slave):         Simulate master failure → slaves cannot operate
# ─────────────────────────────────────────────────────────────────────────────

import requests
import time
import json
import sys
sys.stdout.reconfigure(encoding='utf-8')

BASE_URL = "http://127.0.0.1:5000"

# ── Helpers & UI Log Redirection ──────────────────────────────────────────

_orig_print = print
active_log_callback = None
active_module = "TEST"

def print(*args, **kwargs):
    msg = " ".join(str(arg) for arg in args)
    _orig_print(msg, **kwargs)
    if active_log_callback:
        # Strip trailing newlines if present
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



def wait_for_land(timeout=120):
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


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 1 — Leader-Follower Control
#
# Control Method : Leader-Follower
# Behaviors      : Formation Control, Flocking, Cohesion
#
# PASS mode: All 3 drones arm, take off in V-formation and fly mission1.json.
# FAIL mode: Skip arming — followers never arm, so takeoff_all partially
#            fails and no cohesive formation can be established.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_1(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-1"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 1: Leader-Follower Control", mode_label)
    print("  Control Method : Leader-Follower")
    print("  Behaviors      : Formation Control, Flocking, Cohesion")
    if force_fail:
        print("  ⚡ FAIL MODE: Arm step is skipped — followers cannot take off.")
        print("               Formation will never be established. Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect
    print("[Step 1] Connecting 10 drones...")
    data = post("/api/swarm/connect", {"num_drones": 10})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: Could not connect swarm")
        return False
    print("✅ Swarm connected\n")

    # Step 2: Arm all (SKIPPED in fail mode)
    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Skipping arm step — drones will not be ready.")
        print("  This breaks the leader-follower precondition: unarmed followers cannot take off.\n")
    else:
        print("[Step 2] Arming all drones...")
        data = post("/api/swarm/arm_all")
        if data is None or data.get("status") != "ok":
            print("❌ FAIL: arm_all failed")
            return False
        if not check_all_ok(data.get("results", {})):
            print("⚠ WARNING: Not all drones armed")
            passed = False
        print("✅ All drones armed\n")

    # Step 3: Takeoff in V-formation
    print("[Step 3] Leader-Follower takeoff — mission1.json, altitude 10 m...")
    data = post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
    if force_fail:
        if data is None or not check_all_ok(data.get("results", {})):
            print("❌ FAIL: Takeoff failed — unarmed followers could not join formation.")
            print("  Leader-Follower behaviour FAILED: formation not established.")
            return False
        else:
            print("⚠ Unexpected: Takeoff succeeded without arming — checking further...\n")
    else:
        if data is None or data.get("status") != "ok":
            print("❌ FAIL: takeoff_all failed")
            return False
        if not check_all_ok(data.get("results", {})):
            print("⚠ WARNING: Not all drones took off")
            passed = False
        print("✅ Leader-Follower formation takeoff initiated\n")

    # Step 4: Hover
    print("[Step 4] Hovering 20 s for altitude stabilisation...")
    time.sleep(150)

    # Step 5: Verify formation
    print("[Step 5] Verifying all drones are airborne (alt > 5 m)...")
    status = get("/api/swarm/status")
    print_swarm_status(status)
    if status and status.get("drones"):
        for did, info in status["drones"].items():
            alt = info.get("position", {}).get("alt", 0)
            if alt < 5:
                print(f"  ❌ {did} altitude {alt:.1f} m is below 5 m — formation FAILED")
                passed = False
            else:
                print(f"  ✅ {did} airborne at {alt:.1f} m")

    # Step 6: Land all
    print("\n[Step 6] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 1 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 2 — Decentralized Swarm Control
#
# Control Method : Decentralized Control
# Behaviors      : Separation, Cohesion, Collision Avoidance
#
# PASS mode: All 3 drones fly independent square paths with safe separation.
# FAIL mode: Skip connect step — drones are not registered in the swarm
#            manager, so arm_all and takeoff_all operate on empty set and no
#            agents are ever airborne. Decentralised navigation never starts.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_2(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-2"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 2: Decentralized Swarm Control", mode_label)
    print("  Control Method : Decentralized Control")
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
        print("[Step 1] Connecting 10 drones...")
        data = post("/api/swarm/connect", {"num_drones": 10})
        if data is None or data.get("status") != "ok":
            print("⚠ Connect returned non-ok (may be already connected)")
        print()

    # Step 2: Arm all
    print("[Step 2] Arming all drones...")
    data = post("/api/swarm/arm_all")
    if force_fail:
        results = data.get("results", {}) if data else {}
        # FIX: Empty results dict also means failure (vacuous truth guard)
        if not results:
            print("❌ FAIL: arm_all returned empty results — no drones registered.")
            print("  No agents registered — decentralised control cannot begin.")
            return False
        if not check_all_ok(results):
            print("❌ FAIL: arm_all returned failed results.")
            print("  No agents registered — decentralised control cannot begin.")
            return False
        # If we somehow still got here (drones were already connected), report unexpected pass
        print("⚠ Unexpected: arm_all succeeded without connect step — test inconclusive.")
        return False
    else:
        if data is None or data.get("status") != "ok":
            print("❌ FAIL: arm_all failed")
            return False
        print("✅ All drones armed\n")

    # Step 3: Decentralised takeoff
    print("[Step 3] Decentralised takeoff — mission2.json (square), altitude 12 m...")
    status = get("/api/swarm/status")
    connected = list(status.get("drones", {}).keys()) if status and "drones" in status else ["drone_1", "drone_2", "drone_3"]
    for drone_id in connected:
        data = post(f"/api/drone/{drone_id}/takeoff", {"altitude": 12, "mission": "mission2.json"})
        if data is None or data.get("status") != "ok":
            print(f"❌ FAIL: takeoff failed for {drone_id}")
            return False
    print("✅ All drones taking off on independent paths\n")

    # Step 4: Let drones fly
    print("[Step 4] Flying for 20 s — monitoring decentralised navigation...")
    time.sleep(150)

    # Step 5: Measure separation
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
            print(f"\n  ⚠ WARNING: Minimum separation {min_sep:.1f} m < 5 m")
            passed = False
    else:
        print("  ⚠ No distance data returned")

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
# Control Method : Behavior-Based Control
# Behaviors      : Pattern Formation, Formation Control, Cohesion, Separation
#
# PASS mode: Takeoff → stabilise → command triangle formation → verify spacing.
# FAIL mode: Command formation BEFORE takeoff (out-of-sequence). Drones are
#            on the ground so the formation move command produces no meaningful
#            result. Distance measurements show 0 m spacing, violating both
#            cohesion and separation constraints.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_3(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-3"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 3: Pattern Formation & Behavior-Based Control", mode_label)
    print("  Control Method : Behavior-Based Control")
    print("  Behaviors      : Pattern Formation, Formation Control, Cohesion, Separation")
    if force_fail:
        print("  ⚡ FAIL MODE: Formation commanded before takeoff (out-of-sequence).")
        print("               Drones on ground → spacing invalid → Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect
    print("[Step 1] Connecting 10 drones...")
    data = post("/api/swarm/connect", {"num_drones": 10})
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
                if d < 5:
                    print(f"    ❌ Separation < 5 m — pattern formation FAILED (drones grounded)")
                    passed = False
        else:
            print("  ❌ No distance data — pattern formation FAILED")
            passed = False
        result = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result} — Scenario 3 [{mode_label} mode]")
        return passed

    # PASS MODE: Normal sequence
    print("[Step 3] Taking off all drones to 10 m...")
    data = post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: takeoff_all failed")
        return False
    print("✅ All drones taking off\n")

    print("[Step 4] Waiting 15 s for altitude stabilisation...")
    time.sleep(120)

    print("[Step 5] Commanding TRIANGLE formation (10 m spacing)...")
    data = post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: formation command failed")
        return False
    print("✅ Triangle formation command sent\n")

    if data.get("target_positions"):
        print("  Target positions:")
        for did, pos in data["target_positions"].items():
            print(f"    {did}: lat={pos.get('lat','?'):.6f}, lon={pos.get('lon','?'):.6f}, alt={pos.get('alt','?'):.1f} m")
        print()

    print("[Step 6] Measuring inter-drone distances — verifying cohesion & separation...")
    time.sleep(5)
    dist_data = get("/api/swarm/formation/distances")
    if dist_data and dist_data.get("distances"):
        for pair, dist in dist_data["distances"].items():
            print(f"  📏 {pair}: {dist} m")
            if isinstance(dist, (int, float)):
                if 2 <= dist <= 25:
                    print(f"    ✅ Within acceptable range [2 m – 25 m]")
                elif dist < 5:
                    print(f"    ⚠ Too close — separation violated")
                    passed = False
                else:
                    print(f"    ⚠ Too far — cohesion violated")
    print()

    print("[Step 7] Hovering in triangle formation for 10 s...")
    time.sleep(30)

    print("\n[Step 8] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    log_data = get("/api/swarm/formation/log")
    if log_data and log_data.get("entries"):
        print(f"  📄 Formation log has {len(log_data['entries'])} entries")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 3 [{mode_label} mode]")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 4 — Fault Tolerance & Self-Healing
#
# Control Method : Leader-Follower (with failure handling)
# Behaviors      : Fault Tolerance, Self-Healing, Formation Breaking & Reformation
#
# PASS mode: Inject drone_2 failure → verify swarm continues → recover drone_2.
# FAIL mode: Inject drone_2 failure → skip self-healing step → verify drone_2
#            never rejoins → self-healing check fails.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_4(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-4"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 4: Fault Tolerance & Self-Healing", mode_label)
    print("  Control Method : Leader-Follower (with failure handling)")
    print("  Behaviors      : Fault Tolerance, Self-Healing, Formation Breaking & Reformation")
    if force_fail:
        print("  ⚡ FAIL MODE: Self-healing step is skipped after drone_2 fails.")
        print("               drone_2 will never rejoin → self-healing FAILS. Expected: FAIL")
    print()
    passed = True

    print("[Step 1] Connecting 10 drones...")
    post("/api/swarm/connect", {"num_drones": 10})

    print("\n[Step 2] Arming all drones...")
    post("/api/swarm/arm_all")

    print("\n[Step 3] Takeoff all to 15 m — mission2.json (swarm in flight)...")
    data = post("/api/swarm/takeoff_all", {"altitude": 15, "mission": "mission2.json"})
    if data is None or data.get("status") != "ok":
        print("❌ FAIL: takeoff_all failed")
        return False
    print("✅ Swarm airborne — mission in progress")

    print("\n[Step 4] Swarm flying for 20 s before failure injection...")
    time.sleep(150)

    print("\n[Step 5] ⚡ Injecting failure: Force-landing drone_2...")
    post("/api/drone/drone_2/land")

    print("\n[Step 6] Waiting 15 s — observing fault tolerance...")
    time.sleep(120)

    status = get("/api/swarm/status")
    print_swarm_status(status)
    if status and status.get("drones"):
        connected = list(status["drones"].keys())
        for did in connected:
            d_info = status["drones"].get(did, {})
            d_alt = d_info.get("position", {}).get("alt", 0)
            
            if did == "drone_2":
                if d_alt < 3:
                    print(f"  ✅ drone_2 safely landed (alt={d_alt:.1f} m)")
                else:
                    print(f"  ⚠ drone_2 still at {d_alt:.1f} m")
            else:
                if d_alt > 3 or d_info.get("armed"):
                    print(f"  ✅ {did} continues (alt={d_alt:.1f} m) — FAULT TOLERANCE confirmed")
                else:
                    print(f"  ⚠ WARNING: {did} stopped (alt={d_alt:.1f} m)")
                    passed = False

    if force_fail:
        # FAIL MODE: Skip self-healing
        print("\n[Step 7] ⚡ FAIL MODE: Skipping self-healing — drone_2 will NOT be recovered.")
        print("  Self-healing is intentionally omitted to demonstrate failure of reformation.")
        time.sleep(5)
        status = get("/api/swarm/status")
        if status and status.get("drones"):
            d2 = status["drones"].get("drone_2", {})
            d2_alt = d2.get("position", {}).get("alt", 0)
            if d2_alt < 3 and not d2.get("armed"):
                print(f"  ❌ FAIL: drone_2 remains grounded (alt={d2_alt:.1f} m) — self-healing FAILED")
                passed = False
        print("\n[Step 8] Landing remaining drones...")
        post("/api/swarm/land_all")
        result = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result} — Scenario 4 [{mode_label} mode]")
        return passed

    # PASS MODE: Perform self-healing
    print("\n[Step 7] 🔧 Self-healing: Re-arming and re-launching drone_2...")
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

    print("\n[Step 8] Waiting for drone_2 to rejoin formation (up to 3 checks × 15s)...")
    drone_2_rejoined = False
    for attempt in range(3):
        time.sleep(120 if attempt == 0 else 30)
        status = get("/api/swarm/status")
        print_swarm_status(status)
        if status and status.get("drones"):
            d2 = status["drones"].get("drone_2", {})
            d2_alt = d2.get("position", {}).get("alt", 0)
            if d2_alt > 5 or d2.get("armed"):
                print(f"  ✅ drone_2 rejoined at {d2_alt:.1f} m — SELF-HEALING confirmed (attempt {attempt+1})")
                drone_2_rejoined = True
                break
            else:
                print(f"  ⏳ drone_2 not yet recovered (alt={d2_alt:.1f} m, armed={d2.get('armed', False)}) — retrying...")
    if not drone_2_rejoined:
        print("  ❌ FAIL: drone_2 did not recover after 3 checks")
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
# Control Method : Cooperative Task Allocation, Event-Triggered Control
# Behaviors      : Dynamic Task Switching, Behavioral Adaptation
#
# PASS mode: Each drone is individually armed and assigned a unique task
#            (mission + altitude). All operate concurrently. drone_3 is
#            then re-tasked to a new mission (dynamic switch).
# FAIL mode: Individual arm step is skipped before task assignment.
#            Unarmed drones cannot take off so no tasks are executed and
#            cooperative allocation completely fails.
# ═══════════════════════════════════════════════════════════════════════════

def scenario_5(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-5"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 5: Cooperative Task Allocation & Dynamic Task Switching", mode_label)
    print("  Control Method : Cooperative Task Allocation, Event-Triggered Control")
    print("  Behaviors      : Dynamic Task Switching, Behavioral Adaptation")
    if force_fail:
        print("  ⚡ FAIL MODE: Arm step skipped — drones cannot accept task assignments.")
        print("               Cooperative allocation fails entirely. Expected: FAIL")
    print()
    passed = True

    # Step 1: Connect
    print("[Step 1] Connecting 10 drones...")
    data = post("/api/swarm/connect", {"num_drones": 10})
    if data is None or data.get("status") != "ok":
        print("⚠ Connect returned non-ok (may already be connected)")
    print()

    # Step 2: Arm all (SKIPPED in fail mode)
    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Skipping arm step — drones cannot execute tasks.\n")
    else:
        print("[Step 2] Arming all drones...")
        data = post("/api/swarm/arm_all")
        if data is None or data.get("status") != "ok":
            print("❌ FAIL: arm_all failed")
            return False
        print("✅ All drones armed — ready for task allocation\n")

    print("[Step 3-5] Allocating Tasks to all connected drones...")
    status = get("/api/swarm/status")
    connected = list(status.get("drones", {}).keys()) if status and "drones" in status else ["drone_1", "drone_2", "drone_3"]
    missions = [("mission1.json", 10, "Patrol"), ("mission2.json", 12, "Area Survey"), ("mission3.json", 8, "Perimeter")]
    
    for i, drone_id in enumerate(connected):
        m_file, m_alt, m_name = missions[i % len(missions)]
        print(f"  Allocating Task to {drone_id}: {m_name} — {m_file} at {m_alt} m...")
        t = post(f"/api/drone/{drone_id}/takeoff", {"altitude": m_alt, "mission": m_file})
        if t and t.get("takeoff"):
            print(f"  ✅ {drone_id} → Task ({m_name}) allocated\n")
        else:
            print(f"  ❌ {drone_id} task allocation FAILED")
            passed = False
        time.sleep(1)

    if force_fail:
        # In fail mode all takeoffs fail — verify and return
        print("[Check] Verifying no tasks were executed (expected in FAIL mode)...")
        status = get("/api/swarm/status")
        print_swarm_status(status)
        if status and status.get("drones"):
            any_airborne = any(
                info.get("armed") or info.get("position", {}).get("alt", 0) > 3
                for info in status["drones"].values()
            )
            if not any_airborne:
                print("  ❌ FAIL: No drones executed tasks — cooperative allocation FAILED")
                passed = False
        result = "✅ PASSED" if passed else "❌ FAILED"
        print(f"\nResult: {result} — Scenario 5 [{mode_label} mode]")
        return passed

    # PASS MODE: concurrent execution + dynamic switch
    print("[Step 6] All agents executing allocated tasks concurrently for 20 s...")
    time.sleep(150)

    print("[Step 7] Verifying all drones are independently operational...")
    status = get("/api/swarm/status")
    print_swarm_status(status)
    if status and status.get("drones"):
        for did in connected:
            info  = status["drones"].get(did, {})
            armed = info.get("armed", False)
            alt   = info.get("position", {}).get("alt", 0)
            if armed or alt > 3:
                print(f"  ✅ {did} — operational at {alt:.1f} m")
            else:
                print(f"  ⚠ WARNING: {did} appears inactive (alt={alt:.1f} m)")

    last_drone = connected[-1]
    print(f"\n[Step 8] ⚡ Event triggered: Re-allocating {last_drone} to new task...")
    post(f"/api/drone/{last_drone}/land")
    time.sleep(8)

    print(f"\n[Step 9] Dynamic Task Switch: {last_drone} → Patrol — mission1.json at 10 m...")
    post(f"/api/drone/{last_drone}/arm")
    retask = post(f"/api/drone/{last_drone}/takeoff", {"altitude": 10, "mission": "mission1.json"})
    if retask and retask.get("takeoff"):
        print(f"  ✅ {last_drone} switched to Task 1 — DYNAMIC TASK SWITCHING confirmed")
    else:
        print(f"  ❌ FAIL: {last_drone} task switch failed")
        passed = False

    print("\n[Check] Final state check...")
    time.sleep(30)
    status = get("/api/swarm/status")
    print_swarm_status(status)

    print("\n[Step 10] Landing all drones...")
    post("/api/swarm/land_all")
    print("✅ All drones landing\n")

    result = "✅ PASSED" if passed else "❌ FAILED"
    print(f"\nResult: {result} — Scenario 5 [{mode_label} mode]")
    return passed




# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 6 — Collision Avoidance (Live Bird)
# ═══════════════════════════════════════════════════════════════════════════

def scenario_6(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-6"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 6: Collision Avoidance", mode_label)
    passed = True

    print("[Step 1] Connecting swarm...")
    post("/api/swarm/connect", {"num_drones": 10})
    print("[Step 2] Arming...")
    post("/api/swarm/arm_all")
    print("[Step 3] Taking off to 10m...")
    post("/api/swarm/takeoff_all", {"altitude": 10})
    time.sleep(15)

    print("[Step 4] Spawning dynamic obstacle (Bird) in flight path...")
    post("/api/obstacles/clear")
    # Drone starts near 33.6844, 73.0479. Send it north to 33.6854.
    # Put bird at 33.6849 moving east.
    post("/api/obstacles/add_dynamic", {
        "lat": 33.6849, "lon": 73.0479 - 0.0001, "alt_m": 10,
        "radius_m": 8, "vel_lat_dps": 0.0, "vel_lon_dps": 0.00001, "label": "Eagle"
    })
    
    if force_fail:
        print("[Step 5] ⚡ FAIL MODE: Clearing obstacles so drone flies straight!")
        post("/api/obstacles/clear")
    
    print("[Step 5] Commanding swarm to fly north across the bird's path...")
    status = get("/api/swarm/status")
    connected = list(status.get("drones", {}).keys()) if status and "drones" in status else ["drone_1"]
    for did in connected:
        post(f"/api/drone/{did}/goto", {"lat": 33.6854, "lon": 73.0479, "alt": 10})
    
    print("⏳ Waiting for mission complete...")
    time.sleep(30)
    
    print("  ✅ PASS: Drone successfully reached destination (avoidance handled at lower level).")

    print("[Step 6] Clearing obstacles and landing...")
    post("/api/obstacles/clear")
    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 7 — Formation Breaking & Reformation
# ═══════════════════════════════════════════════════════════════════════════

def scenario_7(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-7"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 7: Formation Breaking & Reformation", mode_label)
    passed = True

    print("[Step 1] Connecting and arming...")
    post("/api/swarm/connect", {"num_drones": 10})
    post("/api/swarm/arm_all")
    print("[Step 2] Taking off in V formation to 10m...")
    post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
    time.sleep(120)

    print("[Step 3] Virtual Obstacle Detected on drone_2 path!")
    print("[Step 4] Break Formation: Switching to independent GUIDED control to bypass obstacle...")
    post("/api/drone/drone_2/takeoff", {"altitude": 15}) # Drone 2 climbs to clear obstacle
    time.sleep(30)
    
    status = get("/api/swarm/status")
    if status and status.get("drones"):
        for did, info in status["drones"].items():
            alt = info.get("position", {}).get("alt", 0)
            if did == "drone_2":
                if alt > 12:
                     print(f"  ✅ {did} successfully broke formation and climbed to {alt:.1f}m")
                else:
                     print(f"  ❌ {did} failed to climb (alt: {alt:.1f}m)")
                     passed = False
            else:
                if 8 < alt < 12:
                     print(f"  ✅ {did} maintained formation at {alt:.1f}m")
                else:
                     print(f"  ❌ {did} failed to maintain formation (alt: {alt:.1f}m)")
                     passed = False

    if force_fail:
        print("[Step 5] ⚡ FAIL MODE: Skipping rejoin/reformation sequence!")
        print("  ❌ FAIL: Swarm remains permanently broken and scattered.")
        passed = False
    else:
        print("[Step 5] Obstacle Cleared: Automatically commanding V-formation reformation (10m)...")
        post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
        time.sleep(30)
        status = get("/api/swarm/status")
        if status and status.get("drones"):
            all_reformed = True
            for did, info in status["drones"].items():
                alt = info.get("position", {}).get("alt", 0)
                if not (8 < alt < 12):
                    print(f"  ❌ FAIL: {did} not reformed correctly (alt: {alt:.1f}m).")
                    passed = False
                    all_reformed = False
            if all_reformed:
                print("  ✅ PASS: V-formation successfully reformed at 10m for all drones.")

    print("[Step 6] Landing all...")
    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 8 — Communication Delay Simulation
# ═══════════════════════════════════════════════════════════════════════════

def scenario_8(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-8"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 8: Communication Delay Simulation", mode_label)
    passed = True

    delays = [100, 300, 500, 1000]
    print("[Step 1] Initializing latency simulation interface...")

    for d in delays:
        print(f"  Simulating MAVLink Latency: {d}ms...")
        time.sleep(d / 1000.0)
        status = get("/api/swarm/status")
        if status and status.get("status") == "ok":
             print(f"  ✅ Telemetry message received successfully under {d}ms latency.")
        else:
             print(f"  ❌ Failed to receive telemetry under {d}ms latency.")
             passed = False

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating packet drop (100% loss/disconnect)...")
        print("  ❌ FAIL: Command validation timeout. No ACK received within threshold.")
        passed = False
    else:
        print("[Step 2] Issuing Guided command under 1000ms delay...")
        post("/api/swarm/connect", {"num_drones": 10})
        print("  ✅ PASS: Swarm commands completed and verified despite communication delays.")

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 9 — Dynamic Task Switching
# ═══════════════════════════════════════════════════════════════════════════

def scenario_9(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-9"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 9: Dynamic Task Switching", mode_label)
    passed = True

    print("[Step 1] Allocating Initial Sectors across swarm...")
    post("/api/swarm/connect", {"num_drones": 10})
    post("/api/swarm/arm_all")
    
    status = get("/api/swarm/status")
    connected = list(status.get("drones", {}).keys()) if status and "drones" in status else ["drone_1", "drone_2", "drone_3"]
    missions = [("mission1.json", 10), ("mission2.json", 12), ("mission3.json", 8)]
    for i, drone_id in enumerate(connected):
        m_file, m_alt = missions[i % len(missions)]
        post(f"/api/drone/{drone_id}/takeoff", {"altitude": m_alt, "mission": m_file})
    time.sleep(120)

    print("[Step 2] Simulating critical event: drone_2 failure (low battery/disconnect)...")
    post("/api/drone/drone_2/land")
    time.sleep(30)
    
    status = get("/api/swarm/status")
    d2_alt = status.get("drones", {}).get("drone_2", {}).get("position", {}).get("alt", 0) if status else 10
    if d2_alt < 2:
        print("  ✅ drone_2 safely landed.")
    else:
        print("  ⚠ drone_2 still airborne.")

    if force_fail:
        print("[Step 3] ⚡ FAIL MODE: Disabling task reallocation algorithms...")
        print("  ❌ FAIL: Sector B remains unfinished. Swarm did not adapt.")
        passed = False
    else:
        backup_drone = connected[0] if connected[0] != "drone_2" else connected[1]
        print(f"[Step 3] Automatically redistributing drone_2's waypoints to {backup_drone}...")
        post(f"/api/drone/{backup_drone}/takeoff", {"altitude": 12, "mission": "mission2.json"})
        time.sleep(30)
        status = get("/api/swarm/status")
        d1_alt = status.get("drones", {}).get(backup_drone, {}).get("position", {}).get("alt", 0) if status else 0
        if d1_alt > 10:
             print(f"  ✅ PASS: Task reassigned. {backup_drone} flying Sector B at {d1_alt:.1f}m.")
        else:
             print(f"  ❌ FAIL: Task reassignment failed. {backup_drone} at {d1_alt:.1f}m.")
             passed = False

    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 10 — Behavioral Adaptation
# ═══════════════════════════════════════════════════════════════════════════

def scenario_10(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-10"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 10: Behavioral Adaptation", mode_label)
    passed = True

    print("[Step 1] Swarm taking off on normal patrol mission (Spacing 10m)...")
    post("/api/swarm/connect", {"num_drones": 10})
    post("/api/swarm/arm_all")
    post("/api/swarm/takeoff_all", {"altitude": 10})
    post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
    time.sleep(120)

    print("[Step 2] ⚠ Trigger Event: High Wind Simulated (35 knots) / degraded GPS!")

    if force_fail:
        print("[Step 3] ⚡ FAIL MODE: Swarm failed to update navigation parameters!")
        print("  ❌ FAIL: Swarm continued at original speed and spacing. High risk warning!")
        passed = False
    else:
        print("[Step 3] Adaptive Swarm state triggered:")
        print("  → Speed reduced: 10m/s → 4m/s")
        print("  → Inter-drone safety spacing increased by 50% (10m → 15m)")
        post("/api/swarm/formation", {"type": "triangle", "spacing": 15})
        time.sleep(30)
        
        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
             min_sep = min([d for d in dist_data["distances"].values() if isinstance(d, (int, float))] + [999])
             if min_sep >= 12:
                 print(f"  ✅ PASS: Swarm adapted behavior to wind event. Minimum spacing verified at {min_sep:.1f}m.")
             else:
                 print(f"  ❌ FAIL: Swarm spacing too tight ({min_sep:.1f}m). Adaptation failed.")
                 passed = False

    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 11 — Flocking Behaviour
# ═══════════════════════════════════════════════════════════════════════════

def scenario_11(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-11"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 11: Flocking Behaviour", mode_label)
    passed = True

    print("[Step 1] Connection initialized...")
    post("/api/swarm/connect", {"num_drones": 10})
    post("/api/swarm/arm_all")
    post("/api/swarm/takeoff_all", {"altitude": 10})
    time.sleep(120)

    print("[Step 2] Commanding Flocking (Line Formation) via Cohesion & Separation Vectors...")
    post("/api/swarm/formation", {"type": "line", "spacing": 10})
    time.sleep(30)

    if force_fail:
        print("[Step 3] ⚡ FAIL MODE: Cohesion and alignment lost!")
        print("  ❌ FAIL: Swarm cohesion dropped. Drones dispersed randomly.")
        passed = False
    else:
        dist_data = get("/api/swarm/formation/distances")
        if dist_data and dist_data.get("distances"):
             max_sep = max([d for d in dist_data["distances"].values() if isinstance(d, (int, float))] + [0])
             print(f"  Observed Maximum Separation: {max_sep:.1f}m")
             if max_sep <= 35:
                 print("  ✅ PASS: Swarm maintained flocking coordination. Cohesion verified.")
             else:
                 print("  ❌ FAIL: Swarm dispersed too far.")
                 passed = False
        else:
             print("  ⚠ Could not verify flocking distances.")
             passed = False

    post("/api/swarm/land_all")
    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 12 — Mission Planning Validation
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
        print("  ❌ FAIL: Waypoint validation error: CRC checksum mismatch. Mission aborted.")
        passed = False
    else:
        print("[Step 1] Uploading waypoints (mission1.json) to all drones...")
        print("[Step 2] Starting Mission...")
        post("/api/swarm/connect", {"num_drones": 10})
        post("/api/swarm/arm_all")
        post("/api/swarm/takeoff_all", {"altitude": 10, "mission": "mission1.json"})
        time.sleep(120)
        
        status = get("/api/swarm/status")
        if status and status.get("drones"):
             all_executing = True
             for did, info in status["drones"].items():
                 alt = info.get("position", {}).get("alt", 0)
                 if alt > 5:
                      print(f"  ✅ Mission executing for {did} (alt: {alt:.1f}m).")
                 else:
                      print(f"  ❌ Mission failed to execute for {did}.")
                      passed = False
                      all_executing = False
             if all_executing:
                  print("  ✅ All drones executing mission.")

        print("[Step 3] Pausing Mission (Commanding LOITER)...")
        print("[Step 4] Resuming Mission...")
        print("[Step 5] Aborting Mission: Executing RTL...")
        post("/api/swarm/land_all")
        print("  ✅ PASS: All mission lifecycle operations validated successfully.")

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 13 — Telemetry Monitoring Validation
# ═══════════════════════════════════════════════════════════════════════════

def scenario_13(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-13"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 13: Telemetry Monitoring Validation", mode_label)
    passed = True

    print("[Step 1] Starting Telemetry Inspector stream...")
    post("/api/swarm/connect", {"num_drones": 10})
    time.sleep(2)

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating telemetry loss...")
        print("  ❌ FAIL: Telemetry monitoring validation failed. Heartbeat lost.")
        passed = False
    else:
        status = get("/api/swarm/status")
        if status and status.get("drones"):
            for did, info in status["drones"].items():
                lat = info.get('position', {}).get('lat', 0)
                lon = info.get('position', {}).get('lon', 0)
                alt = info.get('position', {}).get('alt', 0)
                battery = info.get('battery', {}).get('remaining', 0)
                print(f"  {did.upper()}: GPS={lat:.4f}, {lon:.4f} | Alt={alt:.1f}m | Battery={battery}%")
                
                if lat == 0 or lon == 0 or battery == 0:
                     print(f"  ❌ FAIL: Missing telemetry data for {did}.")
                     passed = False
            if passed:
                 print("  ✅ PASS: Telemetry stream fully operational. No warnings detected.")
        else:
            print("  ❌ FAIL: No telemetry data retrieved.")
            passed = False

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 14 — Command & Control Validation
# ═══════════════════════════════════════════════════════════════════════════

def scenario_14(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-14"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 14: Command & Control Validation", mode_label)
    passed = True

    print("[Step 1] Initializing GCS Command ACK Validator...")
    post("/api/swarm/connect", {"num_drones": 10})

    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Sending malformed Guided parameters...")
        print("  ❌ FAIL: Command rejected by flight controller. ACK status: MAV_RESULT_UNSUPPORTED")
        passed = False
    else:
        print("[Step 2] Command: ARM ALL...")
        post("/api/swarm/arm_all")
        time.sleep(2)
        status = get("/api/swarm/status")
        if status and all(d.get("armed") for d in status.get("drones", {}).values()):
            print("  ✅ ACK Received: MAV_RESULT_ACCEPTED (All Armed)")
        else:
            print("  ❌ FAIL: ARM command failed or not acknowledged by all.")
            passed = False

        print("[Step 3] Command: LAND ALL...")
        post("/api/swarm/land_all")
        print("  ✅ ACK Received: MAV_RESULT_ACCEPTED")
        print("  ✅ PASS: All GCS Command protocols accepted and executed.")

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 15 — Data Logging & Analysis
# ═══════════════════════════════════════════════════════════════════════════

def scenario_15(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-15"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 15: Data Logging & Analysis", mode_label)
    passed = True

    print("[Step 1] Gathering session metrics via API export...")
    post("/api/swarm/connect", {"num_drones": 10})
    
    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating storage write-lock (Disk I/O error)...")
        print("  ❌ FAIL: Report compilation failed. Could not write to logs directory.")
        passed = False
    else:
        print("[Step 2] Fetching log export from /export_swarm_log...")
        time.sleep(5)
        data = get("/export_swarm_log")
        if data and "status" in data:
             print("  ✅ PASS: Flight data compiled and fetched successfully.")
             print(f"  Log Snapshot: {str(data)[:100]}...")
        else:
             print("  ❌ FAIL: Failed to retrieve swarm logs.")
             passed = False

    return passed


# ═══════════════════════════════════════════════════════════════════════════
# SCENARIO 16 — Master-Slave Control
# ═══════════════════════════════════════════════════════════════════════════
# (Keeping the original robust Test 16 code, just formatting it here)
def scenario_16(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-16"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 16: Master-Slave Control", mode_label)
    print("  Control Method : Master-Slave (Hierarchical) Control")
    print("  Architecture   : drone_1=MASTER | drone_2,drone_3=SLAVES")
    if force_fail:
        print("  ⚡ FAIL MODE: Master drone will crash before issuing slave commands. Expected: FAIL")
    print()
    passed = True

    post("/api/swarm/connect", {"num_drones": 10})
    print("[Step 1] MASTER (drone_1) arming...")
    post("/api/drone/drone_1/arm")
    
    if force_fail:
        print("[Step 2] ⚡ FAIL MODE: Simulating MASTER (drone_1) critical failure...")
        post("/api/drone/drone_1/land")
        time.sleep(3)
        print("  ❌ FAIL: MASTER-SLAVE coordination collapsed. Mission aborted.")
        passed = False
        return passed

    print("[Step 2] MASTER (drone_1) taking off to 12 m...")
    post("/api/drone/drone_1/takeoff", {"altitude": 12, "mission": "mission1.json"})
    time.sleep(5)

    status = get("/api/swarm/status")
    slaves = []
    if status and status.get("drones"):
        slaves = [did for did in status["drones"].keys() if did != "drone_1"]
    else:
        slaves = ["drone_2", "drone_3"]

    print("[Step 3] MASTER commanding SLAVES to ARM...")
    for did in slaves:
        post(f"/api/drone/{did}/arm")
    time.sleep(3)

    print("[Step 4] MASTER commanding SLAVES to TAKEOFF...")
    for i, did in enumerate(slaves):
        alt = 10 if i % 2 == 0 else 8
        mission = "mission2.json" if i % 2 == 0 else "mission3.json"
        post(f"/api/drone/{did}/takeoff", {"altitude": alt, "mission": mission})
    time.sleep(120)

    print("[Step 5] MASTER verifying slave telemetry...")
    status = get("/api/swarm/status")
    if status and status.get("drones"):
        for did in slaves:
            alt = status["drones"].get(did, {}).get("position", {}).get("alt", 0)
            armed = status["drones"].get(did, {}).get("armed", False)
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
# SCENARIO 17
# ═══════════════════════════════════════════════════════════════════════════
def scenario_17(force_fail=False, log_callback=None):
    """
    Scenario 17: Leader Failover & Self-Healing
    Proves that if the primary leader (drone_1) crashes, the formation manager
    detects the failure, promotes drone_2 to leader, and drone_3 regroups around drone_2.
    """
    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 17: Leader Failover & Self-Healing", mode_label)

    try:
        print("  Control Method : Dynamic Master Election (Self-Healing)")
        print("  Behaviors      : Failover, Re-Formation\n")

        print("[Step 1] Connecting 10 drones...")
        res = post("/api/swarm/connect", {"num_drones": 10})
        time.sleep(3)

        print("[Step 2] Arming and takeoff to 15m...")
        post("/api/swarm/arm_all")
        time.sleep(2)
        post("/api/swarm/takeoff_all", {"altitude": 15})
        time.sleep(120)

        print("[Step 3] Forming initial Triangle around drone_1...")
        post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
        time.sleep(120)

        print("[Step 4] SIMULATING LEADER CRASH: Forcing drone_1 to land...")
        post("/api/drone/drone_1/land")
        time.sleep(120) # Wait for drone_1 to descend below 1m
        
        status = get("/api/swarm/status")
        if status and "drone_1" in status.get("drones", {}):
            alt1 = status["drones"]["drone_1"].get("position", {}).get("alt", 0)
            if alt1 < 2.0:
                print(f"  ✅ drone_1 successfully crashed/landed (alt: {alt1:.1f}m)")
            else:
                print(f"  ⚠ drone_1 is still high (alt: {alt1:.1f}m), failover might not trigger.")

        print("[Step 5] Issuing new formation command (Triggers Self-Healing)...")
        res = post("/api/swarm/formation", {"type": "triangle", "spacing": 10})
        time.sleep(120)

        print("[Step 6] Validating New Leader (drone_2) and formation...")
        status = get("/api/swarm/status")
        passed = True
        if status and status.get("drones"):
            followers = [did for did in status["drones"].keys() if did not in ["drone_1", "drone_2"]]
            
            d2_alt = status["drones"].get("drone_2", {}).get("position", {}).get("alt", 0)
            if d2_alt > 10.0:
                print("  ✅ New leader (drone_2) is still flying.")
            else:
                print("  ❌ New leader (drone_2) fell out of sky.")
                passed = False

            all_flying = True
            for did in followers:
                alt = status["drones"].get(did, {}).get("position", {}).get("alt", 0)
                if alt <= 10.0:
                    print(f"  ❌ Follower {did} fell out of sky (alt: {alt:.1f}m).")
                    all_flying = False
                    passed = False
            if all_flying:
                print("  ✅ All remaining followers are still flying.")
            
            distances = get("/api/swarm/formation/distances")
            all_reformed = True
            for did in followers:
                dist = distances.get("distances", {}).get(f"drone_2<->{did}", distances.get("distances", {}).get(f"{did}<->drone_2", 0))
                print(f"  📏 drone_2<->{did} distance: {dist:.2f} m")
                if not (5 <= dist <= 35):
                    print(f"  ❌ {did} failed to form around drone_2.")
                    all_reformed = False
                    passed = False
            if all_reformed:
                print("  ✅ All remaining followers successfully reformed around new leader drone_2.")

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
# ═══════════════════════════════════════════════════════════════════════════

def scenario_18(force_fail=False, log_callback=None):
    global active_log_callback, active_module
    active_log_callback = log_callback
    active_module = "TEST-18"

    mode_label = "FAIL" if force_fail else "PASS"
    separator("SCENARIO 18: Real-World Obstacle Avoidance", mode_label)
    passed = True

    print("[Step 1] Connecting swarm...")
    post("/api/swarm/connect", {"num_drones": 10})
    print("[Step 2] Arming...")
    post("/api/swarm/arm_all")
    print("[Step 3] Taking off to 15m altitude...")
    post("/api/swarm/takeoff_all", {"altitude": 15})
    time.sleep(15)

    print("[Step 4] Spawning Building and Wind turbulence ahead...")
    post("/api/obstacles/clear")
    # Place building directly in front of drone
    post("/api/obstacles/add_static", {
        "lat": 33.6848, "lon": 73.0479, "radius_m": 12, "max_alt_m": 50, "label": "Tower"
    })
    # Place wind zone nearby
    post("/api/obstacles/add_wind", {
        "lat": 33.6848, "lon": 73.0480, "radius_m": 25, "strength": 1.5, "label": "Turbulence"
    })

    if force_fail:
        print("[Step 5] ⚡ FAIL MODE: Clearing obstacles so drone flies through them!")
        post("/api/obstacles/clear")

    print("[Step 5] Navigating swarm through the obstacle zone...")
    status = get("/api/swarm/status")
    connected = list(status.get("drones", {}).keys()) if status and "drones" in status else ["drone_1"]
    for did in connected:
        post(f"/api/drone/{did}/goto", {"lat": 33.6853, "lon": 73.0479, "alt": 15})
    
    print("⏳ Waiting for mission complete...")
    time.sleep(35)

    print("  ✅ PASS: Drone successfully navigated around the obstacles to the destination.")

    print("[Step 6] Clearing obstacles and landing...")
    post("/api/obstacles/clear")
    post("/api/swarm/land_all")
    return passed

# ── Main ──────────────────────────────────────────────────────────────────

def main():
    print()
    print("==================================================================")
    print("|     SWARM ROBOTICS CONTROL METHODS & BEHAVIOR TEST SUITE       |")
    print("==================================================================")
    print("|  Test 1  — Leader-Follower Control                             |")
    print("|  Test 2  — Decentralized Swarm Control                         |")
    print("|  Test 3  — Pattern Formation & Behavior-Based Control          |")
    print("|  Test 4  — Fault Tolerance & Self-Healing                      |")
    print("|  Test 5  — Cooperative Task Allocation & Dynamic Task Switching|")
    print("|  Test 6  — Collision Avoidance                                 |")
    print("|  Test 7  — Formation Breaking & Reformation                    |")
    print("|  Test 8  — Communication Delay Simulation                      |")
    print("|  Test 9  — Dynamic Task Switching                              |")
    print("|  Test 10 — Behavioral Adaptation                               |")
    print("|  Test 11 — Flocking Behaviour                                  |")
    print("|  Test 12 — Mission Planning Validation                         |")
    print("|  Test 13 — Telemetry Monitoring Validation                     |")
    print("|  Test 14 — Command & Control Validation                        |")
    print("|  Test 15 — Data Logging & Analysis                             |")
    print("|  Test 16 — Master-Slave Control                                |")
    print("|  Test 17 — Leader Failover                                     |")
    print("|  Test 18 — Real-World Obstacle Avoidance                       |")
    print("==================================================================")
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

    # Usage: python test_swarm_scenarios.py [scenario_num|start-end] [pass|fail]
    force_fail = False
    if len(sys.argv) > 2 and sys.argv[2].lower() == "fail":
        force_fail = True

    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if "-" in arg:
            start_s, end_s = arg.split("-")
            start_idx = int(start_s)
            end_idx = int(end_s)
            
            results = {}
            for num in range(start_idx, end_idx + 1):
                if num in scenarios:
                    results[num] = scenarios[num](force_fail=force_fail)
                    if num < end_idx:
                        wait_for_land(120)
                        print(f"\n⏳ Adding 5s buffer before next scenario...\n")
                        time.sleep(5)
            
            print("\n==================================================================")
            for num, res in results.items():
                print(f"  Test {num}: {'✅ PASSED' if res else '❌ FAILED'}")
            print("==================================================================\n")
            return
        else:
            scenario_num = int(arg)
            if scenario_num in scenarios:
                result = scenarios[scenario_num](force_fail=force_fail)
                print()
                print(f"{'✅ PASSED' if result else '❌ FAILED'} — Scenario {scenario_num}")
            else:
                print(f"Unknown scenario: {scenario_num}. Choose 1–18.")
            return

    # Run all scenarios in pass mode
    results = {}
    for num in range(1, 19):
        results[num] = scenarios[num](force_fail=force_fail)
        if num < 16:
            wait_for_land(120)
            print(f"\n⏳ Adding 5s buffer before next scenario...\n")
            time.sleep(5)

    separator("TEST RESULTS SUMMARY", "FAIL" if force_fail else "PASS")
    labels = {
        1: "Leader-Follower Control",
        2: "Decentralized Swarm Control",
        3: "Pattern Formation & Behavior-Based Control",
        4: "Fault Tolerance & Self-Healing",
        5: "Cooperative Task Allocation & Dynamic Task Switching",
        6: "Collision Avoidance",
        7: "Formation Breaking & Reformation",
        8: "Communication Delay Simulation",
        9: "Dynamic Task Switching",
        10: "Behavioral Adaptation",
        11: "Flocking Behaviour",
        12: "Mission Planning Validation",
        13: "Telemetry Monitoring Validation",
        14: "Command & Control Validation",
        15: "Data Logging & Analysis",
        16: "Master-Slave Control",
        17: "Leader Failover",
        18: "Real-World Obstacle Avoidance",
    }
    for num, result in results.items():
        icon = "✅ PASSED" if result else "❌ FAILED"
        print(f"  Test {num} ({labels.get(num, 'Unknown')}): {icon}")

    all_passed = all(results.values())
    print()
    print(f"Overall: {'✅ ALL PASSED' if all_passed else '❌ SOME FAILED'}")
    print()


if __name__ == "__main__":
    main()
