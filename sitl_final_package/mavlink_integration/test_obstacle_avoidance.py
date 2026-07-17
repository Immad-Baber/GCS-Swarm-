"""
test_obstacle_avoidance.py
───────────────────────────────────────────────────────────────────────────────
Real obstacle avoidance tests for the autonomous drone mission simulator.

These tests work WITHOUT a live SITL connection by testing the obstacle_map
module directly (unit tests) AND by testing the avoidance integration via
the API (integration tests, requires server running).

Run unit tests only (no server needed):
    python test_obstacle_avoidance.py --unit

Run all tests (requires telemetry_server.py running):
    python test_obstacle_avoidance.py

Run a specific test:
    python test_obstacle_avoidance.py TestAPF.test_static_repulsion
"""

import sys
import math
import time
import unittest
import requests
import argparse
import logging

# ── Module under test ──────────────────────────────────────────────────────────
from obstacle_map import (
    ObstacleMap, obstacle_map,
    StaticObstacle, WindZone, DynamicObstacle,
)

BASE_URL = "http://127.0.0.1:5000"
logging.basicConfig(level=logging.WARNING)


# ═══════════════════════════════════════════════════════════════════════════════
# UNIT TESTS — no server, no SITL required
# ═══════════════════════════════════════════════════════════════════════════════

class TestAPF(unittest.TestCase):
    """Verify the Artificial Potential Field maths directly."""

    def setUp(self):
        # Fresh map for every test
        self.omap = ObstacleMap()

    def tearDown(self):
        self.omap.stop()

    # ── Static obstacles ───────────────────────────────────────────────────────

    def test_no_obstacles_zero_force(self):
        """With empty map, force must be exactly (0, 0)."""
        dlat, dlon = self.omap.get_avoidance_vector(33.6844, 73.0479, 10.0)
        self.assertAlmostEqual(dlat, 0.0, places=10)
        self.assertAlmostEqual(dlon, 0.0, places=10)

    def test_static_repulsion_direction(self):
        """Drone south of obstacle should be pushed further south."""
        obs = StaticObstacle(lat=33.6850, lon=73.0479, radius_m=5, max_alt_m=50)
        self.omap.add_static(obs)
        # Drone is 10m south of obstacle — force should push it south (negative dlat)
        drone_lat = 33.6850 - (10 / 111320)  # 10 m south
        dlat, dlon = self.omap.get_avoidance_vector(drone_lat, 73.0479, 10.0)
        self.assertLess(dlat, 0, "Force should push drone south (away from obstacle)")

    def test_static_repulsion_magnitude_decreases_with_distance(self):
        """Force must be weaker farther from the obstacle."""
        obs = StaticObstacle(lat=33.6850, lon=73.0479, radius_m=5, max_alt_m=50)
        self.omap.add_static(obs)

        near_lat  = 33.6850 - (8  / 111320)  # 8m south
        far_lat   = 33.6850 - (20 / 111320)  # 20m south

        dlat_near, _ = self.omap.get_avoidance_vector(near_lat, 73.0479, 10.0)
        dlat_far,  _ = self.omap.get_avoidance_vector(far_lat,  73.0479, 10.0)

        self.assertGreater(abs(dlat_near), abs(dlat_far),
                           "Force nearer to obstacle must be stronger")

    def test_static_out_of_range_zero(self):
        """Drone 200 m away from a 5 m obstacle with 25 m influence → zero force."""
        obs = StaticObstacle(lat=33.6850, lon=73.0479, radius_m=5, max_alt_m=50)
        self.omap.add_static(obs)
        far_lat = 33.6850 - (200 / 111320)
        dlat, dlon = self.omap.get_avoidance_vector(far_lat, 73.0479, 10.0)
        self.assertAlmostEqual(dlat, 0.0, places=8)
        self.assertAlmostEqual(dlon, 0.0, places=8)

    def test_static_above_height_ignored(self):
        """Drone flying above obstacle's max_alt_m must see zero force."""
        obs = StaticObstacle(lat=33.6850, lon=73.0479, radius_m=5, max_alt_m=20)
        self.omap.add_static(obs)
        drone_lat = 33.6850 - (8 / 111320)
        # Drone at 36m — well above max_alt + ALT_BAND (20 + 15 = 35)
        dlat, dlon = self.omap.get_avoidance_vector(drone_lat, 73.0479, 36.0)
        self.assertAlmostEqual(dlat, 0.0, places=8,
                               msg="Drone above obstacle should not be repelled")

    # ── Wind zones ────────────────────────────────────────────────────────────

    def test_wind_repulsion_exists(self):
        """Drone inside wind-zone influence should receive non-zero force."""
        zone = WindZone(lat=33.6850, lon=73.0479, radius_m=15, strength=2.0)
        self.omap.add_wind(zone)
        drone_lat = 33.6850 - (20 / 111320)  # 20m south, just inside influence
        dlat, dlon = self.omap.get_avoidance_vector(drone_lat, 73.0479, 10.0)
        force_m = abs(dlat) * 111320
        self.assertGreater(force_m, 0.0, "Wind zone must generate a repulsive force")

    def test_wind_strength_scales_force(self):
        """Higher strength factor must produce proportionally larger force."""
        z1 = WindZone(lat=33.6850, lon=73.0479, radius_m=15, strength=1.0)
        z2 = WindZone(lat=33.6850, lon=73.0479, radius_m=15, strength=3.0)

        m1 = ObstacleMap()
        m2 = ObstacleMap()
        m1.add_wind(z1)
        m2.add_wind(z2)

        drone_lat = 33.6850 - (20 / 111320)
        d1, _ = m1.get_avoidance_vector(drone_lat, 73.0479, 10.0)
        d2, _ = m2.get_avoidance_vector(drone_lat, 73.0479, 10.0)

        m1.stop(); m2.stop()

        self.assertGreater(abs(d2), abs(d1),
                           "Higher strength wind zone must produce larger force")

    # ── Dynamic obstacles ─────────────────────────────────────────────────────

    def test_dynamic_repulsion_at_same_altitude(self):
        """Bird at the same altitude and 15m away must produce a force."""
        bird = DynamicObstacle(lat=33.6850, lon=73.0479, alt_m=10,
                               radius_m=5, label="bird")
        self.omap.add_dynamic(bird)
        drone_lat = 33.6850 - (15 / 111320)
        dlat, dlon = self.omap.get_avoidance_vector(drone_lat, 73.0479, alt=10.0)
        self.assertNotEqual(dlat, 0.0, "Bird at same altitude should push drone")

    def test_dynamic_different_altitude_ignored(self):
        """Bird 50m above drone must NOT affect it."""
        bird = DynamicObstacle(lat=33.6850, lon=73.0479, alt_m=60,
                               radius_m=5, label="bird-high")
        self.omap.add_dynamic(bird)
        drone_lat = 33.6850 - (10 / 111320)
        dlat, dlon = self.omap.get_avoidance_vector(drone_lat, 73.0479, alt=10.0)
        self.assertAlmostEqual(dlat, 0.0, places=8,
                               msg="Bird 50m above should not affect drone")

    def test_dynamic_obstacle_moves(self):
        """Dynamic obstacle must move over time."""
        bird = DynamicObstacle(lat=33.6850, lon=73.0479, alt_m=10,
                               radius_m=5, vel_lat_dps=0.001, vel_lon_dps=0.0)
        self.omap.add_dynamic(bird)
        initial_lat = bird.lat
        time.sleep(0.5)  # let the background thread tick
        self.assertGreater(bird.lat, initial_lat,
                           "Dynamic obstacle must move northward over time")

    # ── Clear ─────────────────────────────────────────────────────────────────

    def test_clear_removes_all(self):
        """After clear(), the map must return (0,0) everywhere."""
        self.omap.add_static(StaticObstacle(33.6850, 73.0479, 5, 50))
        self.omap.add_wind(WindZone(33.6840, 73.0479, 15))
        self.omap.add_dynamic(DynamicObstacle(33.6860, 73.0479, 10, 5))
        self.omap.clear()
        dlat, dlon = self.omap.get_avoidance_vector(33.6850, 73.0479, 10.0)
        self.assertAlmostEqual(dlat, 0.0, places=10)
        self.assertAlmostEqual(dlon, 0.0, places=10)

    def test_multiple_obstacles_forces_add(self):
        """Two obstacles on the same side should produce a larger force than one."""
        obs1 = StaticObstacle(33.6850, 73.0479, 5, 50)
        obs2 = StaticObstacle(33.6851, 73.0479, 5, 50)

        m_single = ObstacleMap()
        m_double = ObstacleMap()
        m_single.add_static(obs1)
        m_double.add_static(obs1)
        m_double.add_static(obs2)

        drone_lat = 33.6850 - (8 / 111320)
        d1, _ = m_single.get_avoidance_vector(drone_lat, 73.0479, 10.0)
        d2, _ = m_double.get_avoidance_vector(drone_lat, 73.0479, 10.0)

        m_single.stop(); m_double.stop()

        self.assertGreater(abs(d2), abs(d1),
                           "Two obstacles must produce greater combined force")


# ═══════════════════════════════════════════════════════════════════════════════
# INTEGRATION TESTS — require telemetry_server.py running on port 5000
# ═══════════════════════════════════════════════════════════════════════════════

def _api(method, path, body=None):
    url = BASE_URL + path
    try:
        if method == "POST":
            r = requests.post(url, json=body or {}, timeout=10)
        else:
            r = requests.get(url, timeout=10)
        r.raise_for_status()
        return r.json()
    except requests.exceptions.ConnectionError:
        return None
    except Exception as e:
        return {"error": str(e)}


def _server_up() -> bool:
    r = _api("GET", "/api/obstacles/status")
    return r is not None and r.get("status") == "ok"


class TestObstacleAPI(unittest.TestCase):
    """Test obstacle REST API against a running telemetry_server.py."""

    @classmethod
    def setUpClass(cls):
        if not _server_up():
            raise unittest.SkipTest(
                "telemetry_server.py not reachable on port 5000 — "
                "start the server first to run integration tests."
            )

    def setUp(self):
        _api("POST", "/api/obstacles/clear")

    def tearDown(self):
        _api("POST", "/api/obstacles/clear")

    def test_status_empty(self):
        r = _api("GET", "/api/obstacles/status")
        self.assertEqual(r["status"], "ok")
        obs = r["obstacles"]
        self.assertEqual(len(obs["static"]), 0)
        self.assertEqual(len(obs["wind"]),   0)
        self.assertEqual(len(obs["dynamic"]), 0)

    def test_add_static(self):
        r = _api("POST", "/api/obstacles/add_static", {
            "lat": 33.6850, "lon": 73.0479, "radius_m": 10,
            "max_alt_m": 50, "label": "test-wall"
        })
        self.assertEqual(r["status"], "ok")
        status = _api("GET", "/api/obstacles/status")
        self.assertEqual(len(status["obstacles"]["static"]), 1)
        self.assertEqual(status["obstacles"]["static"][0]["label"], "test-wall")

    def test_add_wind_zone(self):
        r = _api("POST", "/api/obstacles/add_wind", {
            "lat": 33.6852, "lon": 73.0482, "radius_m": 20,
            "strength": 1.5, "label": "wind-building"
        })
        self.assertEqual(r["status"], "ok")
        status = _api("GET", "/api/obstacles/status")
        self.assertEqual(len(status["obstacles"]["wind"]), 1)

    def test_add_dynamic_bird(self):
        r = _api("POST", "/api/obstacles/add_dynamic", {
            "lat": 33.6848, "lon": 73.0479, "alt_m": 15, "radius_m": 8,
            "vel_lat_dps": -0.00005, "vel_lon_dps": 0.00003,
            "label": "bird-1"
        })
        self.assertEqual(r["status"], "ok")
        status = _api("GET", "/api/obstacles/status")
        self.assertEqual(len(status["obstacles"]["dynamic"]), 1)
        self.assertEqual(status["obstacles"]["dynamic"][0]["label"], "bird-1")

    def test_clear(self):
        _api("POST", "/api/obstacles/add_static",
             {"lat": 33.6850, "lon": 73.0479, "radius_m": 10})
        _api("POST", "/api/obstacles/add_wind",
             {"lat": 33.6852, "lon": 73.0482, "radius_m": 20})
        _api("POST", "/api/obstacles/clear")
        status = _api("GET", "/api/obstacles/status")
        self.assertEqual(len(status["obstacles"]["static"]), 0)
        self.assertEqual(len(status["obstacles"]["wind"]),   0)

    def test_multiple_obstacles(self):
        for i in range(3):
            _api("POST", "/api/obstacles/add_static", {
                "lat": 33.6850 + i * 0.0001,
                "lon": 73.0479,
                "radius_m": 10,
                "label": f"wall-{i}"
            })
        status = _api("GET", "/api/obstacles/status")
        self.assertEqual(len(status["obstacles"]["static"]), 3)


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Obstacle avoidance test suite")
    parser.add_argument("--unit", action="store_true",
                        help="Run unit tests only (no server required)")
    args, remaining = parser.parse_known_args()

    print("=" * 70)
    print("  AUTONOMOUS DRONE OBSTACLE AVOIDANCE TEST SUITE")
    print("=" * 70)

    if args.unit:
        print("  Mode: UNIT TESTS ONLY (no server required)")
        suite = unittest.TestLoader().loadTestsFromTestCase(TestAPF)
    else:
        if _server_up():
            print(f"  Mode: FULL SUITE (server found at {BASE_URL})")
            suite = unittest.TestSuite([
                unittest.TestLoader().loadTestsFromTestCase(TestAPF),
                unittest.TestLoader().loadTestsFromTestCase(TestObstacleAPI),
            ])
        else:
            print("  WARNING: Server not reachable. Running unit tests only.")
            print(f"  Start telemetry_server.py and re-run for integration tests.")
            suite = unittest.TestLoader().loadTestsFromTestCase(TestAPF)

    print()
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
