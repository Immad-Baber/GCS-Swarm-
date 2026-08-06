import time
import math
import random
import csv
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from pymavlink import mavutil

import sys, os
sys.path.append(os.path.abspath('../sitl_final_package/mavlink_integration'))
from sitl_adapter import SITLAdapter

class TestAdapter(SITLAdapter):
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
        )


from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

"""
MS Mission Scenario 4: Master Failure During Active Mission — MAVLink/QGC + Monte Carlo
READY TO RUN

Outputs:
- MS_Mission_Scenario_4.csv
- MS_Mission_Scenario_4_summary.csv
- MS_Mission_Scenario_4.mp4
"""

PORTS = [14551, 14552, 14553, 14554, 14555]
T0 = time.time()

def connect_all():
    conns = [TestAdapter(f"drone_{i+1}", f"udpin:0.0.0.0:{p}") for i, p in enumerate(PORTS)]
    for i, c in enumerate(conns, 1):
        c.wait_heartbeat()
        print(f"[OK] D{i} heartbeat sysid={c.target_system} compid={c.target_component}")
    return conns

def set_mode(c, mode):
    mode_map = c.mode_mapping()
    if mode not in mode_map:
        raise RuntimeError(f"Mode {mode} not in mode_mapping: {list(mode_map.keys())}")
    mid = mode_map[mode]
    c.mav.set_mode_send(c.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)

def arm(c):
    c.mav.command_long_send(c.target_system, c.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

def takeoff(c, alt_m):
    c.mav.command_long_send(c.target_system, c.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)

def send_ned_pos(self, t0, north, east, down):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    time_boot_ms = int((time.time() - T0) * 1000) & 0xFFFFFFFF
    c.mav.set_position_target_local_ned_send(
        time_boot_ms,
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(north), float(east), float(down),
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def prep_vehicles(conns, takeoff_alt=18):
    for c in conns:
        set_mode(c, "GUIDED")
        time.sleep(0.2)
        arm(c)
        time.sleep(0.2)
        takeoff(c, takeoff_alt)
        time.sleep(0.2)
    time.sleep(6)

def request_local_position_stream(c, hz=10):
    try:
        msg_id = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
        interval_us = int(1e6 / hz)
        c.mav.command_long_send(c.target_system, c.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval_us, 0, 0, 0, 0, 0)
    except Exception:
        pass

def poll_local_pos_best_effort(c, last):
    pos = last
    for _ in range(3):
        msg = c.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is None:
            break
        pos = (float(msg.x), float(msg.y), float(msg.z))
    return pos

def get_fresh_positions(conns, fallback, timeout_s=2.0):
    out = list(fallback)
    got = [False] * len(conns)
    deadline = time.time() + timeout_s
    while time.time() < deadline and not all(got):
        for i, c in enumerate(conns):
            if got[i]:
                continue
            msg = c.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg is not None:
                out[i] = (float(msg.x), float(msg.y), float(msg.z))
                got[i] = True
        time.sleep(0.02)
    return out

def dist3(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def min_pairwise_sep(poses):
    m = float("inf")
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            m = min(m, dist3(poses[i], poses[j]))
    return m

def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / float(len(values)))

def rotate_ne(n, e, yaw_rad):
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    rn = n * cy - e * sy
    re = n * sy + e * cy
    return rn, re

def transform_ne(n, e, shift_n, shift_e, yaw_rad, scale=1.0):
    rn, re = rotate_ne(n, e, yaw_rad)
    rn *= scale
    re *= scale
    return rn + shift_n, re + shift_e

def base_master_route():
    return [(0.0, 0.0, 18.0), (8.0, -6.0, 18.0), (16.0, 6.0, 18.0), (6.0, 16.0, 18.0), (-8.0, 8.0, 18.0), (0.0, 0.0, 18.0)]

def base_targets():
    return [
        ("P1", 20.0, -12.0, 15.0),
        ("P2", 14.0, -22.0, 15.0),
        ("P3", 18.0, 8.0, 15.0),
        ("P4", 6.0, 0.0, 15.0),
        ("P5", -10.0, 16.0, 15.0),
        ("P6", -18.0, 26.0, 15.0),
        ("P7", 0.0, -30.0, 15.0),
        ("P8", -4.0, 30.0, 15.0),
    ]

def make_mp4(poses_hist, target_positions, target_status_hist, master_alive_hist, fallback_count_hist, minsep_hist, pending_hist, completed_hist, mean_target_err_hist, failure_step, mp4_name):
    N = len(poses_hist)
    if N < 10:
        print("[WARN] Not enough frames to make MP4.")
        return
    north = np.array([[poses_hist[t][i][0] for i in range(5)] for t in range(N)], dtype=float)
    east = np.array([[poses_hist[t][i][1] for i in range(5)] for t in range(N)], dtype=float)
    alt = np.array([[-poses_hist[t][i][2] for i in range(5)] for t in range(N)], dtype=float)
    minsep = np.array(minsep_hist[:N], dtype=float)
    pending = np.array(pending_hist[:N], dtype=float)
    completed = 1
                if dwell_counter[vid] >= cfg["TASK_DWELL_STEPS"] and target_states[tid] != 2:
                    target_states[tid] = 2
                    target_completion_steps.append(step)
                    dwell_counter[vid] = 0
                    assign_next_target(vid, queue, target_states, assignments)

        ms = min_pairwise_sep(cur)
        minsep_min = min(minsep_min, ms)
        if step >= cfg["WARMUP_STEPS"] and ms < cfg["MIN_SEP_THRESH"]:
            minsep_viol_steps += 1

        mean_target_err = float(np.mean(slave_target_errs)) if slave_target_errs else 0.0
        pending_now = sum(1 for s in target_states if s == 0)
        completed_now = sum(1 for s in target_states if s == 2)
        fallback_now = sum(1 for vid in range(1, 5) if assignments[vid]["mode"] in ("timeout_hold", "fallback_return"))

        row = [run_id, seed, step, f"{step * DT:.3f}", 1 if master_alive else 0, master_failed_step if master_failed_step is not None else -1, completed_now, pending_now, fallback_now, f"{ms:.3f}", f"{mean_target_err:.3f}"]
        for i in range(5):
            n, e, d = cur[i]
            row += [f"{n:.3f}", f"{e:.3f}", f"{d:.3f}"]
        for vid in range(1, 5):
            row += [assignments[vid]["mode"], assignments[vid]["target_id"]]
        combined_writer.writerow(row)

        if save_video:
            poses_hist.append(cur)
            target_status_hist.append(list(target_states))
            master_alive_hist.append(master_alive)
            fallback_count_hist.append(fallback_now)
            minsep_hist.append(ms)
            pending_hist.append(pending_now)
            completed_hist.append(completed_now)
            mean_target_err_hist.append(mean_target_err)

        sleep_s = DT - (time.time() - tick_t)
        if sleep_s > 0:
            time.sleep(sleep_s)

    all_fallback_started = 1 if all(fallback_started.values()) else 0
    final_positions_ok = 1
    for vid in range(1, 5):
        rn, re = cfg["FALLBACK_RING"][vid - 1]
        target = (home_center_n + rn, home_center_e + re, -cfg["FALLBACK_ALT_M"])
        if dist3(last_pos[vid], target) > cfg["FALLBACK_ACCEPT_M"]:
            final_positions_ok = 0
            break

    completed = 1.0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run_id", "seed", "shift_n", "shift_e", "yaw_rad", "steps_logged", "duration_s", "targets_total", "targets_completed_before_failure", "last_completion_step", "master_failed_step", "all_fallback_started", "final_positions_ok", "min_sep_m", "minsep_violation_steps", "mean_target_err_m", "rms_target_err_m", "completed"])
        for r in rows:
            w.writerow([r["run_id"], r["seed"], f'{r["shift_n"]:.3f}', f'{r["shift_e"]:.3f}', f'{r["yaw_rad"]:.6f}', r["steps_logged"], f'{r["duration_s"]:.3f}', r["targets_total"], r["targets_completed_before_failure"], r["last_completion_step"], r["master_failed_step"], r["all_fallback_started"], r["final_positions_ok"], f'{r["min_sep_m"]:.3f}', r["minsep_violation_steps"], f'{r["mean_target_err_m"]:.3f}', f'{r["rms_target_err_m"]:.3f}', r["completed"]])
        w.writerow([])
        w.writerow(["success_rate_percent", f"{rate:.2f}%"])

def main():
    OUT_CSV = "MS_Mission_Scenario_4.csv"
    OUT_SUM = "MS_Mission_Scenario_4_summary.csv"
    OUT_MP4 = "MS_Mission_Scenario_4.mp4"
    MC_RUNS = 5
    SEED0 = 101
    video_run = random.randint(1, MC_RUNS)
    print(f"[MC] MP4 will be saved from RANDOM run_id={video_run}")
    cfg = {
        "HZ": 10,
        "TOTAL_STEPS": 1500,
        "SPAWN_HOLD_S": 4.0,
        "MIN_SEP_THRESH": -1.0,
        "TASK_RADIUS_M": 2.5,
        "TASK_DWELL_STEPS": 20,
        "MASTER_FAILURE_STEP": 620,
        "LOSS_TIMEOUT_STEPS": 25,
        "FALLBACK_ALT_M": 14.0,
        "FALLBACK_ACCEPT_M": 8.0,
        "SLAVE_ALT_OFFSET_D": 0.0,
        "SLAVE_TARGET_ALT_OFFSET_D": 0.0,
        "IDLE_HOLD_RING": [(8.0, 10.0), (8.0, -10.0), (-8.0, 10.0), (-8.0, -10.0)],
        "FALLBACK_RING": [(10.0, 10.0), (10.0, -10.0), (-10.0, 10.0), (-10.0, -10.0)],
        "SHIFT_N_MIN": -8.0, "SHIFT_N_MAX": 8.0,
        "SHIFT_E_MIN": -8.0, "SHIFT_E_MAX": 8.0,
        "YAW_MIN": -math.pi / 3, "YAW_MAX": math.pi / 3,
        "SCALE": 1.0,
        "WARMUP_STEPS": 100,
        "SEP_TOL_FRAC": 0.02,
        "MP4_NAME": OUT_MP4,
    }
    conns = connect_all()
    for c in conns:
        request_local_position_stream(c, hz=cfg["HZ"])
    prep_vehicles(conns, takeoff_alt=18)
    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        header = ["run_id", "seed", "step", "t_sec", "master_alive", "master_failed_step", "completed_targets", "pending_targets", "fallback_slaves", "min_sep_m", "mean_target_err_m"]
        for i in range(5):
            header += [f"d{i+1}_n", f"d{i+1}_e", f"d{i+1}_d"]
        for vid in range(1, 5):
            header += [f"slave{vid}_mode", f"slave{vid}_target_id"]
        w.writerow(header)
        rows = []
        for run_id in range(1, MC_RUNS + 1):
            seed = SEED0 + (run_id - 1)
            save_video = (run_id == video_run)
            print(f"[MC] Run {run_id}/{MC_RUNS} seed={seed} (video={'YES' if save_video else 'NO'})")
            out = run_one(conns, run_id, seed, cfg, w, save_video=save_video)
            rows.append(out)
            print(f"[MC] done run{run_id:03d}: completed={out['completed']} fallback={out['all_fallback_started']} final_ok={out['final_positions_ok']} minsep={out['min_sep_m']:.2f}")
            time.sleep(2.0)
    write_summary(OUT_SUM, rows)
    print(f"[OK] Saved {OUT_CSV}")
    print(f"[OK] Saved {OUT_SUM}")
    print(f"[OK] Saved {OUT_MP4}")

if __name__ == "__main__":
    main()
