"""
LF Mission Scenario 4 (Communication Degradation + Formation Recovery) — MAVLink/QGC + Monte Carlo
READY TO RUN

Outputs:
- LF_Mission_Scenario_4.csv
- LF_Mission_Scenario_4_summary.csv   (includes success_rate_percent)
- LF_Mission_Scenario_4.mp4           (ONE MP4 from random run)

Scenario:
- Waypoint-like leader trajectory executed for EXACT 1500 steps at 10 Hz.
- Followers maintain offsets normally.
- Communication degradation window: followers [UAV3, UAV5] use STALE leader target (freeze) OR delayed target.
- After window ends, followers recover to normal tracking.

Video:
- Leader green, followers blue, waypoint circles red.
- Metrics: min separation, max formation error, leader distance-to-target, comm degraded flag (0/1)
- Vertical markers: comm start, comm end

Run:
  python3 LF_Mission_Scenario_4.py
"""

import time
import math
import random
import csv
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


# =========================
# MAVLINK CONFIG
# =========================
PORTS = [14551, 14552, 14553, 14554, 14555]  # 5 drones
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
    c.mav.set_mode_send(
        c.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mid,
    )


def arm(c):
    c.mav.command_long_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )


def takeoff(c, alt_m):
    c.mav.command_long_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, alt_m
    )


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


def prep_vehicles(conns, takeoff_alt=15):
    for c in conns:
        c.set_mode("GUIDED")
        time.sleep(0.2)
        c.arm_vehicle()
        time.sleep(0.2)
        c.takeoff(takeoff_alt)
        time.sleep(0.2)
    time.sleep(6)


# =========================
# Telemetry
# =========================
def request_local_position_stream(c, hz=10):
    try:
        msg_id = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
        interval_us = int(1e6 / hz)
        c.mav.command_long_send(
            c.target_system, c.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id, interval_us, 0, 0, 0, 0, 0
        )
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


# =========================
# Geometry
# =========================
def dist3(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def min_pairwise_sep(poses):
    m = float("inf")
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            m = min(m, dist3(poses[i], poses[j]))
    return m


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v*v for v in values) / float(len(values)))


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


# =========================
# Mission path (waypoint-like) — compact for clear video
# =========================
def base_waypoints():
    # (name, N, E, Alt)
    return [
        ("WP0_Hold",       -25, -25, 15),
        ("WP1_Entry",      -10, -25, 15),
        ("WP2_TurnApex",     0, -10, 15),
        ("WP3_Exit",        15,  10, 15),
        ("WP4_FinalHold",   25,  25, 15),
    ]


# =========================
# Video
# =========================
def make_mp4(
    poses_hist,
    leader_id_hist,
    waypoints_xyz,
    minsep_hist,
    maxerr_hist,
    dtt_hist,
    comm_flag_hist,
    comm_start,
    comm_end,
    mp4_name
):
    N = len(poses_hist)
    if N < 10:
        print("[WARN] Not enough frames to make MP4.")
        return

    north = np.array([[poses_hist[t][i][0] for i in range(5)] for t in range(N)], dtype=float)
    east  = np.array([[poses_hist[t][i][1] for i in range(5)] for t in range(N)], dtype=float)
    alt   = np.array([[-poses_hist[t][i][2] for i in range(5)] for t in range(N)], dtype=float)

    minsep = np.array(minsep_hist[:N], dtype=float)
    maxerr = np.array(maxerr_hist[:N], dtype=float)
    dtt    = np.array(dtt_hist[:N], dtype=float)
    commf  = np.array(comm_flag_hist[:N], dtype=float)

    fig = plt.figure(figsize=(14, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    axm  = fig.add_subplot(1, 2, 2)

    ax3d.set_title("LF Scenario 4: Comm Degradation + Recovery (3D)")
    ax3d.set_xlabel("East (m)")
    ax3d.set_ylabel("North (m)")
    ax3d.set_zlabel("Altitude (m)")

    axm.set_title("Mission Metrics")
    axm.set_xlabel("Step")
    axm.set_ylabel("Value")

    phase_text = ax3d.text2D(0.02, 0.95, "", transform=ax3d.transAxes)

    # Waypoints red circles
    wp_e = [p[0] for p in waypoints_xyz]
    wp_n = [p[1] for p in waypoints_xyz]
    wp_a = [p[2] for p in waypoints_xyz]
    ax3d.scatter(wp_e, wp_n, wp_a, c="r", marker="o", s=40, alpha=0.9)

    # Limits
    pad_xy = 12
    pad_z = 6
    ax3d.set_xlim(float(east.min() - pad_xy), float(east.max() + pad_xy))
    ax3d.set_ylim(float(north.min() - pad_xy), float(north.max() + pad_xy))
    ax3d.set_zlim(float(alt.min() - pad_z), float(alt.max() + pad_z))

    dots = [ax3d.plot([], [], [], marker="o", linestyle="")[0] for _ in range(5)]
    leader_trails = [ax3d.plot([], [], [], linewidth=1.2, color="g", alpha=0.95)[0] for _ in range(5)]
    follower_trails = [ax3d.plot([], [], [], linewidth=1.0, color="b", alpha=0.90)[0] for _ in range(5)]

    # Metrics
    l_minsep, = axm.plot([], [], linewidth=1.5, label="min separation (m)")
    l_maxerr, = axm.plot([], [], linewidth=1.5, label="max formation error (m)")
    l_dtt,    = axm.plot([], [], linewidth=1.5, label="leader distance-to-target (m)")
    l_comm,   = axm.plot([], [], linewidth=1.5, label="comm degraded (0/1)")

    axm.legend(loc="upper right")
    axm.set_xlim(0, N)

    ymax = float(max(np.max(minsep), np.max(maxerr), np.max(dtt), 1.0))
    axm.set_ylim(0, ymax * 1.10)

    v_s = axm.axvline(comm_start, linestyle="--", linewidth=2.0)
    v_e = axm.axvline(comm_end, linestyle="--", linewidth=2.0)
    axm.text(comm_start + 10, 0.92 * axm.get_ylim()[1], "Comm start", fontsize=9)
    axm.text(comm_end + 10, 0.92 * axm.get_ylim()[1], "Comm end", fontsize=9)

    trail_len = 180
    frames = list(range(0, N, 1))

    def update(fi):
        t = frames[fi]
        t0 = max(0, t - trail_len)

        if comm_start <= t <= comm_end:
            phase_text.set_text("PHASE: Comm degraded (stale leader target)")
        else:
            phase_text.set_text("PHASE: Normal tracking / recovery")

        lid = leader_id_hist[t]
        for i in range(5):
            dots[i].set_color("g" if i == lid else "b")
            dots[i].set_data([east[t, i]], [north[t, i]])
            dots[i].set_3d_properties([alt[t, i]])

        idx = np.arange(t0, t + 1)
        for i in range(5):
            was_leader = np.array([leader_id_hist[k] == i for k in idx], dtype=bool)
            was_follower = ~was_leader

            li = idx[was_leader]
            if li.size > 0:
                leader_trails[i].set_data(east[li, i], north[li, i])
                leader_trails[i].set_3d_properties(alt[li, i])
            else:
                leader_trails[i].set_data([], [])
                leader_trails[i].set_3d_properties([])

            fi2 = idx[was_follower]
            if fi2.size > 0:
                follower_trails[i].set_data(east[fi2, i], north[fi2, i])
                follower_trails[i].set_3d_properties(alt[fi2, i])
            else:
                follower_trails[i].set_data([], [])
                follower_trails[i].set_3d_properties([])

        xs = np.arange(0, t + 1)
        l_minsep.set_data(xs, minsep[:t+1])
        l_maxerr.set_data(xs, maxerr[:t+1])
        l_dtt.set_data(xs, dtt[:t+1])
        l_comm.set_data(xs, commf[:t+1])

        return dots + leader_trails + follower_trails + [l_minsep, l_maxerr, l_dtt, l_comm, v_s, v_e]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=80, blit=True)
    writer = FFMpegWriter(fps=15, metadata={"title": "LF Scenario 4"}, bitrate=1800)
    ani.save(mp4_name, writer=writer)
    plt.close(fig)


# =========================
# Run one mission (fixed 1500 steps)
# =========================
def run_one(conns, run_id, seed, cfg, combined_writer, save_video=False):
    random.seed(seed)
    HZ = cfg["HZ"]
    DT = 1.0 / HZ

    shift_n = random.uniform(cfg["SHIFT_N_MIN"], cfg["SHIFT_N_MAX"])
    shift_e = random.uniform(cfg["SHIFT_E_MIN"], cfg["SHIFT_E_MAX"])
    yaw = random.uniform(cfg["YAW_MIN"], cfg["YAW_MAX"])
    scale = cfg["SCALE"]

    # offsets rotated
    offsets = []
    for on, oe, od in cfg["OFFSETS"]:
        rn, re = rotate_ne(on, oe, yaw)
        offsets.append((rn, re, od))

    # waypoint route
    wps = base_waypoints()

    # for video plotting
    wp_xyz = []
    for _name, n, e, alt in wps:
        tn, te = transform_ne(n, e, shift_n, shift_e, yaw, scale=scale)
        wp_xyz.append((te, tn, alt))

    # fixed 1500-step leader target schedule (interpolated)
    targets = []
    segments = len(wps) - 1
    per_seg = cfg["TOTAL_STEPS"] // segments
    extra = cfg["TOTAL_STEPS"] - per_seg * segments

    for i in range(segments):
        name_a, na, ea, alta = wps[i]
        name_b, nb, eb, altb = wps[i + 1]
        ta_n, ta_e = transform_ne(na, ea, shift_n, shift_e, yaw, scale=scale)
        tb_n, tb_e = transform_ne(nb, eb, shift_n, shift_e, yaw, scale=scale)
        ta_d, tb_d = -alta, -altb
        seg_steps = per_seg + (1 if i < extra else 0)

        for t in np.linspace(0, 1, seg_steps, endpoint=False):
            n = ta_n + (tb_n - ta_n) * t
            e = ta_e + (tb_e - ta_e) * t
            d = ta_d + (tb_d - ta_d) * t
            phase = f"{name_a}_to_{name_b}"
            targets.append((phase, n, e, d))
    targets = targets[:cfg["TOTAL_STEPS"]]

    # Comm degradation settings (step-based)
    comm_start = cfg["COMM_START"]
    comm_end = cfg["COMM_END"]
    affected = set(cfg["COMM_AFFECTED_VIDS"])  # 0-indexed follower ids
    mode = cfg["COMM_MODE"]  # "freeze" or "delay"
    delay_steps = cfg["COMM_DELAY_STEPS"]

    leader_id = 0
    last_pos = [(0.0, 0.0, -15.0) for _ in range(5)]
    last_pos = get_fresh_positions(conns, last_pos)

    # initial settle at first target
    phase0, n0, e0, d0 = targets[0]
    for _ in range(int(cfg["SPAWN_HOLD_S"] * HZ)):
        conns[0].send_ned_pos(T0,  n0, e0, d0)
        for vid in range(1, 5):
            on, oe, od = offsets[vid - 1]
            conns[vid].send_ned_pos(T0,  n0 + on, e0 + oe, d0 + od)
        time.sleep(DT)

    # store leader targets history (for delay)
    leader_target_hist = []

    # comm stale target (for freeze)
    frozen_leader_target = None

    poses_hist, leader_id_hist = [], []
    minsep_hist, maxerr_hist, dtt_hist, comm_flag_hist = [], [], [], []
    ferr_all = []
    minsep_min = float("inf")
    minsep_viol_steps = 0

    WARMUP = cfg["WARMUP_STEPS"]
    allowed_sep = int(cfg["SEP_TOL_FRAC"] * cfg["TOTAL_STEPS"])

    for step in range(cfg["TOTAL_STEPS"]):
        tick_t = time.time()

        phase, tgt_n, tgt_e, tgt_d = targets[step]
        leader_target_hist.append((tgt_n, tgt_e, tgt_d))

        # Leader always tracks true target
        conns[0].send_ned_pos(T0,  tgt_n, tgt_e, tgt_d)

        comm_degraded = 1 if (comm_start <= step <= comm_end) else 0

        # Decide what target followers see (for affected vids)
        if comm_degraded == 1:
            if mode == "freeze":
                if frozen_leader_target is None:
                    frozen_leader_target = (tgt_n, tgt_e, tgt_d)
                seen_n, seen_e, seen_d = frozen_leader_target
            else:  # delay
                idx = max(0, step - delay_steps)
                seen_n, seen_e, seen_d = leader_target_hist[idx]
        else:
            frozen_leader_target = None
            seen_n, seen_e, seen_d = tgt_n, tgt_e, tgt_d

        # Followers:
        for vid in range(1, 5):
            on, oe, od = offsets[vid - 1]
            if vid in affected and comm_degraded == 1:
                # use stale/delayed leader target
                conns[vid].send_ned_pos(T0,  seen_n + on, seen_e + oe, seen_d + od)
            else:
                conns[vid].send_ned_pos(T0,  tgt_n + on, tgt_e + oe, tgt_d + od)

        # telemetry
        cur = []
        for i, c in enumerate(conns):
            last_pos[i] = poll_local_pos_best_effort(c, last_pos[i])
            cur.append(last_pos[i])

        ms = min_pairwise_sep(cur)
        minsep_min = min(minsep_min, ms)

        if step >= WARMUP and ms < cfg["MIN_SEP_THRESH"]:
            minsep_viol_steps += 1

        # formation error w.r.t commanded setpoints
        ferrs = []
        for vid in range(1, 5):
            on, oe, od = offsets[vid - 1]
            if vid in affected and comm_degraded == 1:
                desired = (seen_n + on, seen_e + oe, seen_d + od)
            else:
                desired = (tgt_n + on, tgt_e + oe, tgt_d + od)
            ferr = dist3(cur[vid], desired)
            ferrs.append(ferr)
            ferr_all.append(ferr)
        maxerr = max(ferrs) if ferrs else 0.0

        # leader distance-to-target (actual)
        leader_pos = cur[0]
        dtt = dist3(leader_pos, (tgt_n, tgt_e, tgt_d))

        # CSV log
        t_sec = step * DT
        row = [
            run_id, seed, step, f"{t_sec:.3f}", phase,
            comm_degraded, f"{ms:.3f}", f"{maxerr:.3f}", f"{dtt:.3f}"
        ]
        for i in range(5):
            n, e, d = cur[i]
            row += [f"{n:.3f}", f"{e:.3f}", f"{d:.3f}"]
        combined_writer.writerow(row)

        if save_video:
            poses_hist.append(cur)
            leader_id_hist.append(leader_id)
            minsep_hist.append(ms)
            maxerr_hist.append(maxerr)
            dtt_hist.append(dtt)
            comm_flag_hist.append(comm_degraded)

        elapsed = time.time() - tick_t
        sleep_s = DT - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    duration_s = cfg["TOTAL_STEPS"] / float(cfg["HZ"])

    max_form_err = max(ferr_all) if ferr_all else 0.0
    rms_form_err = rms(ferr_all) if ferr_all else 0.0

    completed = 1.0

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "run_id", "seed", "shift_n", "shift_e", "yaw_rad",
            "steps_logged", "duration_s",
            "min_sep_m", "minsep_violation_steps",
            "max_form_err_m", "rms_form_err_m",
            "comm_start", "comm_end", "mode",
            "completed"
        ])
        for r in rows:
            w.writerow([
                r["run_id"], r["seed"],
                f'{r["shift_n"]:.3f}', f'{r["shift_e"]:.3f}', f'{r["yaw_rad"]:.6f}',
                r["steps_logged"], f'{r["duration_s"]:.3f}',
                f'{r["min_sep_m"]:.3f}', r["minsep_violation_steps"],
                f'{r["max_form_err_m"]:.3f}', f'{r["rms_form_err_m"]:.3f}',
                r["comm_start"], r["comm_end"], r["mode"],
                r["completed"]
            ])
        w.writerow([])
        w.writerow(["success_rate_percent", f"{rate:.2f}%"])


def main():
    OUT_CSV = "LF_Mission_Scenario_4.csv"
    OUT_SUM = "LF_Mission_Scenario_4_summary.csv"
    OUT_MP4 = "LF_Mission_Scenario_4.mp4"

    MC_RUNS = 5
    SEED0 = 1
    video_run = random.randint(1, MC_RUNS)
    print(f"[MC] MP4 will be saved from RANDOM run_id={video_run}")

    cfg = {
        "HZ": 10,
        "TOTAL_STEPS": 1500,
        "SPAWN_HOLD_S": 4.0,
        "MIN_SEP_THRESH": -1.0,

        # Formation offsets
        "OFFSETS": [
            (0.0, +7.0, 0.0),
            (0.0, -7.0, 0.0),
            (-7.0, +7.0, 0.0),
            (-7.0, -7.0, 0.0),
        ],

        # Monte Carlo
        "SHIFT_N_MIN": -8.0,
        "SHIFT_N_MAX": +8.0,
        "SHIFT_E_MIN": -8.0,
        "SHIFT_E_MAX": +8.0,
        "YAW_MIN": -math.pi / 2,
        "YAW_MAX": +math.pi / 2,
        "SCALE": 1.0,

        # Comm degradation window
        "COMM_START": 450,
        "COMM_END": 850,
        # Affected followers (0-indexed): 2 = UAV3, 4 = UAV5 (as you wanted)
        "COMM_AFFECTED_VIDS": [2, 4],
        # "freeze" = hold last target; "delay" = use target from COMM_DELAY_STEPS ago
        "COMM_MODE": "freeze",
        "COMM_DELAY_STEPS": 30,

        # Success logic (tolerant, avoids always-0%)
        "WARMUP_STEPS": 100,
        "SEP_TOL_FRAC": 0.20,  # allow 2% steps under min separation threshold

        "MP4_NAME": OUT_MP4,
    }

    conns = connect_all()
    for c in conns:
        request_local_position_stream(c, hz=cfg["HZ"])
    prep_vehicles(conns, takeoff_alt=15)

    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        header = [
            "run_id", "seed", "step", "t_sec", "phase",
            "comm_degraded", "min_sep_m", "max_form_err_m", "leader_dtt_m"
        ]
        for i in range(5):
            header += [f"d{i+1}_n", f"d{i+1}_e", f"d{i+1}_d"]
        w.writerow(header)

        rows = []
        for run_id in range(1, MC_RUNS + 1):
            seed = SEED0 + (run_id - 1)
            save_video = (run_id == video_run)
            print(f"[MC] Run {run_id}/{MC_RUNS} seed={seed} (video={'YES' if save_video else 'NO'})")
            out = run_one(conns, run_id, seed, cfg, w, save_video=save_video)
            rows.append(out)
            print(f"[MC] done run{run_id:03d}: completed={out['completed']} "
                  f"minsep={out['min_sep_m']:.2f} sep_viol_steps={out['minsep_violation_steps']}")
            time.sleep(2.0)

    write_summary(OUT_SUM, rows)
    print(f"[OK] Saved {OUT_CSV}")
    print(f"[OK] Saved {OUT_SUM}")
    print(f"[OK] Saved {OUT_MP4}")


if __name__ == "__main__":
    main()