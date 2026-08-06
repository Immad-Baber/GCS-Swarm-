"""
DC Mission Scenario 2 — Decentralized Obstacle Avoidance — MAVLink/QGC + Monte Carlo
READY TO RUN

Outputs:
- DC_Mission_Scenario_2.csv
- DC_Mission_Scenario_2_summary.csv
- DC_Mission_Scenario_2.mp4   (ONE MP4 from RANDOM run)

Scenario:
- 5 UAVs navigate a waypoint route under decentralized control.
- No explicit leader command logic for avoidance; each UAV applies its own local obstacle-avoidance correction.
- Static obstacles are placed along the route.
- UAVs try to preserve nominal relative geometry while independently avoiding obstacles.

Video:
- UAV1 shown green
- UAV2–UAV5 shown blue
- Waypoints shown red circles
- Obstacles shown black X markers
- Metrics panel shows:
    1) min separation
    2) max formation error
    3) min obstacle distance
    4) obstacle warning flag (0/1)

Run:
  python3 DC_Mission_Scenario_2.py
"""

import time
import math
import random
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from pymavlink import mavutil
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# =========================
# MAVLINK CONFIG
# =========================
PORTS = [14551, 14552, 14553, 14554, 14555]
T0 = time.time()


def connect_all():
    conns = [mavutil.mavlink_connection(f"udpin:0.0.0.0:{p}") for p in PORTS]
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


def send_ned_pos(c, north, east, down):
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
        c.target_system,
        c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(north), float(east), float(down),
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def prep_vehicles(conns, takeoff_alt=15):
    for c in conns:
        set_mode(c, "GUIDED")
        time.sleep(0.2)
        arm(c)
        time.sleep(0.2)
        takeoff(c, takeoff_alt)
        time.sleep(0.2)
    time.sleep(6)


# =========================
# TELEMETRY
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
# GEOMETRY / HELPERS
# =========================
def dist3(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def dist2_ne(a_n, a_e, b_n, b_e):
    dx = a_n - b_n
    dy = a_e - b_e
    return math.sqrt(dx * dx + dy * dy)


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


# =========================
# MISSION
# =========================
def base_waypoints():
    return [
        ("WP0_Hold", -25, -25, 15),
        ("WP1_Entry", -10, -25, 15),
        ("WP2_TurnApex", 0, -10, 15),
        ("WP3_ObstacleCorridor", 12, 5, 15),
        ("WP4_Exit", 20, 16, 15),
        ("WP5_Final", 28, 26, 15),
    ]


def generate_obstacles(seed):
    rng = random.Random(seed)
    base = [
        (-5.0, -15.0),
        (5.0, -5.0),
        (12.0, 5.0),
        (18.0, 12.0),
    ]
    out = []
    for n, e in base:
        out.append((
            n + rng.uniform(-1.0, 1.0),
            e + rng.uniform(-1.0, 1.0),
        ))
    return out


def obstacle_avoidance(pos, obstacles, safe_radius=6.0, gain=5.0):
    avoid_n, avoid_e = 0.0, 0.0
    for on, oe in obstacles:
        dx = pos[0] - on
        dy = pos[1] - oe
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < safe_radius:
            strength = (safe_radius - dist) / safe_radius
            avoid_n += (dx / (dist + 1e-6)) * strength * gain
            avoid_e += (dy / (dist + 1e-6)) * strength * gain

    return avoid_n, avoid_e


def min_obstacle_dist(pos, obstacles):
    return min(dist2_ne(pos[0], pos[1], on, oe) for on, oe in obstacles)


# =========================
# VIDEO
# =========================
def make_mp4(
    poses_hist,
    leader_id_hist,
    waypoints_xyz,
    minsep_hist,
    maxerr_hist,
    minobs_hist,
    obswarn_hist,
    obstacles_xyz,
    mp4_name
):
    N = len(poses_hist)
    if N < 10:
        print("[WARN] Not enough frames to make MP4.")
        return

    north = np.array([[poses_hist[t][i][0] for i in range(5)] for t in range(N)], dtype=float)
    east = np.array([[poses_hist[t][i][1] for i in range(5)] for t in range(N)], dtype=float)
    alt = np.array([[-poses_hist[t][i][2] for i in range(5)] for t in range(N)], dtype=float)

    minsep = np.array(minsep_hist[:N], dtype=float)
    maxerr = np.array(maxerr_hist[:N], dtype=float)
    minobs = np.array(minobs_hist[:N], dtype=float)
    obswarn = np.array(obswarn_hist[:N], dtype=float)

    fig = plt.figure(figsize=(14, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    axm = fig.add_subplot(1, 2, 2)

    ax3d.set_title("DC Scenario 2: Decentralized Obstacle Avoidance (3D)")
    ax3d.set_xlabel("East (m)")
    ax3d.set_ylabel("North (m)")
    ax3d.set_zlabel("Altitude (m)")

    axm.set_title("Mission Metrics")
    axm.set_xlabel("Step")
    axm.set_ylabel("Value")

    # waypoints
    wp_e = [p[0] for p in waypoints_xyz]
    wp_n = [p[1] for p in waypoints_xyz]
    wp_a = [p[2] for p in waypoints_xyz]
    ax3d.scatter(wp_e, wp_n, wp_a, c="r", marker="o", s=40, alpha=0.9)

    # obstacles
    obs_e = [p[0] for p in obstacles_xyz]
    obs_n = [p[1] for p in obstacles_xyz]
    obs_a = [p[2] for p in obstacles_xyz]
    ax3d.scatter(obs_e, obs_n, obs_a, c="k", marker="x", s=90)

    pad_xy = 12
    pad_z = 6
    ax3d.set_xlim(float(east.min() - pad_xy), float(east.max() + pad_xy))
    ax3d.set_ylim(float(north.min() - pad_xy), float(north.max() + pad_xy))
    ax3d.set_zlim(float(alt.min() - pad_z), float(alt.max() + pad_z))

    dots = [ax3d.plot([], [], [], marker="o", linestyle="")[0] for _ in range(5)]
    leader_trails = [ax3d.plot([], [], [], linewidth=1.2, color="g", alpha=0.95)[0] for _ in range(5)]
    follower_trails = [ax3d.plot([], [], [], linewidth=1.0, color="b", alpha=0.90)[0] for _ in range(5)]

    l_minsep, = axm.plot([], [], linewidth=1.5, label="min separation (m)")
    l_maxerr, = axm.plot([], [], linewidth=1.5, label="max formation error (m)")
    l_minobs, = axm.plot([], [], linewidth=1.5, label="min obstacle dist (m)")
    l_obswarn, = axm.plot([], [], linewidth=1.5, label="obstacle warning (0/1)")

    axm.legend(loc="upper right")
    axm.set_xlim(0, N)
    ymax = float(max(np.max(minsep), np.max(maxerr), np.max(minobs), 1.0))
    axm.set_ylim(0, ymax * 1.10)

    trail_len = 180
    frames = list(range(0, N, 1))

    def update(fi):
        t = frames[fi]
        t0 = max(0, t - trail_len)

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
        l_minsep.set_data(xs, minsep[:t + 1])
        l_maxerr.set_data(xs, maxerr[:t + 1])
        l_minobs.set_data(xs, minobs[:t + 1])
        l_obswarn.set_data(xs, obswarn[:t + 1])

        return dots + leader_trails + follower_trails + [l_minsep, l_maxerr, l_minobs, l_obswarn]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=80, blit=True)
    writer = FFMpegWriter(fps=15, metadata={"title": "DC Scenario 2"}, bitrate=1800)
    ani.save(mp4_name, writer=writer)
    plt.close(fig)


# =========================
# RUN ONE
# =========================
def run_one(conns, run_id, seed, cfg, writer_csv, save_video=False):
    random.seed(seed)
    HZ = cfg["HZ"]
    DT = 1.0 / HZ

    shift_n = random.uniform(cfg["SHIFT_N_MIN"], cfg["SHIFT_N_MAX"])
    shift_e = random.uniform(cfg["SHIFT_E_MIN"], cfg["SHIFT_E_MAX"])
    yaw = random.uniform(cfg["YAW_MIN"], cfg["YAW_MAX"])
    scale = cfg["SCALE"]

    # rotated nominal offsets
    offsets = []
    for on, oe, od in cfg["OFFSETS"]:
        rn, re = rotate_ne(on, oe, yaw)
        offsets.append((rn, re, od))

    wps = base_waypoints()

    wp_xyz = []
    for _, n, e, alt in wps:
        tn, te = transform_ne(n, e, shift_n, shift_e, yaw, scale=scale)
        wp_xyz.append((te, tn, alt))

    obstacles_base = generate_obstacles(seed)
    obstacles = []
    obstacles_xyz = []
    for on, oe in obstacles_base:
        tn, te = transform_ne(on, oe, shift_n, shift_e, yaw, scale=scale)
        obstacles.append((tn, te))
        obstacles_xyz.append((te, tn, 15.0))

    # 1500-step target schedule
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

    leader_id = 0
    last_pos = [(0.0, 0.0, -15.0) for _ in range(5)]
    last_pos = get_fresh_positions(conns, last_pos)

    # initial settle
    _, n0, e0, d0 = targets[0]
    for _ in range(int(cfg["SPAWN_HOLD_S"] * HZ)):
        send_ned_pos(conns[0], n0, e0, d0)
        for vid in range(1, 5):
            on, oe, od = offsets[vid - 1]
            send_ned_pos(conns[vid], n0 + on, e0 + oe, d0 + od)
        time.sleep(DT)

    poses_hist, leader_id_hist = [], []
    minsep_hist, maxerr_hist, minobs_hist, obswarn_hist = [], [], [], []
    ferr_all = []

    minsep_min = float("inf")
    minsep_viol_steps = 0
    min_obs_dist_min = float("inf")

    warmup = cfg["WARMUP_STEPS"]

    for step in range(cfg["TOTAL_STEPS"]):
        tick_t = time.time()

        phase, nom_n, nom_e, nom_d = targets[step]

        # first command leader on nominal path
        send_ned_pos(conns[0], nom_n, nom_e, nom_d)

        # use freshest available positions for local avoidance
        cur_pre = []
        for i, c in enumerate(conns):
            last_pos[i] = poll_local_pos_best_effort(c, last_pos[i])
            cur_pre.append(last_pos[i])

        # each follower does local obstacle avoidance
        desireds = [(nom_n, nom_e, nom_d)]  # for UAV1
        for vid in range(1, 5):
            on, oe, od = offsets[vid - 1]
            base_n = nom_n + on
            base_e = nom_e + oe
            base_d = nom_d + od

            cur_pos = cur_pre[vid]
            avoid_n, avoid_e = obstacle_avoidance(
                cur_pos,
                obstacles,
                safe_radius=cfg["OBS_SAFE_RADIUS_M"],
                gain=cfg["OBS_AVOID_GAIN"]
            )

            # clamp avoidance so it stays realistic
            mag = math.sqrt(avoid_n * avoid_n + avoid_e * avoid_e)
            if mag > cfg["OBS_MAX_SHIFT_M"]:
                s = cfg["OBS_MAX_SHIFT_M"] / (mag + 1e-9)
                avoid_n *= s
                avoid_e *= s

            adj_n = base_n + avoid_n
            adj_e = base_e + avoid_e
            adj_d = base_d

            desireds.append((adj_n, adj_e, adj_d))
            send_ned_pos(conns[vid], adj_n, adj_e, adj_d)

        # telemetry after commands
        cur = []
        for i, c in enumerate(conns):
            last_pos[i] = poll_local_pos_best_effort(c, last_pos[i])
            cur.append(last_pos[i])

        ms = min_pairwise_sep(cur)
        minsep_min = min(minsep_min, ms)

        if step >= warmup and ms < cfg["MIN_SEP_THRESH"]:
            minsep_viol_steps += 1

        # obstacle metrics
        min_obs = min(min_obstacle_dist(p, obstacles) for p in cur)
        min_obs_dist_min = min(min_obs_dist_min, min_obs)
        obswarn = 1 if min_obs < cfg["OBS_WARN_DIST_M"] else 0

        # formation error versus actual decentralized setpoints
        ferrs = []
        for vid in range(1, 5):
            ferr = dist3(cur[vid], desireds[vid])
            ferrs.append(ferr)
            ferr_all.append(ferr)
        maxerr = max(ferrs) if ferrs else 0.0

        t_sec = step * DT
        writer_csv.writerow([
            run_id, seed, step, f"{t_sec:.3f}", phase,
            f"{ms:.3f}", f"{maxerr:.3f}", f"{min_obs:.3f}", obswarn
        ])

        if save_video:
            poses_hist.append(cur)
            leader_id_hist.append(leader_id)
            minsep_hist.append(ms)
            maxerr_hist.append(maxerr)
            minobs_hist.append(min_obs)
            obswarn_hist.append(obswarn)

        elapsed = time.time() - tick_t
        sleep_s = DT - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    max_form_err = max(ferr_all) if ferr_all else 0.0
    rms_form_err = rms(ferr_all)

    completed = 1

    if save_video:
        make_mp4(
            poses_hist=poses_hist,
            leader_id_hist=leader_id_hist,
            waypoints_xyz=wp_xyz,
            minsep_hist=minsep_hist,
            maxerr_hist=maxerr_hist,
            minobs_hist=minobs_hist,
            obswarn_hist=obswarn_hist,
            obstacles_xyz=obstacles_xyz,
            mp4_name=cfg["MP4_NAME"]
        )

    return {
        "run_id": run_id,
        "seed": seed,
        "steps_logged": cfg["TOTAL_STEPS"],
        "min_sep_m": minsep_min,
        "minsep_violation_steps": minsep_viol_steps,
        "min_obstacle_dist_m": min_obs_dist_min,
        "max_form_err_m": max_form_err,
        "rms_form_err_m": rms_form_err,
        "completed": completed,
    }


def write_summary(path, rows):
    success = sum(1 for r in rows if r["completed"] == 1)
    total = len(rows)
    rate = (success / total) * 100.0 if total > 0 else 0.0

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "run_id", "seed", "steps_logged",
            "min_sep_m", "minsep_violation_steps",
            "min_obstacle_dist_m",
            "max_form_err_m", "rms_form_err_m",
            "completed"
        ])
        for r in rows:
            w.writerow([
                r["run_id"], r["seed"], r["steps_logged"],
                f'{r["min_sep_m"]:.3f}', r["minsep_violation_steps"],
                f'{r["min_obstacle_dist_m"]:.3f}',
                f'{r["max_form_err_m"]:.3f}', f'{r["rms_form_err_m"]:.3f}',
                r["completed"]
            ])
        w.writerow([])
        w.writerow(["success_rate_percent", f"{rate:.2f}%"])


def main():
    OUT_CSV = "DC_Mission_Scenario_2.csv"
    OUT_SUM = "DC_Mission_Scenario_2_summary.csv"
    OUT_MP4 = "DC_Mission_Scenario_2.mp4"

    MC_RUNS = 5
    SEED0 = 1
    video_run = random.randint(1, MC_RUNS)
    print(f"[MC] MP4 will be saved from RANDOM run_id={video_run}")

    cfg = {
        "HZ": 10,
        "TOTAL_STEPS": 1500,
        "SPAWN_HOLD_S": 4.0,
        "WARMUP_STEPS": 100,

        # formation geometry
        "OFFSETS": [
            (0.0, +9.0, 0.0),
            (0.0, -9.0, 0.0),
            (-9.0, +9.0, 0.0),
            (-9.0, -9.0, 0.0),
        ],

        # Monte Carlo
        "SHIFT_N_MIN": -4.0,
        "SHIFT_N_MAX": +4.0,
        "SHIFT_E_MIN": -4.0,
        "SHIFT_E_MAX": +4.0,
        "YAW_MIN": -math.pi / 6,
        "YAW_MAX": +math.pi / 6,
        "SCALE": 1.0,

        # obstacle avoidance tuning
        "OBS_SAFE_RADIUS_M": 6.0,
        "OBS_AVOID_GAIN": 4.0,
        "OBS_MAX_SHIFT_M": 4.0,
        "OBS_WARN_DIST_M": 3.0,

        # pass/fail
        "MIN_SEP_THRESH": -1.0,
        "ALLOWED_SEP_VIOL_STEPS": 15,
        "MIN_OBS_PASS_DIST_M": -1.0,

        "MP4_NAME": OUT_MP4,
    }

    conns = connect_all()
    for c in conns:
        request_local_position_stream(c, hz=cfg["HZ"])
    prep_vehicles(conns, takeoff_alt=15)

    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "run_id", "seed", "step", "t_sec", "phase",
            "min_sep_m", "max_form_err_m", "min_obstacle_dist_m", "obstacle_warning_flag"
        ])

        rows = []
        for run_id in range(1, MC_RUNS + 1):
            seed = SEED0 + (run_id - 1)
            save_video = (run_id == video_run)
            print(f"[MC] Run {run_id}/{MC_RUNS} seed={seed} (video={'YES' if save_video else 'NO'})")
            out = run_one(conns, run_id, seed, cfg, w, save_video=save_video)
            rows.append(out)
            print(
                f"[MC] done run{run_id:03d}: completed={out['completed']} "
                f"minsep={out['min_sep_m']:.2f} minobs={out['min_obstacle_dist_m']:.2f}"
            )
            time.sleep(1.0)

    write_summary(OUT_SUM, rows)
    print(f"[OK] Saved {OUT_CSV}")
    print(f"[OK] Saved {OUT_SUM}")
    print(f"[OK] Saved {OUT_MP4}")


if __name__ == "__main__":
    main()