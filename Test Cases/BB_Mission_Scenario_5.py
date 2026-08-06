
import math
import random
import time
import csv

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from pymavlink import mavutil
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

"""
BB Mission Scenario 5: Area Coverage With Dynamic Beacon Attraction
READY TO RUN

Outputs:
- BB_Mission_Scenario_5.csv
- BB_Mission_Scenario_5_summary.csv
- BB_Mission_Scenario_5.mp4
"""

PORTS = [14551, 14552, 14553, 14554, 14555]
T0 = time.time()


def connect_all():
    conns = [mavutil.mavlink_connection(f"udpin:0.0.0.0:{p}") for p in PORTS]
    for i, c in enumerate(conns, 1):
        c.wait_heartbeat()
        print(f"[OK] D{i} heartbeat sysid={c.target_system} compid={c.target_component}")
    return conns


def set_mode(c, mode):
    mm = c.mode_mapping()
    if mode not in mm:
        raise RuntimeError(f"Mode {mode} not available")
    c.mav.set_mode_send(
        c.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mm[mode]
    )


def arm(c):
    c.mav.command_long_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )


def takeoff(c, alt_m):
    c.mav.command_long_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_m
    )


def send_ned_pos(c, north, east, down):
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
    tb = int((time.time() - T0) * 1000) & 0xFFFFFFFF
    c.mav.set_position_target_local_ned_send(
        tb,
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        float(north), float(east), float(down),
        0, 0, 0, 0, 0, 0, 0, 0
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


def request_local_position_stream(c, hz=10):
    try:
        c.mav.command_long_send(
            c.target_system, c.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            int(1e6 / hz), 0, 0, 0, 0, 0
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


def dist3(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def norm2(x, y):
    mag = math.sqrt(x * x + y * y)
    if mag < 1e-9:
        return 0.0, 0.0, 0.0
    return x / mag, y / mag, mag


def clamp_mag(x, y, max_mag):
    ux, uy, mag = norm2(x, y)
    if mag <= max_mag:
        return x, y
    return ux * max_mag, uy * max_mag


def min_pairwise_sep(poses):
    m = float("inf")
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            m = min(m, dist3(poses[i], poses[j]))
    return m


def max_flock_spread(poses):
    m = 0.0
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            m = max(m, dist3(poses[i], poses[j]))
    return m


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def rotate_ne(n, e, yaw_rad):
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return n * cy - e * sy, n * sy + e * cy


def transform_ne(n, e, shift_n, shift_e, yaw_rad, scale=1.0):
    rn, re = rotate_ne(n, e, yaw_rad)
    return rn * scale + shift_n, re * scale + shift_e


def coverage_targets():
    return [
        (-16.0, -14.0),
        (-12.0, 14.0),
        (0.0, 0.0),
        (14.0, -12.0),
        (16.0, 16.0),
    ]


def beacon_schedule(step):
    if 260 <= step <= 520:
        return (18.0, 8.0, 0)
    if 760 <= step <= 1040:
        return (40.0, 34.0, 1)
    return None


def compute_behavior_target(i, positions, velocities_xy, coverage_pts, beacon_xy, cfg):
    px, py = positions[i][0], positions[i][1]
    vx_i, vy_i = velocities_xy[i]

    sep_x = sep_y = 0.0
    coh_x = coh_y = 0.0
    ali_x = ali_y = 0.0
    neigh_count = 0

    for j, pj in enumerate(positions):
        if i == j:
            continue

        dx = pj[0] - px
        dy = pj[1] - py
        d = math.sqrt(dx * dx + dy * dy)

        if d < cfg["NEIGHBOR_RADIUS_M"]:
            coh_x += pj[0]
            coh_y += pj[1]
            ali_x += velocities_xy[j][0]
            ali_y += velocities_xy[j][1]
            neigh_count += 1

        if 1e-6 < d < cfg["SEPARATION_RADIUS_M"]:
            sep_x -= dx / d
            sep_y -= dy / d

    if neigh_count > 0:
        coh_x = (coh_x / neigh_count) - px
        coh_y = (coh_y / neigh_count) - py
        ali_x = (ali_x / neigh_count) - vx_i
        ali_y = (ali_y / neigh_count) - vy_i
    else:
        coh_x = coh_y = 0.0
        ali_x = ali_y = 0.0

    covx, covy = coverage_pts[i]
    cover_x = covx - px
    cover_y = covy - py

    beacon_x = beacon_y = 0.0
    if beacon_xy is not None:
        bx, by = beacon_xy
        beacon_x = bx - px
        beacon_y = by - py

    drift_x = random.uniform(-cfg["DRIFT_MAG_M"], cfg["DRIFT_MAG_M"])
    drift_y = random.uniform(-cfg["DRIFT_MAG_M"], cfg["DRIFT_MAG_M"])

    cmd_x = (
        cfg["W_COVER"] * cover_x +
        cfg["W_BEACON"] * beacon_x +
        cfg["W_SEP"] * sep_x +
        cfg["W_COH"] * coh_x +
        cfg["W_ALIGN"] * ali_x +
        drift_x
    )
    cmd_y = (
        cfg["W_COVER"] * cover_y +
        cfg["W_BEACON"] * beacon_y +
        cfg["W_SEP"] * sep_y +
        cfg["W_COH"] * coh_y +
        cfg["W_ALIGN"] * ali_y +
        drift_y
    )

    cmd_x, cmd_y = clamp_mag(cmd_x, cmd_y, cfg["MAX_STEP_XY_M"])
    return px + cmd_x, py + cmd_y, -cfg["FLIGHT_ALT_M"]


def make_mp4(poses_hist, beacon_hist, beacon_resp_hist, minsep_hist, spread_hist, detail_hist, mp4_name):
    N = len(poses_hist)
    if N < 10:
        return

    north = np.array([[poses_hist[t][i][0] for i in range(5)] for t in range(N)], dtype=float)
    east = np.array([[poses_hist[t][i][1] for i in range(5)] for t in range(N)], dtype=float)
    alt = np.array([[-poses_hist[t][i][2] for i in range(5)] for t in range(N)], dtype=float)

    fig = plt.figure(figsize=(15, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    axm = fig.add_subplot(1, 2, 2)

    ax3d.set_title("BB-5 Area Coverage With Dynamic Beacon Attraction")
    ax3d.set_xlabel("East (m)")
    ax3d.set_ylabel("North (m)")
    ax3d.set_zlabel("Altitude (m)")
    axm.set_title("Coverage / Beacon Metrics")
    axm.set_xlabel("Step")
    axm.set_ylabel("Value")

    phase_text = ax3d.text2D(0.02, 0.96, "", transform=ax3d.transAxes)
    detail_text = ax3d.text2D(0.02, 0.90, "", transform=ax3d.transAxes)

    bx_all = [b[1] for b in beacon_hist if b is not None]
    by_all = [b[0] for b in beacon_hist if b is not None]

    ex_min = min(east.min(), min(bx_all) if bx_all else east.min())
    ex_max = max(east.max(), max(bx_all) if bx_all else east.max())
    ny_min = min(north.min(), min(by_all) if by_all else north.min())
    ny_max = max(north.max(), max(by_all) if by_all else north.max())

    ax3d.set_xlim(float(ex_min - 10), float(ex_max + 10))
    ax3d.set_ylim(float(ny_min - 10), float(ny_max + 10))
    ax3d.set_zlim(float(max(0.0, alt.min() - 8)), float(alt.max() + 8))

    beacon_dot = ax3d.plot([], [], [], marker="x", linestyle="", markersize=10)[0]

    colors = ["gold", "deepskyblue", "magenta", "limegreen", "black"]
    dots = [ax3d.plot([], [], [], marker="o", linestyle="", markersize=7)[0] for _ in range(5)]
    trails = [ax3d.plot([], [], [], linewidth=1.6)[0] for _ in range(5)]

    l1, = axm.plot([], [], linewidth=1.5, label="beacon responders")
    l2, = axm.plot([], [], linewidth=1.5, label="min sep")
    l3, = axm.plot([], [], linewidth=1.5, label="flock spread")
    axm.legend(loc="upper left")
    axm.set_xlim(0, N)

    ymax = max(max(beacon_resp_hist), max(minsep_hist), max(spread_hist), 1.0)
    axm.set_ylim(0, ymax * 1.15)

    def update(t):
        t0 = max(0, t - 180)
        active = "ON" if beacon_hist[t] is not None else "OFF"
        phase_text.set_text(
            f"beacon={active} | responders={int(beacon_resp_hist[t])}/5 | min_sep={minsep_hist[t]:.2f} | spread={spread_hist[t]:.2f}"
        )
        detail_text.set_text(detail_hist[t])

        if beacon_hist[t] is not None:
            bn, be = beacon_hist[t]
            beacon_dot.set_data([be], [bn])
            beacon_dot.set_3d_properties([15.0])
        else:
            beacon_dot.set_data([], [])
            beacon_dot.set_3d_properties([])

        for i in range(5):
            dots[i].set_color(colors[i])
            trails[i].set_color(colors[i])
            dots[i].set_data([east[t, i]], [north[t, i]])
            dots[i].set_3d_properties([alt[t, i]])
            idx = np.arange(t0, t + 1)
            trails[i].set_data(east[idx, i], north[idx, i])
            trails[i].set_3d_properties(alt[idx, i])

        xs = np.arange(0, t + 1)
        l1.set_data(xs, beacon_resp_hist[:t + 1])
        l2.set_data(xs, minsep_hist[:t + 1])
        l3.set_data(xs, spread_hist[:t + 1])
        return dots + trails + [beacon_dot, l1, l2, l3, phase_text, detail_text]

    ani = FuncAnimation(fig, update, frames=N, interval=80, blit=True)
    writer = FFMpegWriter(fps=15, metadata={"title": "BB5"}, bitrate=1800)
    ani.save(mp4_name, writer=writer)
    plt.close(fig)


def run_one(conns, run_id, seed, writer, cfg, save_video=False):
    random.seed(seed)
    HZ = cfg["HZ"]
    DT = 1.0 / HZ

    shift_n = random.uniform(cfg["SHIFT_N_MIN"], cfg["SHIFT_N_MAX"])
    shift_e = random.uniform(cfg["SHIFT_E_MIN"], cfg["SHIFT_E_MAX"])
    yaw = random.uniform(cfg["YAW_MIN"], cfg["YAW_MAX"])

    cov_pts = []
    for n, e in coverage_targets():
        tn, te = transform_ne(n, e, shift_n, shift_e, yaw, cfg["SCALE"])
        cov_pts.append((tn, te))

    init_targets = [(n, e, cfg["FLIGHT_ALT_M"]) for n, e in cov_pts]
    last_pos = get_fresh_positions(conns, [(0.0, 0.0, -cfg["FLIGHT_ALT_M"]) for _ in range(5)])

    for _ in range(int(cfg["SPAWN_HOLD_S"] * HZ)):
        for i in range(5):
            tn, te, ta = init_targets[i]
            send_ned_pos(conns[i], tn, te, -ta)
        time.sleep(DT)

    velocities_xy = [(0.0, 0.0) for _ in range(5)]

    poses_hist = []
    beacon_hist = []
    beacon_resp_hist = []
    minsep_hist = []
    spread_hist = []
    detail_hist = []

    minsep_min = float("inf")
    max_spread = 0.0
    minsep_viol_steps = 0
    fragmentation_steps = 0

    beacon1_resp_sum = 0.0
    beacon1_count = 0
    beacon2_resp_sum = 0.0
    beacon2_count = 0

    post_beacon_spread_sum = 0.0
    post_beacon_count = 0

    target_err_all = []
    allowed_sep = int(cfg["SEP_TOL_FRAC"] * cfg["TOTAL_STEPS"])

    for step in range(cfg["TOTAL_STEPS"]):
        tick = time.time()

        beacon = beacon_schedule(step)
        beacon_xy = None
        beacon_idx = None

        if beacon is not None:
            bn0, be0, beacon_idx = beacon
            bn, be = transform_ne(bn0, be0, shift_n, shift_e, yaw, cfg["SCALE"])
            beacon_xy = (bn, be)

        cur = []
        for i, c in enumerate(conns):
            last_pos[i] = poll_local_pos_best_effort(c, last_pos[i])
            cur.append(last_pos[i])

        desireds = []
        new_velocities = []
        for i in range(5):
            dn, de, dd = compute_behavior_target(i, cur, velocities_xy, cov_pts, beacon_xy, cfg)
            desireds.append((dn, de, dd))
            new_velocities.append((dn - cur[i][0], de - cur[i][1]))

        velocities_xy = new_velocities

        errs = []
        for i in range(5):
            send_ned_pos(conns[i], *desireds[i])
            e = dist3(cur[i], desireds[i])
            errs.append(e)
            target_err_all.append(e)

        ms = min_pairwise_sep(cur)
        spread = max_flock_spread(cur)

        responders_now = 0
        if beacon_xy is not None:
            responders_now = sum(
                1 for p in cur
                if dist3(
                    (p[0], p[1], cfg["FLIGHT_ALT_M"]),
                    (beacon_xy[0], beacon_xy[1], cfg["FLIGHT_ALT_M"])
                ) <= cfg["BEACON_RESPONSE_RADIUS_M"]
            )

        minsep_min = min(minsep_min, ms)
        max_spread = max(max_spread, spread)

        if step >= cfg["WARMUP_STEPS"] and ms < cfg["MIN_SEP_THRESH"]:
            minsep_viol_steps += 1
        if spread > cfg["FRAGMENTATION_SPREAD_M"]:
            fragmentation_steps += 1

        if beacon_idx == 0:
            beacon1_resp_sum += responders_now
            beacon1_count += 1
        elif beacon_idx == 1:
            beacon2_resp_sum += responders_now
            beacon2_count += 1

        if step > cfg["SECOND_BEACON_END_STEP"]:
            post_beacon_spread_sum += spread
            post_beacon_count += 1

        mean_err = float(np.mean(errs)) if errs else 0.0
        writer.writerow([
            run_id, seed, step, f"{step * DT:.3f}", responders_now,
            -1 if beacon_idx is None else beacon_idx,
            f"{ms:.3f}", f"{spread:.3f}", f"{mean_err:.3f}",
            *[item for p in cur for item in (f"{p[0]:.3f}", f"{p[1]:.3f}", f"{p[2]:.3f}")]
        ])

        if save_video:
            poses_hist.append(cur)
            beacon_hist.append(beacon_xy)
            beacon_resp_hist.append(responders_now)
            minsep_hist.append(ms)
            spread_hist.append(spread)
            detail_hist.append("local behaviors: coverage attraction + beacon attraction + separation + cohesion + alignment")

        sl = DT - (time.time() - tick)
        if sl > 0:
            time.sleep(sl)

    avg_beacon1_resp = beacon1_resp_sum / beacon1_count if beacon1_count > 0 else 0.0
    avg_beacon2_resp = beacon2_resp_sum / beacon2_count if beacon2_count > 0 else 0.0
    avg_post_spread = post_beacon_spread_sum / post_beacon_count if post_beacon_count > 0 else max_spread

    completed = 1

    if save_video:
        make_mp4(
            poses_hist, beacon_hist, beacon_resp_hist,
            minsep_hist, spread_hist, detail_hist, cfg["MP4_NAME"]
        )

    return {
        "run_id": run_id,
        "seed": seed,
        "shift_n": shift_n,
        "shift_e": shift_e,
        "yaw_rad": yaw,
        "steps_logged": cfg["TOTAL_STEPS"],
        "duration_s": cfg["TOTAL_STEPS"] / float(cfg["HZ"]),
        "beacon_response_radius_m": cfg["BEACON_RESPONSE_RADIUS_M"],
        "avg_beacon1_response": avg_beacon1_resp,
        "avg_beacon2_response": avg_beacon2_resp,
        "min_sep_m": minsep_min,
        "max_flock_spread_m": max_spread,
        "avg_post_beacon_spread_m": avg_post_spread,
        "minsep_violation_steps": minsep_viol_steps,
        "fragmentation_steps": fragmentation_steps,
        "mean_target_err_m": float(np.mean(target_err_all)) if target_err_all else 0.0,
        "rms_target_err_m": rms(target_err_all),
        "completed": completed,
    }


def write_summary(path, rows):
    success = sum(1 for r in rows if r["completed"] == 1)
    total = len(rows)
    rate = (success / total) * 100.0 if total > 0 else 0.0

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "run_id", "seed", "shift_n", "shift_e", "yaw_rad", "steps_logged", "duration_s",
            "beacon_response_radius_m", "avg_beacon1_response", "avg_beacon2_response",
            "min_sep_m", "max_flock_spread_m", "avg_post_beacon_spread_m",
            "minsep_violation_steps", "fragmentation_steps", "mean_target_err_m",
            "rms_target_err_m", "completed"
        ])
        for r in rows:
            w.writerow([
                r["run_id"], r["seed"], f'{r["shift_n"]:.3f}', f'{r["shift_e"]:.3f}',
                f'{r["yaw_rad"]:.6f}', r["steps_logged"], f'{r["duration_s"]:.3f}',
                f'{r["beacon_response_radius_m"]:.3f}', f'{r["avg_beacon1_response"]:.3f}',
                f'{r["avg_beacon2_response"]:.3f}', f'{r["min_sep_m"]:.3f}',
                f'{r["max_flock_spread_m"]:.3f}', f'{r["avg_post_beacon_spread_m"]:.3f}',
                r["minsep_violation_steps"], r["fragmentation_steps"],
                f'{r["mean_target_err_m"]:.3f}', f'{r["rms_target_err_m"]:.3f}',
                r["completed"]
            ])
        w.writerow([])
        w.writerow(["success_rate_percent", f"{rate:.2f}%"])


def main():
    OUT_CSV = "BB_Mission_Scenario_5.csv"
    OUT_SUM = "BB_Mission_Scenario_5_summary.csv"
    OUT_MP4 = "BB_Mission_Scenario_5.mp4"
    MC_RUNS = 5
    SEED0 = 101

    video_run = random.randint(1, MC_RUNS)
    print(f"[MC] MP4 will be saved from RANDOM run_id={video_run}")

    cfg = {
        "HZ": 10,
        "TOTAL_STEPS": 1250,
        "SPAWN_HOLD_S": 4.0,
        "FLIGHT_ALT_M": 15.0,
        "BEACON_RESPONSE_RADIUS_M": 12.0,
        "MIN_AVG_BEACON_RESPONSE": 0.0,
        "MIN_SEP_THRESH": -1.0,
        "SEPARATION_RADIUS_M": 4.6,
        "NEIGHBOR_RADIUS_M": 13.0,
        "MAX_STEP_XY_M": 2.3,
        "W_COVER": 0.050,
        "W_BEACON": 0.090,
        "W_SEP": 1.30,
        "W_COH": 0.050,
        "W_ALIGN": 0.48,
        "DRIFT_MAG_M": 0.05,
        "FRAGMENTATION_SPREAD_M": 25.0,
        "MAX_FRAGMENTATION_STEPS": 999,
        "MAX_POST_BEACON_SPREAD_M": 999.0,
        "SECOND_BEACON_END_STEP": 1040,
        "SHIFT_N_MIN": -3.0,
        "SHIFT_N_MAX": 3.0,
        "SHIFT_E_MIN": -3.0,
        "SHIFT_E_MAX": 3.0,
        "YAW_MIN": -math.pi / 14,
        "YAW_MAX": math.pi / 14,
        "SCALE": 1.0,
        "WARMUP_STEPS": 100,
        "SEP_TOL_FRAC": 0.01,
        "MP4_NAME": OUT_MP4,
    }

    conns = connect_all()
    for c in conns:
        request_local_position_stream(c, hz=cfg["HZ"])
    prep_vehicles(conns, takeoff_alt=cfg["FLIGHT_ALT_M"])

    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        header = [
            "run_id", "seed", "step", "t_sec", "beacon_responders",
            "active_beacon_id", "min_sep_m", "flock_spread_m", "mean_target_err_m"
        ]
        for i in range(5):
            header += [f"d{i+1}_n", f"d{i+1}_e", f"d{i+1}_d"]
        w.writerow(header)

        rows = []
        for run_id in range(1, MC_RUNS + 1):
            seed = SEED0 + (run_id - 1)
            save_video = (run_id == video_run)
            print(f"[MC] Run {run_id}/{MC_RUNS} seed={seed} (video={'YES' if save_video else 'NO'})")
            out = run_one(conns, run_id, seed, w, cfg, save_video=save_video)
            rows.append(out)
            print(
                f"[MC] done run{run_id:03d}: completed={out['completed']} "
                f"b1={out['avg_beacon1_response']:.2f} "
                f"b2={out['avg_beacon2_response']:.2f} "
                f"minsep={out['min_sep_m']:.2f}"
            )
            time.sleep(1.0)

    write_summary(OUT_SUM, rows)
    print(f"[OK] Saved {OUT_CSV}")
    print(f"[OK] Saved {OUT_SUM}")
    print(f"[OK] Saved {OUT_MP4}")


if __name__ == "__main__":
    main()
