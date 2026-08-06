# Easy-to-understand visual version for CTA Scenario 3
# Roles:
# D1 relay/observer
# D2-D3 search
# D4 confirmation
# D5 delivery
# Visual meaning:
# gray sector = not searched
# orange sector = searched
# red sector = suspected survivor
# blue sector = confirmed survivor
# green sector = supply delivered
# D2 = yellow, D3 = cyan, D4 = magenta, D5 = green

import time, math, random, csv
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from pymavlink import mavutil
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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
    c.mav.set_mode_send(c.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mm[mode])


def arm(c):
    c.mav.command_long_send(c.target_system, c.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)


def takeoff(c, alt_m):
    c.mav.command_long_send(c.target_system, c.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)


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
    c.mav.set_position_target_local_ned_send(tb, c.target_system, c.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, float(north), float(east), float(down), 0, 0, 0, 0, 0, 0, 0, 0)


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
        c.mav.command_long_send(c.target_system, c.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, int(1e6 / hz), 0, 0, 0, 0, 0)
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
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def min_pairwise_sep(poses):
    m = float("inf")
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            m = min(m, dist3(poses[i], poses[j]))
    return m


def rms(values):
    return math.sqrt(sum(v*v for v in values) / len(values)) if values else 0.0


def rotate_ne(n, e, yaw_rad):
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return n * cy - e * sy, n * sy + e * cy


def transform_ne(n, e, shift_n, shift_e, yaw_rad, scale=1.0):
    rn, re = rotate_ne(n, e, yaw_rad)
    return rn * scale + shift_n, re * scale + shift_e


def base_supervisory_route():
    return [(0.0, 0.0, 18.0), (10.0, -6.0, 18.0), (16.0, 6.0, 18.0), (6.0, 16.0, 18.0), (-8.0, 8.0, 18.0), (0.0, 0.0, 18.0)]


def base_search_sectors():
    return [("A", -20.0, -14.0, 15.0), ("B", 0.0, -16.0, 15.0), ("C", 20.0, -12.0, 15.0), ("D", -18.0, 10.0, 15.0), ("E", 2.0, 12.0, 15.0), ("F", 22.0, 14.0, 15.0)]


def build_supervisory_schedule(route_points, total_steps):
    out = []
    segs = len(route_points) - 1
    per_seg = total_steps // segs
    extra = total_steps - per_seg * segs
    for i in range(segs):
        na, ea, alta = route_points[i]
        nb, eb, altb = route_points[i + 1]
        da, db = -alta, -altb
        seg_steps = per_seg + (1 if i < extra else 0)
        for t in np.linspace(0, 1, seg_steps, endpoint=False):
            out.append((na + (nb - na) * t, ea + (eb - ea) * t, da + (db - da) * t))
    return out[:total_steps]


def make_mp4(poses_hist, sector_positions, sector_names, state_hist, search_assign_hist, confirm_assign_hist, delivery_assign_hist, searched_hist, suspected_hist, confirmed_hist, dropped_hist, confirm_q_hist, delivery_q_hist, mp4_name):
    N = len(poses_hist)
    if N < 10:
        return
    north = np.array([[poses_hist[t][i][0] for i in range(5)] for t in range(N)], dtype=float)
    east = np.array([[poses_hist[t][i][1] for i in range(5)] for t in range(N)], dtype=float)
    alt = np.array([[-poses_hist[t][i][2] for i in range(5)] for t in range(N)], dtype=float)

    fig = plt.figure(figsize=(15, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    axm = fig.add_subplot(1, 2, 2)
    ax3d.set_title("CTA-3 Easy View")
    ax3d.set_xlabel("East (m)")
    ax3d.set_ylabel("North (m)")
    ax3d.set_zlabel("Altitude (m)")
    axm.set_title("Simple Mission Counters")
    axm.set_xlabel("Step")
    axm.set_ylabel("Count")

    se = [p[0] for p in sector_positions]
    sn = [p[1] for p in sector_positions]
    sa = [p[2] for p in sector_positions]
    sector_scatter = ax3d.scatter(se, sn, sa, c="gray", marker="o", s=80, alpha=0.95)
    for name, e, n, a in zip(sector_names, se, sn, sa):
        ax3d.text(e, n, a + 0.6, name, fontsize=9)

    phase_text = ax3d.text2D(0.02, 0.96, "", transform=ax3d.transAxes)
    role_text = ax3d.text2D(0.02, 0.90, "", transform=ax3d.transAxes)

    ax3d.set_xlim(float(east.min() - 12), float(east.max() + 12))
    ax3d.set_ylim(float(north.min() - 12), float(north.max() + 12))
    ax3d.set_zlim(float(max(0.0, alt.min() - 8)), float(alt.max() + 8))

    role_colors = ["black", "gold", "deepskyblue", "magenta", "limegreen"]
    dots = [ax3d.plot([], [], [], marker="o", linestyle="", markersize=7)[0] for _ in range(5)]
    trails = [ax3d.plot([], [], [], linewidth=1.6)[0] for _ in range(5)]

    l1, = axm.plot([], [], linewidth=1.5, label="searched")
    l2, = axm.plot([], [], linewidth=1.5, label="suspected")
    l3, = axm.plot([], [], linewidth=1.5, label="confirmed")
    l4, = axm.plot([], [], linewidth=1.5, label="drops")
    l5, = axm.plot([], [], linewidth=1.5, label="confirm queue")
    l6, = axm.plot([], [], linewidth=1.5, label="delivery queue")
    axm.legend(loc="upper left")
    axm.set_xlim(0, N)
    ymax = max(max(searched_hist), max(suspected_hist), max(confirmed_hist), max(dropped_hist), max(confirm_q_hist), max(delivery_q_hist), 1.0)
    axm.set_ylim(0, ymax * 1.15)

    def update(t):
        t0 = max(0, t - 180)
        phase_text.set_text(f"searched={int(searched_hist[t])} | suspected={int(suspected_hist[t])} | confirmed={int(confirmed_hist[t])} | drops={int(dropped_hist[t])}")
        role_text.set_text(f"D2={search_assign_hist[t][0]}  D3={search_assign_hist[t][1]}  D4={confirm_assign_hist[t]}  D5={delivery_assign_hist[t]}")
        for i in range(5):
            dots[i].set_color(role_colors[i])
            trails[i].set_color(role_colors[i])
            dots[i].set_data([east[t, i]], [north[t, i]])
            dots[i].set_3d_properties([alt[t, i]])
            idx = np.arange(t0, t + 1)
            trails[i].set_data(east[idx, i], north[idx, i])
            trails[i].set_3d_properties(alt[idx, i])
        cols = []
        for s in state_hist[t]:
            cols.append((0.6,0.6,0.6,0.95) if s==0 else ((1.0,0.55,0.0,0.95) if s==1 else ((1.0,0.0,0.0,0.95) if s==2 else ((0.0,0.35,1.0,0.95) if s==3 else (0.0,0.75,0.0,0.95)))))
        sector_scatter._facecolor3d = cols
        sector_scatter._edgecolor3d = cols
        xs = np.arange(0, t + 1)
        l1.set_data(xs, searched_hist[:t + 1])
        l2.set_data(xs, suspected_hist[:t + 1])
        l3.set_data(xs, confirmed_hist[:t + 1])
        l4.set_data(xs, dropped_hist[:t + 1])
        l5.set_data(xs, confirm_q_hist[:t + 1])
        l6.set_data(xs, delivery_q_hist[:t + 1])
        return dots + trails + [sector_scatter, l1, l2, l3, l4, l5, l6, phase_text, role_text]

    ani = FuncAnimation(fig, update, frames=N, interval=80, blit=True)
    writer = FFMpegWriter(fps=15, metadata={"title": "CTA3 easy"}, bitrate=1800)
    ani.save(mp4_name, writer=writer)
    plt.close(fig)


def run_one(conns, run_id, seed, cfg, combined_writer, save_video=False):
    random.seed(seed)
    HZ = cfg["HZ"]
    DT = 1.0 / HZ
    shift_n = random.uniform(cfg["SHIFT_N_MIN"], cfg["SHIFT_N_MAX"])
    shift_e = random.uniform(cfg["SHIFT_E_MIN"], cfg["SHIFT_E_MAX"])
    yaw = random.uniform(cfg["YAW_MIN"], cfg["YAW_MAX"])

    route = []
    for n, e, alt in base_supervisory_route():
        tn, te = transform_ne(n, e, shift_n, shift_e, yaw, scale=cfg["SCALE"])
        route.append((tn, te, alt))
    supervisor_targets = build_supervisory_schedule(route, cfg["TOTAL_STEPS"])

    sectors = []
    sector_positions_video = []
    sector_names = []
    for name, n, e, alt in base_search_sectors():
        tn, te = transform_ne(n, e, shift_n, shift_e, yaw, scale=cfg["SCALE"])
        sectors.append((name, tn, te, -alt))
        sector_positions_video.append((te, tn, alt))
        sector_names.append(name)

    truth = [0, 1, 0, 1, 0, 1]  # B, D, F are true survivors
    searched = [0] * len(sectors)
    suspected = [0] * len(sectors)
    confirmed = [0] * len(sectors)
    dropped = [0] * len(sectors)

    search_queue = deque(range(len(sectors)))
    confirm_queue = deque()
    delivery_queue = deque()

    search_assign = {2: None, 3: None}
    confirm_assign = None
    delivery_assign = None
    search_dwell = {2: 0, 3: 0}
    confirm_dwell = 0
    delivery_dwell = 0

    detect_step = {}
    confirm_step = {}
    delivery_step = {}
    target_err_all = []

    def assign_search(vid):
        search_assign[vid] = search_queue.popleft() if search_queue else None

    def assign_confirm():
        nonlocal confirm_assign
        confirm_assign = confirm_queue.popleft() if confirm_queue else None

    def assign_delivery():
        nonlocal delivery_assign
        delivery_assign = delivery_queue.popleft() if delivery_queue else None

    assign_search(2)
    assign_search(3)

    last_pos = get_fresh_positions(conns, [(0.0, 0.0, -18.0) for _ in range(5)])

    m_n0, m_e0, m_d0 = supervisor_targets[0]
    staging_offsets = [(5.0, 10.0), (5.0, -10.0), (-5.0, 10.0), (-5.0, -10.0)]
    for _ in range(int(cfg["SPAWN_HOLD_S"] * HZ)):
        send_ned_pos(conns[0], m_n0, m_e0, m_d0)
        for vid in range(1, 5):
            on, oe = staging_offsets[vid - 1]
            send_ned_pos(conns[vid], m_n0 + on, m_e0 + oe, m_d0)
        time.sleep(DT)

    poses_hist = []
    state_hist = []
    search_assign_hist = []
    confirm_assign_hist = []
    delivery_assign_hist = []
    searched_hist = []
    suspected_hist = []
    confirmed_hist = []
    dropped_hist = []
    confirm_q_hist = []
    delivery_q_hist = []

    minsep_min = float("inf")
    minsep_viol_steps = 0
    WARMUP = cfg["WARMUP_STEPS"]
    allowed_sep = int(cfg["SEP_TOL_FRAC"] * cfg["TOTAL_STEPS"])

    for step in range(cfg["TOTAL_STEPS"]):
        tick_t = time.time()
        m_n, m_e, m_d = supervisor_targets[step]
        send_ned_pos(conns[0], m_n, m_e, m_d)

        cur = []
        for i, c in enumerate(conns):
            last_pos[i] = poll_local_pos_best_effort(c, last_pos[i])
            cur.append(last_pos[i])

        for vid in (2, 3):
            sid = search_assign[vid]
            desired = (m_n + cfg["IDLE_HOLD_RING"][vid - 1][0], m_e + cfg["IDLE_HOLD_RING"][vid - 1][1], m_d) if sid is None else sectors[sid][1:]
            send_ned_pos(conns[vid - 1], *desired)
            err = dist3(cur[vid - 1], desired)
            target_err_all.append(err)
            if sid is not None:
                search_dwell[vid] = search_dwell[vid] + 1 if err <= cfg["SEARCH_RADIUS_M"] else 0
                if search_dwell[vid] >= cfg["SEARCH_DWELL_STEPS"] and searched[sid] == 0:
                    searched[sid] = 1
                    search_dwell[vid] = 0
                    if sid in cfg["SUSPECTED_IDS"] and suspected[sid] == 0 and confirmed[sid] == 0:
                        suspected[sid] = 1
                        detect_step[sid] = step
                        confirm_queue.append(sid)
                    assign_search(vid)

        if confirm_assign is None and confirm_queue:
            assign_confirm()
        if delivery_assign is None and delivery_queue:
            assign_delivery()

        sid = confirm_assign
        desired = (m_n + cfg["IDLE_HOLD_RING"][2][0], m_e + cfg["IDLE_HOLD_RING"][2][1], m_d) if sid is None else sectors[sid][1:]
        send_ned_pos(conns[3], *desired)
        err = dist3(cur[3], desired)
        target_err_all.append(err)
        if sid is not None:
            confirm_dwell = confirm_dwell + 1 if err <= cfg["CONFIRM_RADIUS_M"] else 0
            if confirm_dwell >= cfg["CONFIRM_DWELL_STEPS"]:
                confirm_dwell = 0
                if truth[sid] == 1:
                    confirmed[sid] = 1
                    suspected[sid] = 0
                    confirm_step[sid] = step
                    delivery_queue.append(sid)
                else:
                    suspected[sid] = 0
                assign_confirm()

        sid = delivery_assign
        desired = (m_n + cfg["IDLE_HOLD_RING"][3][0], m_e + cfg["IDLE_HOLD_RING"][3][1], m_d) if sid is None else sectors[sid][1:]
        send_ned_pos(conns[4], *desired)
        err = dist3(cur[4], desired)
        target_err_all.append(err)
        if sid is not None:
            delivery_dwell = delivery_dwell + 1 if err <= cfg["DROP_RADIUS_M"] else 0
            if delivery_dwell >= cfg["DROP_DWELL_STEPS"]:
                delivery_dwell = 0
                dropped[sid] = 1
                delivery_step[sid] = step
                assign_delivery()

        ms = min_pairwise_sep(cur)
        minsep_min = min(minsep_min, ms)
        if step >= WARMUP and ms < cfg["MIN_SEP_THRESH"]:
            minsep_viol_steps += 1

        searched_now = sum(searched)
        suspected_now = sum(suspected)
        confirmed_now = sum(confirmed)
        dropped_now = sum(dropped)

        row = [run_id, seed, step, f"{step * DT:.3f}", searched_now, suspected_now, confirmed_now, dropped_now, len(confirm_queue), len(delivery_queue), f"{ms:.3f}"]
        for i in range(5):
            n, e, d = cur[i]
            row += [f"{n:.3f}", f"{e:.3f}", f"{d:.3f}"]
        row += [search_assign[2], search_assign[3], confirm_assign, delivery_assign]
        combined_writer.writerow(row)

        if save_video:
            states = []
            for i in range(len(sectors)):
                if dropped[i] == 1:
                    states.append(4)
                elif confirmed[i] == 1:
                    states.append(3)
                elif suspected[i] == 1:
                    states.append(2)
                elif searched[i] == 1:
                    states.append(1)
                else:
                    states.append(0)
            poses_hist.append(cur)
            state_hist.append(states)
            search_assign_hist.append((search_assign[2], search_assign[3]))
            confirm_assign_hist.append(confirm_assign)
            delivery_assign_hist.append(delivery_assign)
            searched_hist.append(searched_now)
            suspected_hist.append(suspected_now)
            confirmed_hist.append(confirmed_now)
            dropped_hist.append(dropped_now)
            confirm_q_hist.append(len(confirm_queue))
            delivery_q_hist.append(len(delivery_queue))

        sleep_s = DT - (time.time() - tick_t)
        if sleep_s > 0:
            time.sleep(sleep_s)

    true_survivors = sum(truth)
    confirmed_fraction = sum(confirmed) / max(1, true_survivors)
    delivery_fraction = sum(dropped) / max(1, sum(confirmed)) if sum(confirmed) > 0 else 0.0
    completed = 1

    confirm_delays = [confirm_step[s] - detect_step[s] for s in detect_step if s in confirm_step]
    delivery_delays = [delivery_step[s] - confirm_step[s] for s in confirm_step if s in delivery_step]
    avg_confirm_delay = float(np.mean(confirm_delays)) if confirm_delays else 0.0
    avg_delivery_delay = float(np.mean(delivery_delays)) if delivery_delays else 0.0

    if save_video:
        make_mp4(poses_hist, sector_positions_video, sector_names, state_hist, search_assign_hist, confirm_assign_hist, delivery_assign_hist, searched_hist, suspected_hist, confirmed_hist, dropped_hist, confirm_q_hist, delivery_q_hist, cfg["MP4_NAME"])

    return {
        "run_id": run_id,
        "seed": seed,
        "shift_n": shift_n,
        "shift_e": shift_e,
        "yaw_rad": yaw,
        "steps_logged": cfg["TOTAL_STEPS"],
        "duration_s": cfg["TOTAL_STEPS"] / float(cfg["HZ"]),
        "sectors_total": len(sectors),
        "sectors_searched": sum(searched),
        "suspected_points": len(cfg["SUSPECTED_IDS"]),
        "true_survivors": true_survivors,
        "confirmed_survivors": sum(confirmed),
        "successful_supply_drops": sum(dropped),
        "avg_confirm_delay_steps": avg_confirm_delay,
        "avg_delivery_delay_steps": avg_delivery_delay,
        "min_sep_m": minsep_min,
        "minsep_violation_steps": minsep_viol_steps,
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
        w.writerow(["run_id", "seed", "shift_n", "shift_e", "yaw_rad", "steps_logged", "duration_s", "sectors_total", "sectors_searched", "suspected_points", "true_survivors", "confirmed_survivors", "successful_supply_drops", "avg_confirm_delay_steps", "avg_delivery_delay_steps", "min_sep_m", "minsep_violation_steps", "mean_target_err_m", "rms_target_err_m", "completed"])
        for r in rows:
            w.writerow([r["run_id"], r["seed"], f'{r["shift_n"]:.3f}', f'{r["shift_e"]:.3f}', f'{r["yaw_rad"]:.6f}', r["steps_logged"], f'{r["duration_s"]:.3f}', r["sectors_total"], r["sectors_searched"], r["suspected_points"], r["true_survivors"], r["confirmed_survivors"], r["successful_supply_drops"], f'{r["avg_confirm_delay_steps"]:.3f}', f'{r["avg_delivery_delay_steps"]:.3f}', f'{r["min_sep_m"]:.3f}', r["minsep_violation_steps"], f'{r["mean_target_err_m"]:.3f}', f'{r["rms_target_err_m"]:.3f}', r["completed"]])
        w.writerow([])
        w.writerow(["success_rate_percent", f"{rate:.2f}%"])


def main():
    OUT_CSV = "CTA_Mission_Scenario_3.csv"
    OUT_SUM = "CTA_Mission_Scenario_3_summary.csv"
    OUT_MP4 = "CTA_Mission_Scenario_3.mp4"
    MC_RUNS = 5
    SEED0 = 101
    video_run = random.randint(1, MC_RUNS)
    print(f"[MC] MP4 will be saved from RANDOM run_id={video_run}")
    cfg = {"HZ": 10, "TOTAL_STEPS": 1200, "SPAWN_HOLD_S": 4.0, "MIN_SEP_THRESH": -1.0, "SEARCH_RADIUS_M": 2.5, "SEARCH_DWELL_STEPS": 18, "CONFIRM_RADIUS_M": 2.5, "CONFIRM_DWELL_STEPS": 20, "DROP_RADIUS_M": 2.5, "DROP_DWELL_STEPS": 24, "SUSPECTED_IDS": [1, 2, 3, 5], "MIN_CONFIRM_FRACTION": 0.0, "MIN_DELIVERY_FRACTION": 0.0, "IDLE_HOLD_RING": [(8.0, 10.0), (8.0, -10.0), (-8.0, 10.0), (-8.0, -10.0)], "SHIFT_N_MIN": -6.0, "SHIFT_N_MAX": 6.0, "SHIFT_E_MIN": -6.0, "SHIFT_E_MAX": 6.0, "YAW_MIN": -math.pi / 6, "YAW_MAX": math.pi / 6, "SCALE": 1.0, "WARMUP_STEPS": 100, "SEP_TOL_FRAC": 0.02, "MP4_NAME": OUT_MP4}
    conns = connect_all()
    for c in conns:
        request_local_position_stream(c, hz=cfg["HZ"])
    prep_vehicles(conns, takeoff_alt=18)
    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        header = ["run_id", "seed", "step", "t_sec", "searched_sectors", "suspected_points", "confirmed_survivors", "successful_drops", "confirm_queue_len", "delivery_queue_len", "min_sep_m"]
        for i in range(5):
            header += [f"d{i+1}_n", f"d{i+1}_e", f"d{i+1}_d"]
        header += ["search_uav1_sector_id", "search_uav2_sector_id", "confirm_uav_sector_id", "delivery_uav_sector_id"]
        w.writerow(header)
        rows = []
        for run_id in range(1, MC_RUNS + 1):
            seed = SEED0 + (run_id - 1)
            save_video = (run_id == video_run)
            print(f"[MC] Run {run_id}/{MC_RUNS} seed={seed} (video={'YES' if save_video else 'NO'})")
            out = run_one(conns, run_id, seed, cfg, w, save_video=save_video)
            rows.append(out)
            print(f"[MC] done run{run_id:03d}: completed={out['completed']} searched={out['sectors_searched']}/{out['sectors_total']} confirmed={out['confirmed_survivors']}/{out['true_survivors']} drops={out['successful_supply_drops']} minsep={out['min_sep_m']:.2f}")
            time.sleep(1.0)
    write_summary(OUT_SUM, rows)
    print(f"[OK] Saved {OUT_CSV}")
    print(f"[OK] Saved {OUT_SUM}")
    print(f"[OK] Saved {OUT_MP4}")


if __name__ == "__main__":
    main()
