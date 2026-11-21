#!/usr/bin/env python3
"""Synthetic evaluation of fwd_cam.FwdRef. SYNTHETIC PICTURES ONLY (integ/fwdcam/fwd_render.py).

  run_eval.py tuning   seeds 1-3 (choose the thresholds in fwd_cam.HZ / FP on these only; not done yet)
  run_eval.py report   seeds 4-6 (never used to choose a setting)
  run_eval.py stats SET   print the tables again from integ/fwdcam/frames_SET.npz

Site: ground/suburb.jpg (a TUNING site in DATA_SPLITS.md). Heights 5, 10, 30, 100 m. Hover: review/geometry/
common_hover.hover_path (wobble 1 deg, roll and pitch up to about 6 deg), yaw swing +-20 deg at 0.05 Hz, drift about
+-1.3 m. Pictures 640 x 480 at 5 per second, 20 s after the reference (the reference at t = 1 s).
Hidden from the code: the true lens is "Round 1 Metashape fisheye" (the code uses "Round 1 independent k2"); the true
mount is 22 deg up, 1.2 deg turned about the optical axis, 0.8 deg yawed (the code solves tilt and turn from the FC
attitude at the reference, which has an error of about 0.8 deg per axis, and assumes no mount yaw).
Gyro: the true attitude plus a drift of 0.05 deg/s (roll, pitch) and 0.3 deg/s (yaw), random signs, plus 0.02 deg noise.
Conditions, one step harder each: C0 clean, C1 + noise 3.7 grey, C2 + haze (visibility 2 km), C3 + rough skyline,
C4 + OSD text in the corners, C5 + one video field (half the rows). C6 = C5 without the skyline.
Runs at low priority with 2 library threads, and waits while integ/kfmap/SITL_ACTIVE exists.
"""
import math
import os
import sys
import time

os.environ.setdefault("OMP_NUM_THREADS", "2")
import cv2               # noqa: E402
import numpy as np       # noqa: E402

cv2.setNumThreads(2)
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PROJ)
sys.path.insert(0, HERE)
import fwd_cam as F          # noqa: E402
import fwd_render as FR      # noqa: E402

SITL_FLAG = os.path.join(PROJ, "integ", "kfmap", "SITL_ACTIVE")
W, H, HZ_REC, T_REF, T_RUN = 640, 480, 5.0, 1.0, 20.0
HEIGHTS = (5, 10, 30, 100)
SITE = "suburb"
TRUE_LENS, CODE_LENS = "Round 1 Metashape fisheye", F.FWD_LENS
MOUNT = (22.0, 1.2, 0.8)
CONDS = {"C0 clean": {}, "C1 +noise": dict(noise=3.7), "C2 +haze": dict(noise=3.7, haze_m=2000.0),
         "C3 +skyline": dict(noise=3.7, haze_m=2000.0, skyline=True),
         "C4 +text": dict(noise=3.7, haze_m=2000.0, skyline=True, text=True),
         "C5 +field": dict(noise=3.7, haze_m=2000.0, skyline=True, text=True, field=True),
         "C6 C5-skyline": dict(noise=3.7, haze_m=2000.0, text=True, field=True)}
SETS = {"tuning": (1, 2, 3), "report": (4, 5, 6)}


GIVE_UP = "06:10"          # local time: stop waiting for the simulator lock and exit


def wait_sitl():
    """Pause while the other session's simulator runs (integ/kfmap/SITL_ACTIVE). Checked before every picture."""
    said = False
    while os.path.exists(SITL_FLAG):
        if time.strftime("%H:%M") >= GIVE_UP:
            print(f"  integ/kfmap/SITL_ACTIVE still exists at {GIVE_UP}: exit", flush=True)
            sys.exit(3)
        if not said:
            print(f"  {time.strftime('%H:%M:%S')} paused: integ/kfmap/SITL_ACTIVE exists", flush=True)
            said = True
        time.sleep(20)
    if said:
        print(f"  {time.strftime('%H:%M:%S')} resumed", flush=True)


def run_one(cond, h, seed, tex):
    kw = CONDS[cond]
    K, D = FR.I.field_K(FR.I.lens_profile(TRUE_LENS), W, H)
    R_bc = FR.fwd_mount_true(*MOUNT)
    rend = FR.FwdRenderer(K, D, R_bc, W, H, tex, seed=seed, **kw)
    pos_fn, att_fn, h_fn, off = FR.hover(h, seed)
    rend.centre = off
    rng = np.random.default_rng(seed + 500)
    bias = np.radians([0.05, 0.05, 0.3]) * rng.choice([-1, 1], 3)
    fc_err = np.radians(0.8) * rng.choice([-1, 1], 2)
    lens = F.Lens.profile(W, H // 2 if kw.get("field") else H, CODE_LENS)
    ref = F.FwdRef(lens, height_m=float(h))
    ts = np.arange(T_REF, T_REF + T_RUN + 1e-9, 1.0 / HZ_REC)
    rows, t_ref, a_ref = [], None, None
    for t in ts:
        wait_sitl()
        a = att_fn(np.array([t]))[0]
        img = rend.frame(pos_fn(np.array([t]))[0], h, a)
        if t_ref is None:
            why = ref.set_reference(img, t, (a[0] + fc_err[0], a[1] + fc_err[1], a[2]))
            if why is None:
                t_ref, a_ref = t, a
            rows.append([t, np.nan, np.nan, np.nan, 0, np.nan, np.nan, np.nan, 0, np.nan, np.nan, 0, 1])
            continue
        g = a + bias * (t - t_ref) + np.radians(0.02) * rng.standard_normal(3)
        st = ref.update(img, t, tuple(g))
        tr = (F.wrap(a[0] - a_ref[0]), a[1] - a_ref[1], F.wrap(a[2] - a_ref[2]))
        nan = float("nan")
        rows.append([t, tr[0], tr[1], tr[2], int(st.tilt_ok),
                     F.wrap(st.droll - tr[0]) if st.tilt_ok else nan, (st.dpitch - tr[1]) if st.tilt_ok else nan,
                     F.wrap(st.dyaw - tr[2]) if st.yaw_ok else nan, int(st.yaw_ok),
                     F.wrap(st.droll_pts - tr[0]) if st.yaw_ok else nan, (st.dpitch_pts - tr[1]) if st.yaw_ok else nan,
                     st.n_pts, 0])
    return np.array(rows, float), t_ref


COLS = ["t", "tr_roll", "tr_pitch", "tr_yaw", "tilt_ok", "e_roll", "e_pitch", "e_yaw", "yaw_ok", "e_roll_pts",
        "e_pitch_pts", "n_pts", "is_ref"]


def pct(x, q):
    x = np.abs(x[~np.isnan(x)])
    return float(np.percentile(x, q)) if len(x) else float("nan")


def stats(rows_by_key, out):
    """One line per condition and height, seeds pooled; worst seed's p95 after the slash."""
    lines = ["SYNTHETIC pictures only (integ/fwdcam/fwd_render.py). Errors in deg of the change since the reference.",
             "refused: share of pictures after the reference with no answer (a run whose reference was refused for "
             "the whole run counts as 100 %).",
             "tilt = sqrt(roll err^2 + pitch err^2). p95 a/b: a = seeds pooled, b = worst seed. pos95 = tan(tilt p95) x h.",
             f"{'condition':15s} {'h':>4s} {'tilt_ref':>8s} {'roll50':>6s} {'roll95':>6s} {'pit50':>6s} {'pit95':>6s} "
             f"{'tilt50':>6s} {'tilt95':>11s} {'tiltmax':>7s} {'pos95_m':>7s} {'yaw_ref':>7s} {'yaw50':>6s} "
             f"{'yaw95':>11s} {'yawmax':>6s} {'ptilt95':>7s}"]
    for (cond, h), runs in rows_by_key.items():
        R = np.concatenate([r[r[:, 12] == 0] for r in runs]) if runs else np.zeros((0, 13))
        n = sum(int((r[:, 12] == 0).sum()) for r in runs)
        n_exp = len(runs) * int(round(T_RUN * HZ_REC))
        miss = n_exp - n                                                    # pictures spent waiting for a reference
        tilt_ref = (1 - (R[:, 4].sum() if len(R) else 0) / max(n_exp, 1)) * 100
        yaw_ref = (1 - (R[:, 8].sum() if len(R) else 0) / max(n_exp, 1)) * 100
        e_r, e_p, e_y = (np.degrees(R[:, k]) if len(R) else np.array([np.nan]) for k in (5, 6, 7))
        tilt = np.hypot(e_r, e_p)
        ptilt = np.hypot(np.degrees(R[:, 9]), np.degrees(R[:, 10])) if len(R) else np.array([np.nan])
        worst = lambda v: max([x for x in v if not np.isnan(x)], default=float("nan"))           # noqa: E731
        w_tilt = worst([pct(np.hypot(np.degrees(r[:, 5]), np.degrees(r[:, 6])), 95) for r in runs])
        w_yaw = worst([pct(np.degrees(r[:, 7]), 95) for r in runs])
        t95 = pct(tilt, 95)
        lines.append(f"{cond:15s} {h:4d} {tilt_ref:7.0f}% {pct(e_r, 50):6.3f} {pct(e_r, 95):6.3f} {pct(e_p, 50):6.3f} "
                     f"{pct(e_p, 95):6.3f} {pct(tilt, 50):6.3f} {t95:5.3f}/{w_tilt:5.3f} {pct(tilt, 100):7.3f} "
                     f"{math.tan(math.radians(t95)) * h:7.3f} {yaw_ref:6.0f}% {pct(e_y, 50):6.3f} "
                     f"{pct(e_y, 95):5.3f}/{w_yaw:5.3f} {pct(e_y, 100):6.3f} {pct(ptilt, 95):7.3f}"
                     + (f"  ({miss} pictures before the reference)" if miss > len(runs) else ""))
    txt = "\n".join(lines)
    print(txt)
    if out:
        open(out, "w").write(txt + "\n")
    return txt


def main(argv):
    if argv[0] == "stats":
        d = np.load(os.path.join(HERE, f"frames_{argv[1]}.npz"), allow_pickle=True)
        stats(d["runs"].item(), None)
        return
    which = argv[0]
    conds = argv[1].split(",") if len(argv) > 1 and argv[1] != "all" else list(CONDS)
    conds = [c for c in CONDS if any(c.startswith(x) for x in conds)]
    heights = [int(x) for x in argv[2].split(",")] if len(argv) > 2 else HEIGHTS
    seeds = SETS[which]
    wait_sitl()
    tex = FR.ground_texture(SITE)
    path = os.path.join(HERE, f"frames_{which}.npz")
    runs = np.load(path, allow_pickle=True)["runs"].item() if os.path.exists(path) else {}
    t0 = time.time()
    for cond in conds:
        for h in heights:
            runs[(cond, h)] = []
            for seed in seeds:
                wait_sitl()
                r, t_ref = run_one(cond, h, seed, tex)
                runs[(cond, h)].append(r)
                print(f"{cond} {h} m seed {seed}: ref at {t_ref}, {time.time() - t0:.0f} s", flush=True)
            np.savez(path, runs=np.array(runs, dtype=object), cols=np.array(COLS))
    order = {c: i for i, c in enumerate(CONDS)}
    runs = dict(sorted(runs.items(), key=lambda kv: (order.get(kv[0][0], 99), kv[0][1])))
    stats(runs, os.path.join(HERE, f"results_{which}.txt"))


if __name__ == "__main__":
    main(sys.argv[1:] or ["tuning"])
