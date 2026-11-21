#!/usr/bin/env python3
"""Open loop: the analog PAL camera against the Orqa Justice D, estimator only (Hold), no drone physics.

Pictures: px4sim/camera.py (own geometry, not hold.py's) renders each frame from the TRUE pose of a recorded
ArduCopter SITL flight: apsim/runs/c_d140_100m_w20g40_s<seed>.npz (100 m, wind 20 gusting 40 km/h, the wind series
of that seed, validation seeds 11-13). The horizontal motion and the attitude are those of that flight; the height
is shifted to 20, 50 or 100 m (the whole height curve moves, the FC's height error stays). Same flight, ground,
seeds and camera vibration for every camera. Hold gets ArduCopter's own ATTITUDE (with rates, 100 Hz) and its
LOCAL_POSITION_NED height (10 Hz) from that flight, the exact frame times (no stamp error), the true mount and the
true readout time, unless a variant says otherwise.
Lens given to Hold: PAL - "Round 1 independent k2" (the true lens is "Round 1 Metashape fisheye", as in the closed
loop); Justice D - camera.calib_lens(true lens): fx +0.5 %, fy -0.5 %, k1 +0.01, no k2, centre at the picture centre.
Score: every fix Hold lets through (var not None), against the truth turned into the FC's heading frame. No re-anchor
help after a loss (as sim_ground.py). Ground: ottawa2022 (VALIDATION site).

Run: ~/.venvs/vio/bin/python integ/justice_d/open_loop.py [--workers 6] [--only PAL,JD_full,JD_crop] [--heights 20,50,100]
Each job writes integ/justice_d/open/<label>.json (refuses to write over an existing file; --tag makes new names).
--diag (diagnostic, not a result for the camera choice): the same frames with Hold given the exact true lens, the true
attitude, or both, to show which error source sets the number. --centre: true attitude, Justice D calib-like lens with
and without the true lens centre.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import camera as C   # noqa: E402
import hold          # noqa: E402
import sitl          # noqa: E402

OUT = os.path.join(HERE, "open")
GROUND = "ottawa2022"
FLIGHT = os.path.join(PROJ, "apsim", "runs", "c_d140_100m_w20g40_s{seed}.npz")
CAMERAS = {"PAL": dict(video="PAL"),
           "JD_full": dict(video="JUSTICE_D", true_lens="sim Justice D near full sensor"),
           "JD_crop": dict(video="JUSTICE_D", true_lens="sim Justice D near crop")}
SECONDS = 55.0


def flight(seed, height):
    """(pose(t), att Series, hgt Series, yaw offset, t_start) from the recorded flight, height shifted."""
    path = FLIGHT.format(seed=seed)
    d = np.load(path)
    tr, att, ekf, te = d["truth"], d["att"], d["ekf"], float(d["t_engage"])
    hold_rows = (tr[:, 0] >= te) & (tr[:, 0] <= te + 60)
    dh = height - float(np.median(tr[hold_rows, 3]))

    def pose(t):
        t = np.atleast_1d(np.asarray(t, float))
        i = np.clip(np.searchsorted(tr[:, 0], t) - 1, 0, len(tr) - 2)
        f = np.clip((t - tr[i, 0]) / (tr[i + 1, 0] - tr[i, 0]), 0, 1)[:, None]
        v = tr[i, 1:4] * (1 - f) + tr[i + 1, 1:4] * f
        q0, q1 = tr[i, 4:], tr[i + 1, 4:]
        q1 = q1 * np.sign(np.sum(q0 * q1, axis=1))[:, None]
        q = q0 * (1 - f) + q1 * f
        return v[:, 0], v[:, 1], v[:, 2] + dh, q / np.linalg.norm(q, axis=1)[:, None]
    ty = C.q_yaw(tr[:, 4:])
    dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
    A, Hs = hold.Series(1e9), hold.Series(1e9)
    for r in att[::2]:                       # ATTITUDE at 100 Hz, as the onboard program gets it
        A.add(*r)
    for r in ekf[::3]:                       # LOCAL_POSITION_NED height at 10 Hz (settings hgt_hz)
        Hs.add(r[0], -r[3] + dh)
    md5 = hashlib.md5(open(path, "rb").read()).hexdigest()[:10]
    return pose, A, Hs, dy, te, dict(file=os.path.relpath(path, PROJ), md5=md5, dh=round(dh, 2))


def estimator_lens(cam, c, d, exact=False):
    """K, D the estimator gets (never camera.TRUE_LENSES). exact=True (diagnostic only): the true lens itself, as
    camera.py renders it (camera.lens at the picture size: no field shift, no p1, p2, b2)."""
    if exact is True:
        return C.lens(c["true_lens"], cam.W, cam.H)
    if cam.digital:
        m = C.calib_lens(c["true_lens"])
        if exact == "centre":     # diagnostic: the calib-like lens with the true centre
            t = C.profile(c["true_lens"])
            m.update(cx=t["cx"], cy=t["cy"], name=m["name"] + " true centre")
        p = os.path.join(d, "lens.json")
        json.dump({"models": [m]}, open(p, "w"))
        return hold.load_lens(p, m["name"], cam.W, cam.H)          # field None: a digital frame is whole
    return hold.load_lens(C.LENS_JSON, c["model_lens"], cam.W, cam.H, field=0)   # as apb_main.lens, field top


def true_attitude(pose, att, dy):
    """Diagnostic: the true roll, pitch and yaw (+ the FC's yaw offset) at the FC's ATTITUDE times, with the FC's
    body rates. No camera vibration (the FC does not feel it either)."""
    a = att.array()
    _, _, _, q = pose(a[:, 0])
    r, p = C.q_roll_pitch(q)
    y = np.angle(np.exp(1j * (C.q_yaw(q) + dy)))
    A = hold.Series(1e9)
    for row in np.c_[a[:, 0], r, p, y, a[:, 4:7]]:
        A.add(*row)
    return A


def variants(cam_name, height, readout, diag=False):
    """name -> (readout given to Hold, process every n-th frame, exact lens?, true attitude?)."""
    v = {"main": (readout, 1, False, False)}
    if diag == "centre":
        return {"att_true": (readout, 1, False, True), "att_true_lens_centre": (readout, 1, "centre", True)}
    if diag:
        v.update({"lens_exact": (readout, 1, True, False), "att_true": (readout, 1, False, True),
                  "lens_exact_att_true": (readout, 1, True, True)})
    elif cam_name.startswith("JD") and height == 100:
        v.update({"readout_0": (0.0, 1, False, False), "readout_wrong_sign": (-readout, 1, False, False),
                  "rate_30Hz": (readout, 2, False, False), "rate_15Hz": (readout, 4, False, False),
                  "rate_10Hz": (readout, 6, False, False)})
    return v


def stats(rows, n_frames):
    e = np.array([r[1] for r in rows if r[1] is not None])
    out = dict(frames=n_frames, fixes=len(e), pct_frames_fix=round(100.0 * len(e) / max(n_frames, 1), 1),
               lost_frames=sum(1 for r in rows if r[2] == "lost"))
    if len(e):
        out.update(rms_m=float(np.sqrt(np.mean(e ** 2))), worst_m=float(e.max()), p95_m=float(np.percentile(e, 95)),
                   first_fix_s=float(next(r[0] for r in rows if r[1] is not None)))
    return out


def job(cam_name, height, seed, tag="", hold_seconds=SECONDS, diag=False):
    cv2.setNumThreads(1)
    label = f"ol_{cam_name}_{height}m_w20g40_s{seed}{tag}"
    path = os.path.join(OUT, label + ".json")
    if os.path.exists(path):
        raise FileExistsError(f"{path} exists. Give --tag to make a new run.")
    case = dict(CAMERAS[cam_name], ground=GROUND, seed=seed)
    c = {**sitl.DEFAULTS, **sitl.video_defaults(case), **case}
    cam = C.Camera(c, C.photo_ground(GROUND))
    pose, att, hgt, dy, te, src = flight(seed, height)
    Rf = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])
    cfg = {}
    if cam.digital:   # the overlay's Hold settings, as the onboard program gets them (tools/profiles/justice_d.json)
        cfg = json.load(open(sitl.JUSTICE_D_OVERLAY)).get("hold_cfg", {})
    with tempfile.TemporaryDirectory() as d:
        lenses = {False: estimator_lens(cam, c, d), True: estimator_lens(cam, c, d, exact=True)}
        if cam.digital:
            lenses["centre"] = estimator_lens(cam, c, d, exact="centre")
    att_true = true_attitude(pose, att, dy) if diag else None
    V = variants(cam_name, height, c["readout_s"], diag)
    holds = {}
    for name, (ro, _, exact, _) in V.items():
        h = hold.Hold(*lenses[exact], hold.make_R_bc(), ro, **cfg)
        N, E, _, _ = pose(te)
        h.reanchor(Rf @ np.array([N[0], E[0]]))
        holds[name] = h
    rows = {k: [] for k in V}
    whys = {k: {} for k in V}             # why Hold sent nothing, by first words, per variant
    n_proc = {k: 0 for k in V}
    t_render, t_hold = 0.0, {k: 0.0 for k in V}
    n = int(hold_seconds * cam.fps)
    w0 = time.time()
    for k in range(n):
        t0 = te + k / cam.fps + cam.exposure / 2
        a = time.perf_counter()
        img = cam.finish(cam.render(t0, pose))
        t_render += time.perf_counter() - a
        for name, (ro, every, _, ta) in V.items():
            if k % every:
                continue
            a = time.perf_counter()
            e = holds[name].process(img, t0, att_true if ta else att, hgt)
            t_hold[name] += time.perf_counter() - a
            n_proc[name] += 1
            err = None
            if e.var is not None:
                Nt, Et, _, _ = pose(e.t)
                err = float(np.linalg.norm(e.pos - Rf @ np.array([Nt[0], Et[0]])))
            rows[name].append((round(e.t - te, 4), None if err is None else round(err, 4), e.state))
            if e.why:
                w = e.why.split(" (")[0]
                whys[name][w] = whys[name].get(w, 0) + 1
    res = dict(label=label, camera=cam_name, height=height, seed=seed, wind="w20g40", ground=GROUND, flight=src,
               video=c["video"], picture=[cam.W, cam.H], fps=cam.fps, true_lens=c["true_lens"],
               estimator_lens=(C.calib_lens(c["true_lens"]) if cam.digital else c["model_lens"]),
               true_readout_s=c["readout_s"], noise=c["noise"], exposure_s=c["exposure_s"], vib_deg=c["vib_deg"],
               hold_cfg=cfg, hold_seconds=hold_seconds, yaw_offset_deg=round(math.degrees(dy), 3),
               render_ms_per_frame=round(1e3 * t_render / n, 2), wall_s=round(time.time() - w0, 1),
               hold_py_md5=sitl.HOLD_MD5, camera_py_md5=hashlib.md5(open(C.__file__, "rb").read()).hexdigest()[:10],
               variants={name: dict(readout_s=ro, every=every, hz=round(cam.fps / every, 2), exact_lens=ex, true_att=ta,
                                    hold_ms_per_frame=round(1e3 * t_hold[name] / max(n_proc[name], 1), 2),
                                    **stats(rows[name], n_proc[name]), why=whys[name], fixes_t_err_state=rows[name])
                         for name, (ro, every, ex, ta) in V.items()})
    os.makedirs(OUT, exist_ok=True)
    json.dump(res, open(path, "w"))
    return {k: v for k, v in res.items() if k != "variants"} | {
        "variants": {k: {x: y for x, y in v.items() if x != "fixes_t_err_state"} for k, v in res["variants"].items()}}


def _job(a):
    try:
        return job(*a)
    except Exception as e:      # a failed job is reported, not lost
        import traceback
        return dict(label=f"ol_{a[0]}_{a[1]}m_w20g40_s{a[2]}{a[3]}", error=f"{type(e).__name__}: {e}",
                    trace=traceback.format_exc()[-1500:])


def main():
    import multiprocessing as mp
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--only", default=",".join(CAMERAS))
    ap.add_argument("--heights", default="20,50,100")
    ap.add_argument("--seeds", default="11,12,13")
    ap.add_argument("--tag", default="")
    ap.add_argument("--diag", action="store_true", help="variants: exact true lens and true attitude (diagnostic)")
    ap.add_argument("--centre", action="store_true", help="diagnostic: true attitude, calib-like lens with and without the true centre")
    a = ap.parse_args()
    jobs = [(cn, int(h), int(s), a.tag, SECONDS, "centre" if a.centre else a.diag) for h in a.heights.split(",") for cn in a.only.split(",")
            for s in a.seeds.split(",")]
    jobs.sort(key=lambda j: -(j[1] == 100 and j[0].startswith("JD")))      # long jobs first
    with mp.get_context("spawn").Pool(a.workers) as pool:
        for r in pool.imap_unordered(_job, jobs):
            if "error" in r:
                print("FAILED", r["label"], r["error"], r["trace"], flush=True)
                continue
            for k, v in r["variants"].items():
                print(f"{r['label']:34s} {k:20s} fix {v['pct_frames_fix']:5.1f} %  rms {v.get('rms_m', math.nan):6.3f} m  "
                      f"worst {v.get('worst_m', math.nan):6.3f} m  lost {v['lost_frames']:4d}  hold {v['hold_ms_per_frame']:.1f} ms",
                      flush=True)


if __name__ == "__main__":
    main()
