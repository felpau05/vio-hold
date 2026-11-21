#!/usr/bin/env python3
"""Open-loop replay of one recorded ArduCopter closed-loop run through the onboard pipeline (apb_main.Pipe + Hold),
with the liveness test of Hold made visible. Diagnostic only: no code under test is changed.

Frames: px4sim/camera.py renders each processed frame of the run again from the run's TRUE pose (apsim/runs/<label>.npz),
with the run's camera settings (cfg in <label>.json: lens, mount, readout, noise, vibration, seed). Frame times: the
t column (fr.t0) of the run's onboard log, so Hold sees the frames the closed-loop program processed. Attitude: the
run's ArduCopter ATTITUDE at 100 Hz through apb_main.att_filter (GyroAtt, as onboard). Height: the onboard log's own
height column (hgt.at(fr.t0) of the run). Settings: the run's onboard settings file (lens, mask, mount, hold_cfg).

Per processed frame it writes: time from the hold start, the Pipe's answer (sent, why), and the liveness numbers of
Hold._track1 against the reference frame: mm = |median picture motion| px, md = |median attitude-predicted motion| px,
p90dd = 90th percentile of the per-point attitude-predicted motion px, follow = |median(dm - dd)| px, and the
verdict that Hold's own rule gives (move / still / none). Also the true height and yaw change since the reference.

Run: ~/.venvs/vio/bin/python integ/gap/replay_run.py <label> [--from -12] [--to 3] [--hold_cfg '{"live_s": 1.0}']
     [--patch path/to/hold.py]   (a patched copy of hold.py, for the fix test)
Writes integ/gap/results/replay_<label><tag>.csv and prints the reason counts in the longest gap.
"""
import argparse
import csv
import importlib.util
import json
import math
import os
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(PROJ, "apsim", "runs")
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]


def load_hold(patch):
    if not patch:
        import hold
        return hold
    spec = importlib.util.spec_from_file_location("hold", patch)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["hold"] = mod
    spec.loader.exec_module(mod)
    return mod


def liveness_numbers(h, kf, r):
    """The numbers of Hold._track1's live test, from the reference frame and the returned dict (same rule)."""
    ref = kf["ref"]
    p2, pred = r["p2"], r["pred"]
    com = np.isfinite(p2[:, 0]) & np.isfinite(ref["px"][:, 0]) & np.isfinite(pred[:, 0])
    if com.sum() < h.MIN_PTS // 2:
        return None
    dm, dd = p2[com] - ref["px"][com], pred[com] - ref["pred"][com]
    med = lambda x: np.partition(x, (len(x) - 1) // 2, axis=0)[(len(x) - 1) // 2]   # noqa: E731  hold._med
    mm, mdv = float(np.linalg.norm(med(dm))), med(dd)
    mp = float(med(np.linalg.norm(dm, axis=1)))      # median per-point motion (shift, turn and scale), not in hold.py
    md = float(np.linalg.norm(mdv))
    follow = float(np.linalg.norm(med(dm - dd)))
    p90 = float(np.percentile(np.linalg.norm(dd, axis=1), 90))
    if mm > h.MOVE_PX or (md > h.LIVE_STILL_PX and follow < h.MOVE_TOL * md):
        v = "move"
    elif mm < h.LIVE_STILL_PIC_PX and p90 < h.LIVE_STILL_PX:
        v = "still"
    else:
        v = "none"
    # split the predicted and the measured motion into shift + turn + scale about the picture centre (least squares):
    # the medians of the live test see only the shift
    c = np.array([h.K[0, 2], h.K[1, 2]])
    q = ref["pred"][com] - c

    def sim(v):
        A = np.zeros((2 * len(q), 4))
        A[0::2, 0], A[1::2, 1] = 1, 1
        A[0::2, 2], A[1::2, 2] = q[:, 0], q[:, 1]          # scale
        A[0::2, 3], A[1::2, 3] = -q[:, 1], q[:, 0]         # turn
        x = np.linalg.lstsq(A, v.reshape(-1), rcond=None)[0]
        return math.degrees(x[3]), 100 * x[2]
    rd, sd = sim(dd)
    rm, sm = sim(dm)
    # motion since the last "moved" proof (or the reference) of the LK-tracked corners, and what one similarity
    # (shift, turn, scale) leaves of it: noise moves each point its own way, ground motion moves them together
    sn = ref.get("seen", ref["px"])
    cs = np.isfinite(p2[:, 0]) & np.isfinite(sn[:, 0]) & ~kf["virt"]
    mv_mot = mv_res = None
    if cs.sum() >= h.MIN_PTS // 2:
        import hold as _H
        a_, b_ = sn[cs].astype(float), p2[cs].astype(float)
        C_, s_, ph_ = _H._similarity(b_, a_)
        res_ = b_ - (C_ + s_ * a_ @ _H._rot2(ph_).T)
        mv_mot, mv_res = float(med(np.linalg.norm(b_ - a_, axis=1))), float(med(np.linalg.norm(res_, axis=1)))
    return dict(mm=mm, mp=mp, md=md, follow=follow, p90dd=p90, verdict=v, ref_t=float(ref["t"]), n_com=int(com.sum()),
                turn_dd=rd, scale_dd=sd, turn_dm=rm, scale_dm=sm, mv_mot=mv_mot, mv_res=mv_res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("--from", dest="t_from", type=float, default=None, help="s from the hold start (default: first frame)")
    ap.add_argument("--to", dest="t_to", type=float, default=3.0)
    ap.add_argument("--hold_cfg", default="{}")
    ap.add_argument("--patch", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--vib", type=float, default=None, help="diagnostic: camera vibration (deg) of the render")
    ap.add_argument("--att", default="fc", help="diagnostic: 'true' gives Hold the true attitude (yaw turned by "
                    "the FC's mean yaw offset, no filter) instead of the FC's ATTITUDE through GyroAtt")
    a = ap.parse_args()

    hold = load_hold(a.patch)
    import apb_main
    import camera as C
    import capture
    import settings as S

    d = np.load(os.path.join(RUNS, a.label + ".npz"))
    run = json.load(open(os.path.join(RUNS, a.label + ".json")))
    tr, att_rows, ekf, te = d["truth"], d["att"], d["ekf"], float(d["t_engage"])
    c = run["cfg"]
    if a.vib is not None:
        c = dict(c, vib_deg=a.vib)
    s = S.load(os.path.join(RUNS, a.label + "_settings.json"))
    s["hold_cfg"] = dict(s["hold_cfg"], **json.loads(a.hold_cfg))
    if not os.path.exists(s["mask_file"]):
        s["mask_file"] = ""
    rows = [x for x in csv.DictReader(open(os.path.join(RUNS, a.label + "_onboard.csv"))) if x["t"]]

    def pose(t):
        t = np.atleast_1d(np.asarray(t, float))
        i = np.clip(np.searchsorted(tr[:, 0], t) - 1, 0, len(tr) - 2)
        f = np.clip((t - tr[i, 0]) / (tr[i + 1, 0] - tr[i, 0]), 0, 1)[:, None]
        v = tr[i, 1:4] * (1 - f) + tr[i + 1, 1:4] * f
        q0, q1 = tr[i, 4:], tr[i + 1, 4:]
        q1 = q1 * np.sign(np.sum(q0 * q1, axis=1))[:, None]
        q = q0 * (1 - f) + q1 * f
        return v[:, 0], v[:, 1], v[:, 2], q / np.linalg.norm(q, axis=1)[:, None]

    ground = C.photo_ground(c["ground"]) if not c["ground"].startswith("snow") else C.make_ground(c["ground"])
    cam = C.Camera(c, ground)
    att, hgt, f = hold.Series(1e9), hold.Series(1e9), apb_main.att_filter(s)
    if a.att == "true":
        ty = C.q_yaw(tr[:, 4:])
        dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att_rows[:, 0], np.unwrap(att_rows[:, 3])) - ty)))))
        rr, pp = C.q_roll_pitch(tr[:, 4:])
        for r in np.c_[tr[:, 0], rr, pp, np.angle(np.exp(1j * (ty + dy)))][::2]:
            att.add(*r)
    else:
        for r in att_rows[::2]:                              # ATTITUDE at about 100 Hz, as the onboard program got it
            att.add(*(f(tuple(r)) if f else r))
    for x in rows:
        if x["height"]:
            t = float(x["t"])
            if t > hgt.last_t():
                hgt.add(t, float(x["height"]))
    fc_pos = lambda t: np.array([np.interp(t, ekf[:, 0], ekf[:, 1]), np.interp(t, ekf[:, 0], ekf[:, 2])])   # noqa: E731
    pipe = apb_main.Pipe(s, fc_pos)
    judge = capture.judge(s)

    # wrap Hold._track to read the live test numbers of every successful track
    info = {}
    orig = hold.Hold._track

    def track(self, kf, img, t0, att_, hgt_, tref, deep=False, start=None):
        r = orig(self, kf, img, t0, att_, hgt_, tref, deep, start)
        if isinstance(r, dict) and not deep:
            info["live"] = liveness_numbers(self, kf, r)
        return r
    hold.Hold._track = track

    tag = a.tag or (("" if a.hold_cfg == "{}" else "_cfg") + ("" if a.vib is None else f"_vib{a.vib:g}")
                    + ("" if a.att == "fc" else f"_att{a.att}"))
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    out_path = os.path.join(HERE, "results", f"replay_{a.label}{tag}.csv")
    w = csv.writer(open(out_path, "w", newline=""))
    cols = ["t_rel", "sent", "state", "why", "orig_sent", "orig_why", "mm", "md", "follow", "p90dd", "verdict", "ref_age",
            "n_com", "dh_true_since_ref", "dyaw_fc_deg_since_ref", "turn_dd_deg", "scale_dd_pct", "turn_dm_deg",
            "scale_dm_pct", "dyaw_hold_deg_since_ref", "dyaw_true_deg_since_ref"]
    w.writerow(cols)
    out, blank = [], np.zeros((cam.H, cam.W), np.uint8)
    yaw_u = np.unwrap(att_rows[:, 3])
    yaw_true = np.unwrap(C.q_yaw(tr[:, 4:]))
    for x in rows:
        t0 = float(x["t"])
        rel = t0 - te
        if rel > a.t_to:
            break
        if a.t_from is not None and rel < a.t_from:
            continue
        gated = x["why"].startswith("height") and "under" in x["why"]
        img = blank if gated else cam.finish(cam.render(t0, pose))
        fr = capture.Frame(img, t0, t0 + s["video_delay_s"], 0, None if gated else judge(img))
        info.clear()
        e, ok, why = pipe.step(fr, att, hgt)
        lv = info.get("live")
        dh = dyaw = ""
        if lv:
            dh = float(pose(t0)[2][0] - pose(lv["ref_t"])[2][0])
            dyaw = math.degrees(float(np.interp(t0, att_rows[:, 0], yaw_u) - np.interp(lv["ref_t"], att_rows[:, 0], yaw_u)))
            ah = att.array()
            yh = np.unwrap(ah[:, 3])
            dyaw_h = math.degrees(float(np.interp(t0, ah[:, 0], yh) - np.interp(lv["ref_t"], ah[:, 0], yh)))
            dyaw_t = math.degrees(float(np.interp(t0, tr[:, 0], yaw_true) - np.interp(lv["ref_t"], tr[:, 0], yaw_true)))
        row = [round(rel, 3), int(ok), e.state if e else "", why or "", x["sent"], x["why"]]
        row += ([round(lv[k], 3) for k in ("mm", "md", "follow", "p90dd")] + [lv["verdict"], round(t0 - lv["ref_t"], 3),
                lv["n_com"], round(dh, 3), round(dyaw, 3)] + [round(lv[k], 4) for k in ("turn_dd", "scale_dd", "turn_dm",
                "scale_dm")] + [round(dyaw_h, 4), round(dyaw_t, 4)]) if lv else [""] * 15
        w.writerow(row)
        out.append(row)
    # longest gap without a sent fix in [-10, +3] s of the hold start, and the reasons inside it
    sent_t = [r[0] for r in out if r[1] == 1 and -10 < r[0] < 3]
    if len(sent_t) > 1:
        g = np.diff(sent_t)
        k = int(np.argmax(g))
        lo, hi = sent_t[k], sent_t[k + 1]
        reasons = Counter(r[3].split(" (")[0] for r in out if lo < r[0] < hi)
        verdicts = Counter(r[10] for r in out if lo < r[0] < hi)
        print(f"{a.label}{tag}: replay gap {g[k]:.2f} s from {lo:+.2f} to {hi:+.2f} s; reasons {dict(reasons)}; "
              f"live verdicts {dict(verdicts)}")
    print("wrote", os.path.relpath(out_path, PROJ))


if __name__ == "__main__":
    main()
