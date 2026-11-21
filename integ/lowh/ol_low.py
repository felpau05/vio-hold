#!/usr/bin/env python3
"""Open loop at low height: a recorded ArduCopter SITL hover in wind, replayed at a chosen height, through one or more
hold.py versions on the very same frames.

Pictures, motion and FC feeds as integ/justice_d/open_loop.py (its flight() is used): px4sim/camera.py renders each frame
from the TRUE pose of apsim/runs/c_d140_100m_w20g40_s<seed>.npz (100 m, wind 20 gusting 40 km/h, VALIDATION seeds 11-13;
the drone drifts 0.9-1.3 m from the engage point in 55 s), with the whole height curve moved to --height. Hold gets
ArduCopter's own ATTITUDE (100 Hz, with rates) and LOCAL_POSITION_NED height (10 Hz), the exact frame times, the true
mount and readout, and the calib-like lens (common.est_lens). Justice D camera with the crop lens of the SITL runs
(--lens full: the full-sensor lens), vibration 0.2 deg (FC does not see it), sensor noise 1 grey.
Rate: every 5th frame of 60 fps (12 Hz), as the 80 ms board model of the SITL runs.
The onboard Pipe is emulated (apb_main.Pipe rules): re-anchor at the FC position when Hold says 'lost' (here the true
position in the FC heading frame: a perfect FC position), and send only after 10 good fixes in a row.

Kept per frame and version: sent, fix error, state, refusal text, n, the whole-picture halves gap of every _dense call
with the true drift from that keyframe (body forward / right, m; the keyframe's true position at its time), and
which path gave the fix (corner or whole picture).

Each whole-picture call also keeps the halves gap at the picture centre's scale (gap_centre, as the repository's hold.py)
and in px of each half's own texture (gap_local, common.local_gap), whatever the version measures.
--bad KIND: a bad picture from --bad_from to --bad_to s (see BAD). --diag: diagnostic feeds (true height, true attitude,
no vibration, true lens), not results.
Run: ~/.venvs/vio/bin/python integ/lowh/ol_low.py --hold before=hold.py [--hold after=/copy/hold.py] --height 5 --seed 11
     [--ground ottawa2022|farmland|lowh_fine] [--lens crop|full] [--seconds 55] [--tag x] [--bad KIND] [--diag a,b]
Writes integ/lowh/results/ol/<label>_<version>.json, one file per version (refuses to write over a file).
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import common as CM   # noqa: E402

C = CM.C
sys.path.insert(0, os.path.join(CM.PROJ, "integ", "justice_d"))
sys.dont_write_bytecode = True   # write no cache file into integ/justice_d
import open_loop as OL   # noqa: E402  (flight(): pose, ATTITUDE and height of the recorded flight)


BAD = {
    "frozen_bottom": "the bottom half of the picture is frozen (the rows of the first bad frame), the top half is live",
    "frozen_top": "the top half is frozen, the bottom half is live",
    "overlay_half": "a static text overlay (white on dark grey, OSD-like) covers the top half; the bottom half is live",
    "overlay_flat": "a flat grey picture (no ground) with a static text overlay on its top half; fresh sensor noise",
}


def bad_picture(kind, img, store, rng):
    """img with the bad picture of kind; store keeps the frozen rows between frames."""
    H, W = img.shape
    out = img.copy()
    if kind.startswith("frozen"):
        rows = slice(H // 2, H) if kind == "frozen_bottom" else slice(0, H // 2)
        if "rows" not in store:
            store["rows"] = img[rows].copy()
        out[rows] = store["rows"]
        return out
    if "ov" not in store:
        import cv2
        ov = np.full((H // 2, W), 60, np.uint8)
        for i, line in enumerate(("REC 00:12:34   BAT 15.2 V", "ALT 5 M  SPD 0.0  GPS 14", "HOME 12 M   RSSI 99")):
            cv2.putText(ov, line, (20, 40 + 55 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.1, 235, 3)
        store["ov"] = ov.astype(np.float32)
    top = store["ov"] + rng.normal(0, 1.0, store["ov"].shape)       # fresh sensor noise: not a repeated frame
    out[:H // 2] = np.clip(top, 0, 255).astype(np.uint8)
    if kind == "overlay_flat":
        out[H // 2:] = np.clip(128 + rng.normal(0, 1.0, (H - H // 2, W)), 0, 255).astype(np.uint8)
    return out


def bad_summary(rows, lo, hi):
    w = [r for r in rows if lo <= r["t"] < hi]
    why = {}
    for r in w:
        if r["err_all"] is None:
            k = r["why"].split(" (")[0]
            why[k] = why.get(k, 0) + 1
    fx = [r for r in w if r["err_all"] is not None]
    late = [r for r in fx if r["t"] >= lo + 0.5]
    return dict(window=[lo, hi], frames=len(w), hold_fixes=len(fx), hold_fixes_after_0_5s=len(late), sent=sum(r["sent"] for r in w),
                whole_picture_fixes=sum(r["path"] == "dense" for r in fx),
                max_err_m=round(max([r["err_all"] for r in fx], default=0.0), 3),
                halves_refused=sum(1 for r in w if r["why"].startswith("whole picture: halves disagree")),
                why=dict(sorted(why.items(), key=lambda kv: -kv[1])))


def summary(rows, seconds):
    sent = [r for r in rows if r["sent"]]
    ts = np.array([r["t"] for r in sent])
    x = np.r_[0.0, ts, seconds]
    errs = np.array([r["err"] for r in sent])
    why = {}
    for r in rows:
        if not r["sent"]:
            k = r["why"].split(" (")[0]
            why[k] = why.get(k, 0) + 1
    gaps = [g["gap"] for r in rows for g in r["dense"] if g["gap"] is not None and g["cur"]]
    corner = {}
    for r in rows:
        if r.get("corner"):
            k = r["corner"].split(" (")[0]
            k = "N points tracked" if k.endswith("points tracked") else "N points match" if k.endswith("points match") else k
            corner[k] = corner.get(k, 0) + 1
    return dict(frames=len(rows), sent=len(sent), fix_hz=round(len(sent) / seconds, 2),
                longest_gap_s=round(float(np.diff(x).max()), 2),
                err_rms_m=round(float(np.sqrt(np.mean(errs ** 2))), 3) if len(errs) else None,
                err_max_m=round(float(errs.max()), 3) if len(errs) else None,
                halves_refused=sum(1 for r in rows if r["why"].startswith("whole picture: halves disagree")),
                reanchors=sum(1 for r in rows if r.get("reanchor")),
                new_anchors=sum(1 for r in rows if r.get("new_kf") == "anchor"),
                kf_changes=sum(1 for r in rows if r.get("new_kf") == "change"),
                sent_whole_picture=sum(1 for r in sent if r["path"] == "dense"),
                sent_corner=sum(1 for r in sent if r["path"] == "corner"),
                halves_gap_px=dict(n=len(gaps), median=round(float(np.median(gaps)), 2) if gaps else None,
                                   p90=round(float(np.percentile(gaps, 90)), 2) if gaps else None),
                why=dict(sorted(why.items(), key=lambda kv: -kv[1])),
                corner_path=dict(sorted(corner.items(), key=lambda kv: -kv[1])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", action="append", required=True, help="name=path/to/hold.py (repeat for more versions)")
    ap.add_argument("--height", type=float, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--ground", default="ottawa2022")
    ap.add_argument("--lens", default="crop", choices=list(CM.LENSES))
    ap.add_argument("--seconds", type=float, default=55.0)
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--reanchor", default="truth", choices=["truth", "last"],
                    help="where the emulated Pipe re-anchors after 'lost': truth (a perfect FC position) or last (Hold's "
                         "last fix, as apb_main.Pipe does without an FC position)")
    ap.add_argument("--diag", default="", help="diagnostic feeds, comma list: true_height (Hold gets the true height), "
                    "true_att (the true attitude at the FC's times, open_loop.true_attitude), vib0 (no camera vibration), "
                    "exact_lens (Hold gets the true lens)")
    ap.add_argument("--tag", default="")
    ap.add_argument("--bad", default=None, choices=list(BAD), help="a bad picture from --bad_from to --bad_to s")
    ap.add_argument("--bad_from", type=float, default=10.0)
    ap.add_argument("--bad_to", type=float, default=20.0)
    a = ap.parse_args()
    if a.seed not in (11, 12, 13):
        raise SystemExit("validation seeds 11, 12, 13 only (DATA_SPLITS.md)")
    diag = [x for x in a.diag.split(",") if x]
    label = (f"ol_{a.ground}_{a.lens}_{a.height:g}m_s{a.seed}{a.tag}" + (f"_bad_{a.bad}" if a.bad else "")
             + ("_relast" if a.reanchor == "last" else "")
             + "".join(f"_{x}" for x in diag))
    outd = os.path.join(HERE, "results", "ol")
    versions = [x.split("=", 1) for x in a.hold]
    for name, _ in versions:
        if any(os.path.exists(os.path.join(outd, f"{label}_{name}.json{z}")) for z in ("", ".gz")):
            raise SystemExit(f"{label}_{name}.json exists")
    import cv2
    cv2.setNumThreads(2)
    cam, c = CM.camera(a.ground, a.lens, a.seed, 0.0 if "vib0" in diag else 0.2)
    pose, att, hgt, dy, te, src = OL.flight(a.seed, a.height)
    if "true_att" in diag:
        att = OL.true_attitude(pose, att, dy)
    if "true_height" in diag:                     # the true height at 10 Hz (open_loop's hold module: its Series only)
        hgt = OL.hold.Series(1e9)
        for t in np.arange(te - 10.0, te + a.seconds + 2.0, 0.1):
            hgt.add(t, float(pose(t)[2][0]))
    Rf = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])

    def truth(t):                     # true (north, east) in the FC heading frame, and the true yaw in that frame
        N, E, _, q = pose(t)
        return Rf @ np.array([N[0], E[0]]), float(C.q_yaw(q)[0]) + dy

    cur = {}
    V = {}
    for name, path in versions:
        hm = CM.load_hold(os.path.abspath(path), "hold_" + name)
        CM.capture_ds(hm)
        mk, dn = hm.Hold._make_kf, hm.Hold._dense

        def make_kf(self, img, t0, att_, hgt_, tref, pos, acc, chain, pending, _mk=mk):
            k = _mk(self, img, t0, att_, hgt_, tref, pos, acc, chain, pending)
            if k is not None:
                k["_t"] = tref
                self._newkf = "anchor" if pending else "change"
            return k

        def dense(self, kf, img, idx, guess, J, p_lk, _dn=dn):
            self._ds, self._AB, self.dq = None, None, [None, None, None]
            r = _dn(self, kf, img, idx, guess, J, p_lk)
            p_now, yaw = truth(cur["tref"])
            p_kf, _ = truth(kf["_t"])
            d = p_now - p_kf
            fw, rt = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
            g = dict(cur=kf is self.kf, gap=None if self.dq[1] is None else round(float(self.dq[1]), 3),
                     resp=None if self.dq[0] is None else round(float(self.dq[0]), 3), drift_fwd=round(float(d @ fw), 3), drift_right=round(float(d @ rt), 3),
                     ok=not isinstance(r, str), why=r if isinstance(r, str) else "")
            if self._ds is not None:
                g["dl_diff_fwd_right"] = [round(float((self._ds[0] - self._ds[1]) @ fw), 4),
                                          round(float((self._ds[0] - self._ds[1]) @ rt), 4)]
                g["gap_centre"] = round(float(np.linalg.norm(self._Jc @ (self._ds[0] - self._ds[1]))), 3)
                gl = CM.local_gap(self._AB, self._ds, self.DENSE_SCALE)
                g["gap_local"] = None if gl is None else round(gl, 3)
            self._calls.append(g)
            return r
        t1 = hm.Hold._track1

        def track1(self, kf, img, t0, att_, hgt_, tref, deep, start, dense_, _t1=t1):
            r = _t1(self, kf, img, t0, att_, hgt_, tref, deep, start, dense_)
            if not dense_ and kf is self.kf and not deep:      # the corner path on the current keyframe: its outcome
                self._corner = r if isinstance(r, str) else "ok"
            return r
        hm.Hold._make_kf, hm.Hold._dense, hm.Hold._track1 = make_kf, dense, track1
        K, D = CM.est_lens(hm, cam, c, exact="exact_lens" in diag)
        h = hm.Hold(K, D, hm.make_R_bc(), c["readout_s"], **CM.HOLD_CFG)
        h.reanchor(truth(te)[0])
        V[name] = dict(h=h, rows=[], good=0, hm=hm, wall=0.0)
    n = int(a.seconds * cam.fps)
    w0 = time.time()
    store, brng = {}, np.random.default_rng(300 + a.seed)
    for k in range(0, n, a.every):
        t0 = te + k / cam.fps + cam.exposure / 2
        img = cam.finish(cam.render(t0, pose))
        if a.bad and a.bad_from <= t0 - te < a.bad_to:
            img = bad_picture(a.bad, img, store, brng)
        cur["tref"] = t0 + c["readout_s"] / 2
        for name, v in V.items():
            h = v["h"]
            h._calls, h._newkf, h._corner = [], None, None
            w = time.perf_counter()
            e = h.process(img, t0, att, hgt)
            v["wall"] += time.perf_counter() - w
            re = False
            if e.state == "lost":          # apb_main.Pipe: re-anchor at the FC position
                h.reanchor(truth(e.t)[0] if a.reanchor == "truth" else h.pos.copy())
                v["good"], re = 0, True
            if e.var is None:
                if e.state != "degraded":
                    v["good"] = 0
                sent, why = False, e.why or e.state
            else:
                v["good"] += 1
                sent = v["good"] >= 10
                why = "" if sent else f"good fix {v['good']} of 10"
            err = None
            if e.var is not None:
                err = round(float(np.linalg.norm(e.pos - truth(e.t)[0])), 4)
            cd = [g for g in h._calls if g["cur"]]
            path = ("dense" if cd and cd[-1]["ok"] else "corner") if e.var is not None else ""
            v["rows"].append(dict(t=round(t0 - te, 4), sent=sent, err=err if sent else None, err_all=err, state=e.state,
                                  why=why, n=e.n, path=path, dense=h._calls, corner=h._corner, new_kf=h._newkf, reanchor=re,
                                  height=round(float(hgt.at(t0)), 3)))
    for name, v in V.items():
        res = dict(label=f"{label}_{name}", version=name, hold_py=os.path.abspath(dict(versions)[name]), ground=a.ground,
                   lens=a.lens, true_lens=CM.LENSES[a.lens], height=a.height, seed=a.seed, flight=src, every=a.every,
                   hz=cam.fps / a.every, seconds=a.seconds, vib_deg=c["vib_deg"], diag=diag, reanchor_at=a.reanchor, hold_cfg=CM.HOLD_CFG,
                   hold_ms_per_frame=round(1e3 * v["wall"] / max(len(v["rows"]), 1), 1), wall_s=round(time.time() - w0, 1),
                   summary=summary(v["rows"], a.seconds), rows=v["rows"])
        if a.bad:
            res.update(bad=a.bad, bad_text=BAD[a.bad], bad_window=bad_summary(v["rows"], a.bad_from, a.bad_to))
        os.makedirs(outd, exist_ok=True)
        json.dump(res, open(os.path.join(outd, res["label"] + ".json"), "w"))
        s = res["summary"]
        print(f"{res['label']}: fix {s['fix_hz']}/s, longest gap {s['longest_gap_s']} s, err RMS {s['err_rms_m']} m, "
              f"halves refused {s['halves_refused']}, reanchors {s['reanchors']}, whole-picture fixes {s['sent_whole_picture']}"
              f"/{s['sent']}, gap px {s['halves_gap_px']}, hold {res['hold_ms_per_frame']} ms; why {list(s['why'].items())[:5]}"
              + (f"\n    BAD {a.bad}: {res['bad_window']}" if a.bad else ""), flush=True)


if __name__ == "__main__":
    main()
