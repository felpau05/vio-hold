#!/usr/bin/env python3
"""Open-loop replay of a recorded ArduCopter SITL flight through one or more hold.py copies, with diagnostic feeds.
No simulator runs here: px4sim/camera.py renders each frame from the TRUE pose of the recorded flight (apsim/runs/*.npz).

Flight: --flight <label> (apsim/runs/<label>.npz). The closed-loop SITL runs at 10 m (lowap_apb_10m_w20g40_s1/s2, gn_*_10m,
arm_*) are on ottawa2019fall with seeds 1-2 (TUNING). For VALIDATION use --flight c_d140_100m_w20g40_s11..13 (seeds 11-13)
with --height 10 (height curve shifted, as integ/lowh/ol_low.py) and --ground ottawa2022 or farmland.
Feeds (as integ/lowh/ol_low.py and the SITL runs): ArduCopter ATTITUDE (with rates) of the flight, LOCAL_POSITION_NED
height at 10 Hz, Justice D camera (1280x720 -> 640x360, readout 14 ms, exposure 4 ms, noise 1 grey, vibration 0.2 deg),
calib-like lens (camera.calib_lens), 12 Hz (every 5th frame of 60 fps). --att gyro: the onboard program's default
(apb_main.GyroAtt, tau 20 s; copied here, not imported). Pipe emulated: re-anchor after 'lost' (--reanchor truth: the true
position; last: Hold's last fix; ekf: the recorded FC position of the flight, which in closed loop followed our fixes),
send after 10 good fixes.
Diagnostic feeds (--diag, comma list; not results):
  true_height  Hold gets the true height (10 Hz)
  true_att     true roll, pitch, yaw (+ the FC's mean yaw offset) at the FC's ATTITUDE times, FC rates (before --att gyro)
  true_yaw     after the attitude filter: yaw replaced by the true yaw + the FC's mean yaw offset
  true_tilt    after the attitude filter: roll and pitch replaced by the true roll and pitch
  vib0         no camera vibration
  exact_lens   Hold gets the true lens
  gs           global shutter: camera readout 0 and Hold readout 0 (removes rolling shutter)
  kf_true      every keyframe (new anchor and change) is placed at the TRUE position: no chain error
  dense_off    no whole-picture path (Hold DENSE_RESP 9: every whole-picture call fails)
Run: ~/.venvs/vio/bin/python integ/diag10/replay.py --flight lowap_apb_10m_w20g40_s1 --ground ottawa2019fall --seed 1
     --hold base=integ/diag10/hold_base.py [--hold x=...] [--diag a,b] [--reanchor truth|last|ekf] [--tag t]
Writes integ/diag10/results/<label>_<version>.json.gz
"""
import argparse
import gzip
import importlib.util
import json
import math
import os
import sys
import tempfile
import time

import numpy as np
import cv2

cv2.setNumThreads(1)          # before any other OpenCV call (load limit, 2026-11-21)
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(PROJ, "px4sim")]
sys.dont_write_bytecode = True
import camera as C   # noqa: E402

TUNING = dict(grounds={"ottawa2019fall", "suburb", "prairie"}, seeds=set(range(1, 10)))
VALID = dict(grounds={"ottawa2022", "farmland"}, seeds=set(range(11, 20)))
JD = dict(video="JUSTICE_D", readout_s=0.014, exposure_s=0.004, noise=1.0, noise_corr_px=0.0, vib_deg=0.2,
          focal_scale=1.0, pp_shift=(0.0, 0.0), cam_top="front", cam_mirror=False, cam_tilt=(0.0, 0.0), cam_twist=0.0,
          true_lens="sim Justice D near crop")
HOLD_CFG = {"repeat_diff": 0.01, "h_min": 2.0}


def load_hold(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class GyroAtt:
    """Copy of apb_main.GyroAtt (2026-11-21): FC body rates integrated, leaning toward the FC attitude with tau."""

    def __init__(self, tau_s, euler_R):
        import cv2
        self.cv2, self.euler_R = cv2, euler_R
        self.tau, self.R, self.t, self.w = tau_s, None, None, None

    def __call__(self, row):
        cv2 = self.cv2
        t, w = row[0], np.asarray(row[4:7], float)
        R_fc = self.euler_R(*row[1:4])
        dt = math.inf if self.t is None else t - self.t
        if dt > 0.5:
            self.R = R_fc
        elif dt > 0:
            self.R = self.R @ cv2.Rodrigues((w + self.w) / 2 * dt)[0]
            e = cv2.Rodrigues(R_fc @ self.R.T)[0]
            self.R = cv2.Rodrigues(e * min(1.0, dt / self.tau))[0] @ self.R
            u, _, vt = np.linalg.svd(self.R)
            self.R = u @ vt
        self.t, self.w = t, w
        R = self.R
        return (t, math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
                math.atan2(R[1, 0], R[0, 0]), *row[4:])


def flight(label, height, Series):
    path = os.path.join(PROJ, "apsim", "runs", label + ".npz")
    d = np.load(path)
    tr, att, ekf, te = d["truth"], d["att"], d["ekf"], float(d["t_engage"])
    hold_rows = (tr[:, 0] >= te) & (tr[:, 0] <= te + 60)
    dh = 0.0 if height is None else height - float(np.median(tr[hold_rows, 3]))

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
    return pose, att, ekf, dh, dy, te, tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight", required=True)
    ap.add_argument("--height", type=float, default=None, help="shift the height curve to this median (m); default: as flown")
    ap.add_argument("--ground", required=True)
    ap.add_argument("--seed", type=int, required=True, help="camera seed (noise); must match the data split of the ground")
    ap.add_argument("--hold", action="append", required=True)
    ap.add_argument("--seconds", type=float, default=58.0)
    ap.add_argument("--start", type=float, default=0.0, help="start (s after the engage time of the flight)")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--reanchor", default="truth", choices=["truth", "last", "ekf"])
    ap.add_argument("--att", default="fc", choices=["fc", "gyro", "gyro_fcyaw", "gyro_tau2"],
                    help="fc: ATTITUDE as sent; gyro: apb_main.GyroAtt (tau 20 s); gyro_fcyaw: GyroAtt roll and pitch, "
                         "the FC's yaw as sent (a proposed feed change); gyro_tau2: GyroAtt with tau 2 s")
    ap.add_argument("--diag", default="")
    ap.add_argument("--cfg", default="", help="extra Hold settings as JSON, for every version")
    ap.add_argument("--tag", default="")
    ap.add_argument("--att_lead", type=float, default=12.0, help="s of attitude fed to the filter before the start; the onboard program runs GyroAtt from its own start (before take-off), so 60 is closer to the closed loop than 12")
    ap.add_argument("--yaw_rate_dps", type=float, default=0.0, help="diagnostic: add a yaw error growing at this rate")
    ap.add_argument("--yaw_off_deg", type=float, default=0.0, help="diagnostic: add a constant yaw error")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    split = "TUNING" if (a.ground in TUNING["grounds"] and a.seed in TUNING["seeds"]) else \
        "VALIDATION" if (a.ground in VALID["grounds"] and a.seed in VALID["seeds"]) else None
    if split is None:
        raise SystemExit("ground and seed must be both TUNING (ottawa2019fall/suburb/prairie, 1-9) or both VALIDATION "
                         "(ottawa2022/farmland, 11-19) (DATA_SPLITS.md)")
    diag = [x for x in a.diag.split(",") if x]
    hstr = "" if a.height is None else f"_{a.height:g}m"
    label = (f"{a.flight}{hstr}_{a.ground}_s{a.seed}_{a.reanchor}_{a.att}" + "".join(f"_{x}" for x in diag)
             + (f"_L{a.att_lead:g}" if a.att_lead != 12.0 else "") + (f"_yr{a.yaw_rate_dps:g}" if a.yaw_rate_dps else "") + (f"_yo{a.yaw_off_deg:g}" if a.yaw_off_deg else "") + a.tag)
    outd = os.path.join(HERE, "results")
    os.makedirs(outd, exist_ok=True)
    versions = [x.split("=", 1) for x in a.hold]
    for name, _ in versions:
        if os.path.exists(os.path.join(outd, f"{label}_{name}.json.gz")) and not a.force:
            raise SystemExit(f"{label}_{name}.json.gz exists")
    import cv2
    cv2.setNumThreads(1)
    hm0 = load_hold(os.path.join(HERE, "hold_base.py"), "hold_series")
    Series = hm0.Series
    c = dict(JD, seed=a.seed, vib_deg=0.0 if "vib0" in diag else 0.2)
    if "gs" in diag:
        c["readout_s"] = 0.0
    cam = C.Camera(c, C.make_ground(a.ground))
    pose, attarr, ekf, dh, dy, te, tr = flight(a.flight, a.height, Series)
    t_lo, t_hi = te + a.start - a.att_lead, te + a.start + a.seconds + 2.0
    # attitude feed
    rows = attarr[(attarr[:, 0] > t_lo) & (attarr[:, 0] < t_hi)]
    rows = rows[::2] if np.median(np.diff(rows[:, 0])) < 0.0075 else rows      # ~100 Hz, as the onboard program
    if "true_att" in diag:
        _, _, _, q = pose(rows[:, 0])
        r_, p_ = C.q_roll_pitch(q)
        y_ = np.angle(np.exp(1j * (C.q_yaw(q) + dy)))
        rows = np.c_[rows[:, 0], r_, p_, y_, rows[:, 4:7]]
    att = Series(1e9)
    g = GyroAtt(2.0 if a.att == "gyro_tau2" else 20.0, hm0.euler_R) if a.att.startswith("gyro") else None
    fcyaw = rows[:, 3].copy()
    rows = np.array([g(tuple(r)) for r in rows]) if g else rows
    if a.att == "gyro_fcyaw":
        rows[:, 3] = fcyaw
    if "true_yaw" in diag or "true_tilt" in diag:      # after the filter: replace one part with the truth
        _, _, _, q = pose(rows[:, 0])
        rows = rows.copy()
        if "true_yaw" in diag:
            rows[:, 3] = np.angle(np.exp(1j * (C.q_yaw(q) + dy)))
        if "true_tilt" in diag:
            rows[:, 1], rows[:, 2] = C.q_roll_pitch(q)
    if a.yaw_rate_dps or a.yaw_off_deg:                # diagnostic: a known yaw error (offset + ramp from the start)
        rows = rows.copy()
        rows[:, 3] = rows[:, 3] + math.radians(a.yaw_off_deg) + math.radians(a.yaw_rate_dps) * np.maximum(rows[:, 0] - (te + a.start), 0)
    for r in rows:
        att.add(*r)
    hgt = Series(1e9)
    if "true_height" in diag:
        for t in np.arange(t_lo, t_hi, 0.1):
            hgt.add(t, float(pose(t)[2][0]))
    else:
        e3 = ekf[(ekf[:, 0] > t_lo) & (ekf[:, 0] < t_hi)]
        for r in e3[::3] if np.median(np.diff(e3[:, 0])) < 0.05 else e3:
            hgt.add(r[0], -r[3] + dh)
    Rf = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])

    def truth(t):
        N, E, _, q = pose(t)
        return Rf @ np.array([N[0], E[0]])

    def fcpos(t):            # the recorded FC (EKF) position, already in the FC frame
        return np.array([np.interp(t, ekf[:, 0], ekf[:, 1]), np.interp(t, ekf[:, 0], ekf[:, 2])])

    extra = json.loads(a.cfg) if a.cfg else {}
    if "dense_off" in diag:
        extra["dense_resp"] = 9.0
    cur = {}
    V = {}
    for name, path in versions:
        hm = load_hold(os.path.abspath(path), "hold_" + name)
        mk = hm.Hold._make_kf

        def make_kf(self, img, t0, att_, hgt_, tref, pos, acc, chain, pending, _mk=mk):
            if "kf_true" in diag:
                pos = truth(tref)
            k = _mk(self, img, t0, att_, hgt_, tref, pos, acc, chain, pending)
            if k is not None:
                k["_t"] = tref
                self._newkf = "anchor" if pending else "change"
            return k
        dn = hm.Hold._dense

        def dense(self, kf, img, idx, guess, J, p_lk, _dn=dn):
            r = _dn(self, kf, img, idx, guess, J, p_lk)
            self._calls.append(dict(cur=kf is self.kf, ok=not isinstance(r, str)))
            return r
        hm.Hold._make_kf, hm.Hold._dense = make_kf, dense
        if "exact_lens" in diag:
            K, D = C.lens(c["true_lens"], cam.W, cam.H)
        else:
            m = C.calib_lens(c["true_lens"])
            with tempfile.TemporaryDirectory() as d:
                p = os.path.join(d, "lens.json")
                json.dump({"models": [m]}, open(p, "w"))
                K, D = hm.load_lens(p, m["name"], cam.W, cam.H)
        h = hm.Hold(K, D, hm.make_R_bc(), c["readout_s"], **dict(HOLD_CFG, **extra))
        h.reanchor(truth(te + a.start))
        V[name] = dict(h=h, rows=[], good=0, wall=0.0)
    n = int(a.seconds * cam.fps)
    w0 = time.time()
    k0 = int(round(a.start * cam.fps))
    for k in range(k0, k0 + n, a.every):
        t0 = te + k / cam.fps + cam.exposure / 2
        img = cam.finish(cam.render(t0, pose))
        tref = t0 + c["readout_s"] / 2
        for name, v in V.items():
            h = v["h"]
            h._calls, h._newkf = [], None
            w = time.perf_counter()
            e = h.process(img, t0, att, hgt)
            v["wall"] += time.perf_counter() - w
            re = False
            if e.state == "lost":
                p = truth(e.t) if a.reanchor == "truth" else h.pos.copy() if a.reanchor == "last" else fcpos(e.t)
                h.reanchor(p)
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
                err = [round(float(x), 4) for x in (e.pos - truth(e.t))]
            cd = [g_ for g_ in h._calls if g_["cur"]]
            path = ("dense" if cd and cd[-1]["ok"] else "corner") if e.var is not None else ""
            v["rows"].append(dict(t=round(t0 - te, 4), sent=sent, err=err, state=e.state, why=why, n=e.n, path=path,
                                  new_kf=h._newkf, reanchor=re, sd=None if e.var is None else round(math.sqrt(e.var / 2), 4)))
    for name, v in V.items():
        rr = v["rows"]
        es = np.array([r["err"] for r in rr if r["sent"]])
        summ = dict(sent=len(es), fix_hz=round(len(es) / a.seconds, 2),
                    err_rms=round(float(np.sqrt(np.mean(np.sum(es ** 2, 1)))), 3) if len(es) else None,
                    err_mean=np.round(es.mean(0), 3).tolist() if len(es) else None,
                    reanchors=sum(r["reanchor"] for r in rr), kf_changes=sum(r["new_kf"] == "change" for r in rr),
                    anchors=sum(r["new_kf"] == "anchor" for r in rr),
                    dense_share=round(float(np.mean([r["path"] == "dense" for r in rr if r["sent"]])), 3) if len(es) else None)
        why = {}
        for r in rr:
            if not r["sent"]:
                kk = r["why"].split(" (")[0]
                why[kk] = why.get(kk, 0) + 1
        summ["why"] = dict(sorted(why.items(), key=lambda kv: -kv[1])[:8])
        res = dict(label=f"{label}_{name}", version=name, hold_py=os.path.abspath(dict(versions)[name]), split=split,
                   flight=a.flight, height_shift_m=round(dh, 3), ground=a.ground, seed=a.seed, diag=diag, att=a.att,
                   reanchor=a.reanchor, cfg=extra, start=a.start, seconds=a.seconds, every=a.every,
                   hold_ms=round(1e3 * v["wall"] / max(len(rr), 1), 1), wall_s=round(time.time() - w0, 1), summary=summ, rows=rr)
        json.dump(res, gzip.open(os.path.join(outd, res["label"] + ".json.gz"), "wt"))
        print(f"{res['label']}: {split} err RMS {summ['err_rms']} m mean {summ['err_mean']} fix {summ['fix_hz']}/s "
              f"re-anchors {summ['reanchors']} kf changes {summ['kf_changes']} dense {summ['dense_share']} "
              f"hold {res['hold_ms']} ms; why {list(summ['why'].items())[:4]}", flush=True)


if __name__ == "__main__":
    main()
