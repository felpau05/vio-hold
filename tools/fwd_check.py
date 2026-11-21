#!/usr/bin/env python3
"""How far the FC attitude moved against the forward-camera reference during each hold. A CHECK, not a control.

  tools/fwd_check.py --fwd "logs/fwd_*.npz" --run logs/run_X.csv --raw "logs/raw_X_*.npz" [--delay-s 0.06]
                     [--mask MASK.png] [--lens NAME] [--alpha-deg 15]

Inputs (all in the board's time.monotonic() clock):
  --fwd  forward pictures from tools/fwd_rec.py (imgs, t_grab, K, D)
  --run  the program's run log (apb_main.Log: t = frame time, mode, note). A hold starts at a row whose note has
         "engaged" and ends at the next "released" (or the end of the log).
  --raw  the program's raw chunks (apb_main.Raw, run --raw): att rows (t, roll, pitch, yaw, p, q, r) of the FC.
For each hold: the reference is the first forward picture after the switch with a horizon (FC roll and pitch then
solve the mount). Then for each picture: the FC change since the reference (roll, pitch, yaw) against the camera's.
The FC body rates, integrated from the reference, are the gyro prediction for the far-point search.
One line for each hold: p50 and p95 of the tilt difference (deg), the camera's yaw change, the FC-minus-camera yaw at
the end of the hold (and its largest size), and the share of pictures with no horizon.
--delay-s: the forward picture's age at t_grab (assumed 0.06 s; not measured). At 20 deg/s of wobble, each 10 ms of
delay error adds 0.2 deg of tilt difference: measure it before trusting a p95.
"""
import argparse
import csv
import glob
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, PROJ)
os.environ.setdefault("OMP_NUM_THREADS", "2")

import cv2               # noqa: E402
import numpy as np       # noqa: E402

import fwd_cam as F      # noqa: E402

cv2.setNumThreads(2)


def holds_from_log(path):
    """[(t_start, t_end, mode, median height or None)] from the run log's "engaged" / "released" notes
    (t: frame time)."""
    out, start, mode, last_t, hs = [], None, "", None, []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["t"])
            except (KeyError, ValueError):
                continue                                   # "no frame" rows have no time
            last_t = t
            note = row.get("note") or ""
            if "engaged" in note and start is None:
                start, mode, hs = t, row.get("mode", ""), []
            elif "released" in note and start is not None:
                out.append((start, t, mode, float(np.median(hs)) if hs else None))
                start = None
            if start is not None:
                try:
                    hs.append(float(row["height"]))
                except (KeyError, ValueError):
                    pass
    if start is not None and last_t is not None:
        out.append((start, last_t, mode, float(np.median(hs)) if hs else None))
    return out


def load_fwd(paths):
    imgs, ts, K, D = [], [], None, None
    for p in sorted(paths):
        d = np.load(p, allow_pickle=False)
        imgs.extend(list(d["imgs"]))
        ts.extend(list(np.asarray(d["t_grab"], float)))
        if K is None and "K" in d and np.any(d["K"]):
            K, D = d["K"], d["D"]
    order = np.argsort(ts)
    return [imgs[i] for i in order], np.asarray(ts)[order], K, D


def load_att(paths):
    rows = {}
    for p in sorted(paths):
        for r in np.load(p)["att"]:
            rows[float(r[0])] = r
    if not rows:
        return np.zeros((0, 7))
    return np.array([rows[k] for k in sorted(rows)])


class GyroOnly:
    """The FC body rates (p, q, r) integrated from a start attitude, no leaning: the gyro's attitude change."""

    def __init__(self, att, t0):
        self.att = att
        i0 = max(0, int(np.searchsorted(att[:, 0], t0)) - 1)
        R = F.euler_to_R(*att[i0, 1:4])
        self.t, self.R = [att[i0, 0]], [R]
        for k in range(i0 + 1, len(att)):
            dt = att[k, 0] - att[k - 1, 0]
            w = (att[k, 4:7] + att[k - 1, 4:7]) / 2 if att.shape[1] >= 7 else np.zeros(3)
            if att.shape[1] < 7 or dt > 0.5:
                R = F.euler_to_R(*att[k, 1:4])                # no rates or a gap: the FC attitude as it is
            else:
                R = R @ cv2.Rodrigues(w * dt)[0]
            self.t.append(att[k, 0])
            self.R.append(R)
        self.t = np.asarray(self.t)

    def at(self, t):
        i = int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))
        if i > 0 and abs(self.t[i - 1] - t) < abs(self.t[i] - t):
            i -= 1
        return F.R_to_euler(self.R[i])


def fc_at(att, t):
    """FC (roll, pitch, yaw) at t, linear between rows (yaw unwrapped)."""
    yaw = np.unwrap(att[:, 3])
    return (float(np.interp(t, att[:, 0], att[:, 1])), float(np.interp(t, att[:, 0], att[:, 2])),
            float(np.interp(t, att[:, 0], yaw)))


def check_hold(imgs, ts, att, lens, t0, t1, mask=None, alpha_deg=15.0, height_m=None, max_ref_s=3.0):
    """Per-picture differences for one hold. Returns a dict, or None when no reference was found."""
    idx = [i for i, t in enumerate(ts) if t0 <= t <= t1]
    if not idx or len(att) < 2 or att[0, 0] > t0 or att[-1, 0] < t1:
        return dict(why="no forward pictures or no FC attitude over the hold", n=len(idx))
    ref = F.FwdRef(lens, mask=mask, alpha_deg=alpha_deg, height_m=height_m)
    k, t_ref, why = 0, None, None
    for k, i in enumerate(idx):
        if ts[i] - t0 > max_ref_s:
            break
        why = ref.set_reference(imgs[i], ts[i], fc_at(att, ts[i]))
        if why is None:
            t_ref = ts[i]
            break
    if t_ref is None:
        return dict(why=f"no reference in the first {max_ref_s:g} s: {why}", n=len(idx))
    gyro = GyroOnly(att, t_ref)
    a0 = fc_at(att, t_ref)
    d_tilt, d_yaw, cam_yaw, no_hz, n = [], [], [], 0, 0
    for i in idx[k + 1:]:
        st = ref.update(imgs[i], ts[i], gyro.at(ts[i]))
        a = fc_at(att, ts[i])
        n += 1
        if st.tilt_ok:
            d_tilt.append(math.degrees(math.hypot(F.wrap(a[0] - a0[0]) - st.droll, (a[1] - a0[1]) - st.dpitch)))
        else:
            no_hz += 1
        if st.yaw_ok:
            cam_yaw.append(math.degrees(st.dyaw))
            d_yaw.append(math.degrees(F.wrap((a[2] - a0[2]) - st.dyaw)))
    return dict(why=None, n=n, t_ref=t_ref, d_tilt=np.array(d_tilt), d_yaw=np.array(d_yaw), cam_yaw=np.array(cam_yaw),
                no_hz=no_hz, mount_deg=(math.degrees(ref.alpha), math.degrees(ref.rho)))


def line(k, t0, t1, mode, r):
    head = f"hold {k} ({t1 - t0:5.1f} s, {mode or '?'}):"
    if r.get("why"):
        return f"{head} NO RESULT: {r['why']} ({r.get('n', 0)} pictures)"
    dt, dy, cy = r["d_tilt"], r["d_yaw"], r["cam_yaw"]
    p = lambda x, q: f"{np.percentile(x, q):.2f}" if len(x) else "-"           # noqa: E731
    return (f"{head} tilt FC-camera p50 {p(dt, 50)} p95 {p(dt, 95)} deg; camera yaw change "
            f"{(cy[-1] if len(cy) else float('nan')):+.1f} deg (range {p(cy, 0)} to {p(cy, 100)}); FC-camera yaw "
            f"at end {(dy[-1] if len(dy) else float('nan')):+.2f}, largest {p(np.abs(dy), 100)} deg; no horizon "
            f"{r['no_hz'] / max(r['n'], 1):.0%} of {r['n']}; mount solved {r['mount_deg'][0]:.1f} deg up, "
            f"{r['mount_deg'][1]:+.1f} deg turned")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--fwd", required=True, help="forward chunks (glob)")
    ap.add_argument("--run", required=True, help="run log CSV")
    ap.add_argument("--raw", required=True, help="the program's raw chunks with FC attitude (glob)")
    ap.add_argument("--delay-s", type=float, default=0.06)
    ap.add_argument("--mask", default=None, help="PNG, 0 = ignore (OSD text), size of the forward pictures")
    ap.add_argument("--lens", default=None, help="lens profile name (default: the K, D stored by fwd_rec.py)")
    ap.add_argument("--alpha-deg", type=float, default=15.0)
    a = ap.parse_args(argv)
    fp, rp = glob.glob(a.fwd), glob.glob(a.raw)
    if not fp:
        print(f"no forward chunks match {a.fwd}")
        return 2
    imgs, ts, K, D = load_fwd(fp)
    ts = ts - a.delay_s
    h, w = imgs[0].shape[:2]
    lens = F.Lens.profile(w, h, a.lens) if (a.lens or K is None) else F.Lens(K, D, w, h)
    att = load_att(rp)
    mask = cv2.imread(a.mask, cv2.IMREAD_GRAYSCALE) if a.mask else None
    holds = holds_from_log(a.run)
    print(f"{len(imgs)} forward pictures {ts[0]:.1f}-{ts[-1]:.1f} s, {len(att)} FC attitude rows, {len(holds)} holds "
          f"in {os.path.basename(a.run)}. Picture delay assumed {a.delay_s:g} s.")
    for k, (t0, t1, mode, hgt) in enumerate(holds, 1):
        print(line(k, t0, t1, mode, check_hold(imgs, ts, att, lens, t0, t1, mask, a.alpha_deg, hgt)), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
