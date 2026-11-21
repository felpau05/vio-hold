"""Estimator inputs from the APB's own sensors (no FC needed), and replay of apb_rec.py recordings.

ApbAtt: attitude from the APB gyro, roll and pitch leaning toward the accelerometer's gravity direction,
  yaw free (no magnetometer). Gyro bias from the first still stretch (the ICM-42605 here: ~0.5 deg/s, measured
  in buffered mode 2026-11-21). Output rows as Hold wants them: (t, roll, pitch, yaw, p, q, r), body FRD.
BaroHeight: lens height above the floor = floor_lens_m + barometric height change since the floor zero.
replay_rec: apb_rec.py chunks -> the same Pipe as the live program -> a run log (CSV) and a summary.

  python3 apb_est.py replay /path/walk_*.npz [--height 1.0] [--zero-s 3] [--out log.csv]

Assumptions not yet measured (calibrate before trusting distances): the APB IMU axes are body FRD
(imu_R_sb identity; the accelerometer's z reads -g at rest, so z is down), the Justice D lens profile,
video_delay_s for end-of-frame stamps.
"""
import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np

import apb_sensors as A
from hold import Series, euler_R

G0 = 9.80665

# APB defaults, overridable through settings keys of the same name.
APB = {
    "imu_R_sb": np.eye(3).tolist(),   # sensor -> body FRD. assumed identity until calib measures it
    "apb_att_tau_s": 5.0,             # s; roll/pitch lean toward gravity with this time constant. assumed (walking)
    "apb_acc_gate": 1.0,              # m/s^2; |a| this far from g: no gravity correction (the drone accelerates)
    "apb_still_gyro": 0.02,           # rad/s; gyro sd below this and |a| near g: still (bias estimate)
    "apb_bias_s": 2.0,                # s of still data for the first gyro bias
    "floor_lens_m": 0.05,             # m; lens height when the drone sits on the floor. assumed: measure it
    "baro_smooth_s": 0.5,             # s; causal moving average of the pressure
    "video_delay_s_eof": 0.020,       # s; end-of-frame stamp -> mid-exposure of row 0. assumed (readout ~16 ms)
}


def apb(s, k):
    return s.get(k, APB[k]) if s else APB[k]


class ApbAtt:
    """Feed gyro and accel rows (sensor axes, apb_sensors format) in time order; get attitude rows."""

    def __init__(self, s=None):
        self.Rsb = np.array(apb(s, "imu_R_sb"), float)
        self.tau, self.gate = apb(s, "apb_att_tau_s"), apb(s, "apb_acc_gate")
        self.still_sd, self.bias_s = apb(s, "apb_still_gyro"), apb(s, "apb_bias_s")
        self.bias, self.R, self.t, self.w = None, None, None, None
        self._win = []                  # (t, w, a) for the bias estimate
        self.acc = None                 # latest body accel
        self.note = "waiting for a still moment (gyro bias)"

    def _still_bias(self, t, w, a):
        self._win.append((t, w, a))
        while self._win and self._win[0][0] < t - self.bias_s:
            self._win.pop(0)
        if len(self._win) < 20 or self._win[-1][0] - self._win[0][0] < 0.9 * self.bias_s:
            return None
        ws = np.array([x[1] for x in self._win])
        an = np.linalg.norm(np.array([x[2] for x in self._win]), axis=1)
        if ws.std(0).max() < self.still_sd and abs(an.mean() - G0) < 0.5:
            return ws.mean(0)
        return None

    def _gravity_R(self, a, yaw):
        d = -a / np.linalg.norm(a)                  # down in body axes (at rest the accelerometer reads -g along down)
        roll, pitch = math.atan2(d[1], d[2]), math.atan2(-d[0], math.hypot(d[1], d[2]))
        return euler_R(roll, pitch, yaw)

    def update(self, gyro, accel):
        """gyro, accel: arrays of (t, x, y, z) rows, sensor axes, sorted. Returns new attitude rows."""
        out = []
        if not len(gyro):
            return out
        gyro, accel = np.asarray(gyro, float), np.asarray(accel, float)
        ab = (self.Rsb @ accel[:, 1:4].T).T if len(accel) else np.zeros((0, 3))
        ia = 0
        for row in gyro:
            t = row[0]
            w = self.Rsb @ row[1:4]
            while ia < len(accel) and accel[ia, 0] <= t:
                self.acc = ab[ia]
                ia += 1
            if self.acc is None:
                continue
            if self.bias is None:
                b = self._still_bias(t, w, self.acc)
                if b is None:
                    continue
                self.bias, self.note = b, None
                self.R = self._gravity_R(self.acc, 0.0)
                self.t, self.w = t, w - b
                continue
            wc = w - self.bias
            dt = t - self.t
            if dt <= 0:
                continue
            if dt > 0.2:                            # gap: restart from gravity, keep yaw
                yaw = math.atan2(self.R[1, 0], self.R[0, 0])
                self.R = self._gravity_R(self.acc, yaw)
            else:
                self.R = self.R @ cv2.Rodrigues((wc + self.w) / 2 * dt)[0]
                if abs(np.linalg.norm(self.acc) - G0) < self.gate:
                    yaw = math.atan2(self.R[1, 0], self.R[0, 0])
                    e = cv2.Rodrigues(self._gravity_R(self.acc, yaw) @ self.R.T)[0]
                    self.R = cv2.Rodrigues(e * min(1.0, dt / self.tau))[0] @ self.R
                u, _, vt = np.linalg.svd(self.R)
                self.R = u @ vt
            self.t, self.w = t, wc
            R = self.R
            out.append((t, math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
                        math.atan2(R[1, 0], R[0, 0]), *wc))
        return out


class BaroHeight:
    """Lens height above the floor from (t, Pa) rows. zero(p_floor) sets the floor pressure."""

    def __init__(self, s=None):
        self.floor = apb(s, "floor_lens_m")
        self.smooth = apb(s, "baro_smooth_s")
        self.p0, self.rows = None, []

    def zero(self, p_floor):
        self.p0 = p_floor

    def update(self, baro):
        """baro rows (t, Pa) in time order -> new height rows (t, h) once zeroed."""
        out = []
        for t, p in baro:
            self.rows.append((t, p))
            while self.rows and self.rows[0][0] < t - self.smooth:
                self.rows.pop(0)
            if self.p0 is None:
                continue
            pm = float(np.mean([r[1] for r in self.rows]))
            out.append((t, self.floor + A.pa_to_m(pm - self.p0)))
        return out


# ---------------------------------------------------------------- replay
def load_rec(paths):
    """apb_rec.py chunks -> (chunk paths in order, gyro, accel, baro arrays without the 1 s overlaps)."""
    paths = sorted(paths)
    merged = {"gyro": [], "accel": [], "baro": []}
    for p in paths:
        d = np.load(p)
        for k in merged:
            if k in d.files and len(d[k]):
                merged[k].append(d[k])
    out = []
    for k in ("gyro", "accel", "baro"):
        a = np.concatenate(merged[k]) if merged[k] else np.zeros((0, 4 if k != "baro" else 2))
        a = a[np.argsort(a[:, 0], kind="stable")]
        keep = np.r_[True, np.diff(a[:, 0]) > 0] if len(a) else np.zeros(0, bool)
        out.append(a[keep])
    return (paths, *out)


def bench_settings(s):
    """The low-height profile for hand-carried tests at about 1 m (Stage 2 bench)."""
    return dict(s, min_height_m=0.2, hold_cfg=dict(s.get("hold_cfg", {}), h_min=0.1),
                video_delay_s=apb(s, "video_delay_s_eof"))


def replay_rec(s, paths, fixed_height=None, zero_s=3.0, log_path=None, say=print):
    """Replay apb_rec.py chunks through Pipe. Height: fixed_height, or the barometer zeroed on the mean
    pressure of the first zero_s seconds (the drone on the floor). Returns (log path, summary dict)."""
    import capture
    from apb_main import Log, Pipe
    s = bench_settings(s)
    paths, gyro, accel, baro = load_rec(paths)
    if not len(gyro):
        raise ValueError("no gyro rows in the recording")
    att_f = ApbAtt(s)
    att = Series(1e9)
    for r in att_f.update(gyro, accel):
        att.add(*r)
    if not att.rows:
        raise ValueError("no still moment for the gyro bias in the whole recording")
    hgt = Series(1e9)
    if fixed_height is not None:
        hgt.add(gyro[0, 0] - 1.0, fixed_height)
        hgt.add(gyro[-1, 0] + 1.0, fixed_height)
    else:
        if not len(baro):
            raise ValueError("no barometer rows; use --height")
        bh = BaroHeight(s)
        bh.zero(float(baro[baro[:, 0] <= baro[0, 0] + zero_s, 1].mean()))
        for r in bh.update(baro):
            hgt.add(*r)
    log = Log(log_path or os.path.join(os.path.dirname(paths[0]), f"replay_{time.strftime('%H%M%S')}.csv"))
    pipe = Pipe(s, lambda t: None)
    judge = capture._Judge(s)
    counts, track = {"frames": 0, "sent": 0}, []
    for p in paths:
        d = np.load(p)
        for img, tf, seq in zip(d["imgs"], d["t_frame"], d["seq"]):
            fr = capture.Frame(img, float(tf) - s["video_delay_s"], float(tf), int(seq), judge(img))
            t1 = time.monotonic()
            e, ok, why = pipe.step(fr, att, hgt)
            counts["frames"] += 1
            counts["sent"] += int(ok)
            if e is not None:
                counts[e.state] = counts.get(e.state, 0) + 1
                if e.var is not None:
                    track.append((fr.t0, *e.pos[:2]))
            else:
                counts[why] = counts.get(why, 0) + 1
            h = float(hgt.at(fr.t0)) if hgt.rows else None
            log.row(fr, e, ok, why, (time.monotonic() - t1) * 1e3, None, h, pipe.note)
    log.close()
    tr = np.array(track) if track else np.zeros((0, 3))
    summ = dict(counts=counts, bias_dps=np.round(np.degrees(att_f.bias), 3).tolist(),
                fixes=len(tr), first=tr[0, 1:].round(3).tolist() if len(tr) else None,
                last=tr[-1, 1:].round(3).tolist() if len(tr) else None,
                max_from_first=float(np.max(np.linalg.norm(tr[:, 1:] - tr[0, 1:], axis=1))) if len(tr) else None)
    return log.path, summ


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("replay")
    r.add_argument("paths", nargs="+")
    r.add_argument("--height", type=float, help="fixed lens height (m) instead of the barometer")
    r.add_argument("--zero-s", type=float, default=3.0, help="seconds on the floor at the start (barometer zero)")
    r.add_argument("--out")
    a = ap.parse_args()
    import settings as S
    s = S.load()
    paths = [p for x in a.paths for p in sorted(glob.glob(x))]
    log, summ = replay_rec(s, paths, a.height, a.zero_s, a.out)
    print(f"log {log}")
    for k, v in summ.items():
        print(f"{k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
