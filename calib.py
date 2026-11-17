"""Event-day calibration from a 20-60 s recording made while a person turns the drone by hand.

A camera that only turns sees a far scene point in one fixed NED direction. Each tracked point's
NED ray in its keyframe and in a later frame must agree when the picture orientation (4 tops x
mirror), video delay, readout time (with its sign), focal scale and mount tilt are right. The
score is the median angle between the two rays. Translation adds error only as
(hand movement / scene distance), so aim the camera at a scene more than 50 m away (drone on its
side), or turn the drone about the lens. Reference: review/hardware_gap/bench_calib.py (H1, H3).

Recording (.npz or dict): imgs (N, rows, cols) uint8 as Hold gets them, t_grab (N,) from the live
stamping code, att (M, 4+) [t, roll, pitch, yaw, ...] mapped by the live clock code, K, D of the
lens profile at this picture size (focal scale 1).
"""
import itertools
import math
import time

import cv2
import numpy as np

from hold import euler_R, make_R_bc

LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
ORIENTATIONS = list(itertools.product(("front", "right", "rear", "left"), (False, True)))
LAT_MAX, RO_MAX = 0.25, 0.04            # search 0-250 ms delay, -40..+40 ms readout (G6)


def track_pairs(imgs, mask=None, max_pts=60, max_age=60, min_alive=25):
    """Keyframe tracks: (keyframe index, frame index, keyframe pixel, frame pixel) per match.

    Frame-to-frame pairs only see the change of rotation rate; matching back to a keyframe gives
    the long baseline a time error needs to show (H1).
    """
    IA, IB, P0, P1 = [], [], [], []
    kf, pk, prev, alive = 0, None, None, None
    for i in range(len(imgs)):
        if pk is None:
            p = cv2.goodFeaturesToTrack(imgs[i], max_pts, 0.01, 7, mask=mask)
            if p is not None:
                kf, pk, prev, alive = i, p.astype(np.float32), p.astype(np.float32), np.ones(len(p), bool)
            continue
        q, s, _ = cv2.calcOpticalFlowPyrLK(imgs[i - 1], imgs[i], prev, None, **LK)
        q2, s2, _ = cv2.calcOpticalFlowPyrLK(imgs[kf], imgs[i], pk, q.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **LK)
        pb, s3, _ = cv2.calcOpticalFlowPyrLK(imgs[i], imgs[kf], q2, pk.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **LK)
        alive &= (s.ravel() == 1) & (s2.ravel() == 1) & (s3.ravel() == 1)
        alive &= np.linalg.norm((pb - pk).reshape(-1, 2), axis=1) < 0.5
        IA.append(np.full(int(alive.sum()), kf))
        IB.append(np.full(int(alive.sum()), i))
        P0.append(pk.reshape(-1, 2)[alive])
        P1.append(q2.reshape(-1, 2)[alive])
        prev = q2
        if alive.sum() < min_alive or i - kf >= max_age:
            pk = None
    if not IA:
        return np.zeros(0, int), np.zeros(0, int), np.zeros((0, 2)), np.zeros((0, 2))
    return np.concatenate(IA), np.concatenate(IB), np.concatenate(P0), np.concatenate(P1)


def nelder_mead(f, x0, step, iters):
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * step[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        o = np.argsort(vals)
        pts, vals = [pts[i] for i in o], [vals[i] for i in o]
        c = np.mean(pts[:-1], axis=0)
        xr = c + (c - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = c + 2 * (c - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (p - pts[0]) for p in pts[1:]]
                vals = [vals[0]] + [f(p) for p in pts[1:]]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


class Rec:
    def __init__(self, rec, mask=None):
        d = np.load(rec) if isinstance(rec, str) else rec
        imgs, tg, att = d["imgs"], np.asarray(d["t_grab"], float), np.asarray(d["att"], float)
        # Keep frames whose whole delay search window has attitude.
        keep = (tg - LAT_MAX - RO_MAX >= att[0, 0]) & (tg + RO_MAX <= att[-1, 0])
        self.imgs, self.tg, self.att = imgs[keep], tg[keep], att
        self.K, self.D = np.asarray(d["K"], float), np.asarray(d["D"], float)
        self.rows = self.imgs.shape[1]
        self.ia, self.ib, self.p0, self.p1 = track_pairs(self.imgs, mask)
        self._yaw = np.unwrap(att[:, 3])
        # The found tilt is camera mount + FC roll/pitch bias together (H8): that is what Hold needs.
        self._cache = {}
        # Rotation rate on a 0.1 s grid: differences of 100 Hz samples would measure attitude noise.
        tt = np.arange(self.tg[0], self.tg[-1], 0.1) if keep.sum() > 1 else np.zeros(0)
        w = np.diff(np.c_[[np.interp(tt, att[:, 0], c) for c in (att[:, 1], att[:, 2], self._yaw)]].T, axis=0) / 0.1
        self.rate_axes_dps = np.degrees(np.sqrt(np.mean(w ** 2, axis=0))) if len(w) else np.zeros(3)
        self.rate_dps = float(np.sqrt(np.sum(self.rate_axes_dps ** 2)))

    def _rays(self, fscale):
        key = round(fscale, 6)
        if key not in self._cache:
            K = self.K.copy()
            K[0, 0] *= fscale
            K[1, 1] *= fscale
            out = []
            for px in (self.p0, self.p1):
                n = cv2.fisheye.undistortPoints(px.astype(np.float64).reshape(-1, 1, 2), K, self.D).reshape(-1, 2)
                f = np.c_[n, np.ones(len(n))]
                out.append(f / np.linalg.norm(f, axis=1, keepdims=True))
            if len(self._cache) > 64:
                self._cache.clear()
            self._cache[key] = out
        return self._cache[key]

    def _R(self, t):
        a = self.att
        return euler_R(np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]), np.interp(t, a[:, 0], self._yaw))

    def residual(self, lat, ro, R_bc, fscale=1.0):
        """Median angle (deg) between the NED rays of each match. Row time = t_grab - lat + row/rows * ro."""
        c0, c1 = self._rays(fscale)
        ta = self.tg[self.ia] - lat + self.p0[:, 1] / self.rows * ro
        tb = self.tg[self.ib] - lat + self.p1[:, 1] / self.rows * ro
        n0 = np.einsum("nij,nj->ni", self._R(ta), c0 @ R_bc.T)
        n1 = np.einsum("nij,nj->ni", self._R(tb), c1 @ R_bc.T)
        return float(np.degrees(np.median(np.linalg.norm(np.cross(n0, n1), axis=1))))

    def timing(self, R_bc, fscale=1.0):
        """(delay, readout, residual): grid 10 ms x 5 ms, then +-8 / +-4 ms in 2 ms steps."""
        lats, ros = np.arange(0, LAT_MAX + 1e-9, 0.01), np.arange(-RO_MAX, RO_MAX + 1e-9, 0.005)
        g = np.array([[self.residual(l, r, R_bc, fscale) for r in ros] for l in lats])
        il, ir = np.unravel_index(np.argmin(g), g.shape)
        lats2, ros2 = lats[il] + np.arange(-0.008, 0.0081, 0.002), ros[ir] + np.arange(-0.004, 0.0041, 0.002)
        g2 = np.array([[self.residual(l, r, R_bc, fscale) for r in ros2] for l in lats2])
        il, ir = np.unravel_index(np.argmin(g2), g2.shape)
        return float(lats2[il]), float(ros2[ir]), float(g2[il, ir])

    def coarse(self):
        """[(residual, top, mirror, delay)] for the 8 orientations, best first (readout 0)."""
        out = []
        for top, mirror in ORIENTATIONS:
            R = make_R_bc(top, mirror)
            out.append(min((self.residual(l, 0.0, R), top, mirror, l) for l in np.arange(0, LAT_MAX + 1e-9, 0.025)))
        return sorted(out)

    def refine(self, lat, ro, top, mirror, x0=(0, 0, 0, 0)):
        """Mount tilt (roll, pitch), twist (deg) and focal scale for one orientation."""
        R = lambda x: make_R_bc(top, mirror, (x[0], x[1]), x[2])      # noqa: E731
        x, v = nelder_mead(lambda x: self.residual(lat, ro, R(x), 1.0 + x[3]), x0, [2.0, 2.0, 2.0, 0.05], 70)
        return dict(tilt_deg=[float(x[0]), float(x[1])], twist_deg=float(x[2]), focal_scale=float(1.0 + x[3]),
                    R_bc=R(x), x=x, resid_deg=v)


def solve(rec, s, mask=None):
    """Everything from one recording. Returns a dict with ok (bool) and why (plain words when not ok)."""
    t_start = time.time()
    r = Rec(rec, mask)
    out = dict(ok=False, n_matches=int(len(r.ia)), rate_dps=round(r.rate_dps, 1),
               rate_axes_dps=[round(float(x), 1) for x in r.rate_axes_dps])
    if r.rate_dps < s["calib_min_rate_dps"] or (r.rate_axes_dps >= 0.3 * s["calib_min_rate_dps"]).sum() < 2:
        out["why"] = (f"the drone turned too little ({r.rate_dps:.0f} deg/s RMS, per axis "
                      f"{', '.join(f'{x:.0f}' for x in r.rate_axes_dps)}); need {s['calib_min_rate_dps']:.0f} deg/s "
                      f"and two axes. Turn it more, about all three axes.")
        return out
    if len(r.ia) < 1000:                   # assumed: the sim gives about 15000 matches in 10 s
        out["why"] = f"only {len(r.ia)} tracked matches; aim the camera at a scene with more texture."
        return out
    coarse = r.coarse()
    _, top, mirror, lat = coarse[0]
    m = r.refine(lat, 0.0, top, mirror)
    lat, ro, _ = r.timing(m["R_bc"], m["focal_scale"])
    m = r.refine(lat, ro, top, mirror, m["x"])
    lat, ro, res = r.timing(m["R_bc"], m["focal_scale"])
    runner = coarse[1]
    out.update(top=top, mirror=mirror, tilt_deg=m["tilt_deg"], twist_deg=m["twist_deg"],
               focal_scale=m["focal_scale"], video_delay_s=lat, readout_s=ro, resid_deg=res,
               runner_up=dict(top=runner[1], mirror=runner[2], resid_deg=runner[0]),
               ratio=runner[0] / max(res, 1e-9), seconds=round(time.time() - t_start, 1))
    if out["ratio"] < s["calib_min_ratio"]:
        out["why"] = (f"orientation not clear: best {top}{' mirrored' if mirror else ''} {res:.2f} deg, next "
                      f"{runner[1]}{' mirrored' if runner[2] else ''} {runner[0]:.2f} deg (need {s['calib_min_ratio']:.0f}x). "
                      "Turn the drone more, toward a far scene.")
    elif res > s["calib_max_resid_deg"]:
        out["why"] = f"residual {res:.2f} deg is over {s['calib_max_resid_deg']} deg; the scene is too near or moving."
    elif not (0.0 < lat < LAT_MAX - 0.005) or abs(ro) > RO_MAX - 0.002:
        out["why"] = f"delay {lat * 1e3:.0f} ms or readout {ro * 1e3:.0f} ms is at the edge of the search; record again."
    else:
        out["ok"] = True
    return out


def sweep(path, R_bc):
    """(latency_s, readout_s) for a known R_bc. latency = t_grab - first-row time, so live t0 = t_grab - latency.

    Refuses (ValueError) when another of the 8 picture orientations fits clearly better than R_bc (H1, H3).
    """
    r = Rec(path)
    lat, ro, res = r.timing(R_bc)
    best = r.coarse()[0]
    if best[0] * 5 < res:
        raise ValueError(f"the picture orientation is not the configured one: {best[1]}"
                         f"{' mirrored' if best[2] else ''} fits {best[0]:.2f} deg against {res:.2f} deg. Run calib.")
    return lat, ro


def write(s, result, capture_mode):
    """Copy an accepted result into the settings dict and mark it calibrated."""
    prev = s.get("calib_info") or {}
    for k in ("top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s", "video_delay_s"):
        s[k] = result[k]
    s["calibrated"], s["calib_capture"] = True, capture_mode
    s["calib_info"] = {k: result[k] for k in ("resid_deg", "ratio", "runner_up", "rate_dps", "n_matches")}
    s["calib_info"]["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
    s["calib_info"]["previous_delay_s"] = prev.get("delay_s")
    s["calib_info"]["delay_s"] = result["video_delay_s"]
    return s
