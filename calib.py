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

fit_lens (the digital camera, square pixels): fx and fy separately instead of one focal scale, and
k1 when the tracked points reach K1_MIN_DEG off axis. A grid in picture units gives the start, so
the start profile does not set the result. write() stores the lens in settings as lens_measured.
"""
import copy
import itertools
import json
import math
import os
import time

import cv2
import numpy as np

from hold import euler_R, make_R_bc, same_aspect
from settings import LENS_KEYS

LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
ORIENTATIONS = list(itertools.product(("front", "right", "rear", "left"), (False, True)))
LAT_MAX, RO_MAX = 0.25, 0.04            # search 0-250 ms delay, -40..+40 ms readout (G6)
# fit_lens grid: fx / (w / 2) from 0.6 to 1.5 (equidistant 191 to 76 deg wide) and fy / fx from 0.65 to
# 1.5, in 5 % steps. The three assumed Justice D profiles (0.75 and 1.0; fy / fx 0.75 and 1) are inside.
FOCAL_GRID = np.exp(np.arange(math.log(0.6), math.log(1.5), 0.05))
ASPECT_GRID = np.exp(np.arange(math.log(0.65), math.log(1.5), 0.05))
COVER_MIN_DEG = 20.0      # assumed: fx and fy need points this far off axis; not tested below 40 deg
K1_MIN_DEG = 40.0         # assumed: at 40 deg off axis, k1 = -0.025 (Justice A) moves a point 1.2 % of its radius
# assumed: an equidistant fit of a fisheye gives |k1| under this (equisolid -0.042, stereographic +0.083).
# A fit outside it, or outside the grid, is not trusted (D3).
K1_MAX = 0.15
# The lens search uses this many matches, drawn with a fixed seed; the final residual uses all (D4: about 4000
# residuals). assumed: enough; test_lens_d.py keeps fx and fy within 1 % on synthetic pictures.
FIT_MATCHES = 5000


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

    def sample(self, n, seed=0):
        """This recording with at most n of its matches, drawn with a fixed seed: a search then costs n, not all."""
        r = copy.copy(self)
        if len(self.ia) > n:
            i = np.sort(np.random.default_rng(seed).choice(len(self.ia), n, replace=False))
            r.ia, r.ib, r.p0, r.p1 = self.ia[i], self.ib[i], self.p0[i], self.p1[i]
        r._cache = {}
        return r

    def _rays(self, fscale, lens=None):
        """Unit camera rays of p0 and p1. lens: None, or (fx, fy, k1) in place of K's focal lengths and D[0]."""
        key = round(fscale, 6) if lens is None else tuple(round(float(v), 9) for v in lens)
        if key not in self._cache:
            K, D = self.K.copy(), self.D.copy()
            if lens is None:
                K[0, 0] *= fscale
                K[1, 1] *= fscale
            else:
                K[0, 0], K[1, 1], D[0] = lens
            out = []
            for px in (self.p0, self.p1):
                n = cv2.fisheye.undistortPoints(px.astype(np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)
                f = np.c_[n, np.ones(len(n))]
                out.append(f / np.linalg.norm(f, axis=1, keepdims=True))
            if len(self._cache) > 64:
                self._cache.clear()
            self._cache[key] = out
        return self._cache[key]

    def _R(self, t):
        a = self.att
        return euler_R(np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]), np.interp(t, a[:, 0], self._yaw))

    def residual(self, lat, ro, R_bc, fscale=1.0, lens=None):
        """Median angle (deg) between the NED rays of each match. Row time = t_grab - lat + row/rows * ro."""
        c0, c1 = self._rays(fscale, lens)
        ta = self.tg[self.ia] - lat + self.p0[:, 1] / self.rows * ro
        tb = self.tg[self.ib] - lat + self.p1[:, 1] / self.rows * ro
        n0 = np.einsum("nij,nj->ni", self._R(ta), c0 @ R_bc.T)
        n1 = np.einsum("nij,nj->ni", self._R(tb), c1 @ R_bc.T)
        return float(np.degrees(np.median(np.linalg.norm(np.cross(n0, n1), axis=1))))

    def timing(self, R_bc, fscale=1.0, lens=None):
        """(delay, readout, residual): grid 10 ms x 5 ms, then +-8 / +-4 ms in 2 ms steps."""
        lats, ros = np.arange(0, LAT_MAX + 1e-9, 0.01), np.arange(-RO_MAX, RO_MAX + 1e-9, 0.005)
        g = np.array([[self.residual(l, r, R_bc, fscale, lens) for r in ros] for l in lats])
        il, ir = np.unravel_index(np.argmin(g), g.shape)
        lats2, ros2 = lats[il] + np.arange(-0.008, 0.0081, 0.002), ros[ir] + np.arange(-0.004, 0.0041, 0.002)
        g2 = np.array([[self.residual(l, r, R_bc, fscale, lens) for r in ros2] for l in lats2])
        il, ir = np.unravel_index(np.argmin(g2), g2.shape)
        return float(lats2[il]), float(ros2[ir]), float(g2[il, ir])

    def coarse(self, lens=None):
        """[(residual, top, mirror, delay)] for the 8 orientations, best first (readout 0)."""
        out = []
        for top, mirror in ORIENTATIONS:
            R = make_R_bc(top, mirror)
            out.append(min((self.residual(l, 0.0, R, lens=lens), top, mirror, l) for l in np.arange(0, LAT_MAX + 1e-9, 0.025)))
        return sorted(out)

    def cam_rates(self, R_bc):
        """RMS turn rate (deg/s) about the picture's x, y and z axes. fx needs turns about y, fy about x."""
        R = self._R(np.arange(self.tg[0], self.tg[-1], 0.1))
        d = np.einsum("nji,njk->nik", R[:-1], R[1:])            # body turn in each 0.1 s step
        w = np.c_[d[:, 2, 1] - d[:, 1, 2], d[:, 0, 2] - d[:, 2, 0], d[:, 1, 0] - d[:, 0, 1]] / 0.2
        return np.degrees(np.sqrt(np.mean((w @ R_bc) ** 2, axis=0)))

    def coverage(self, lens):
        """(points, corner) in deg off the optical axis: 99th percentile of the tracked points, picture corner."""
        z = np.concatenate(self._rays(1.0, lens))[:, 2]
        K, D = self.K.copy(), self.D.copy()
        K[0, 0], K[1, 1], D[0] = lens
        h, w = self.imgs.shape[1:3]
        c = cv2.fisheye.undistortPoints(np.array([[[-0.5, -0.5]], [[w - 0.5, -0.5]], [[-0.5, h - 0.5]], [[w - 0.5, h - 0.5]]]), K, D)
        return float(np.degrees(np.arccos(np.percentile(z, 1)))), float(np.degrees(np.arctan(np.hypot(*c.reshape(-1, 2).T)).max()))

    def lens_grid(self, lat, R_bc):
        """(fx, fy, k1, edge) of the best grid lens at readout 0; edge: it is on the first or last grid value.
        The grid is in picture units: only k1 comes from the start."""
        hw = self.imgs.shape[2] / 2
        _, i, j = min((self.residual(lat, 0.0, R_bc, lens=(a * hw, a * hw * b, self.D[0])), i, j)
                      for i, a in enumerate(FOCAL_GRID) for j, b in enumerate(ASPECT_GRID))
        a, b = FOCAL_GRID[i], ASPECT_GRID[j]
        edge = i in (0, len(FOCAL_GRID) - 1) or j in (0, len(ASPECT_GRID) - 1)
        return float(a * hw), float(a * hw * b), float(self.D[0]), edge

    def refine_lens(self, lat, ro, top, mirror, lens, fit_k1, x0=(0, 0, 0)):
        """Mount tilt (roll, pitch), twist (deg), fx, fy and, with fit_k1, k1 for one orientation."""
        R = lambda x: make_R_bc(top, mirror, (x[0], x[1]), x[2])                                         # noqa: E731
        L = lambda x: (lens[0] * (1 + x[3]), lens[1] * (1 + x[4]), lens[2] + (x[5] if fit_k1 else 0.0))   # noqa: E731
        n = 6 if fit_k1 else 5
        x, v = nelder_mead(lambda x: self.residual(lat, ro, R(x), lens=L(x)), list(x0) + [0.0] * (n - 3),
                           [2.0, 2.0, 2.0, 0.03, 0.03, 0.02][:n], 60 * n)
        return dict(tilt_deg=[float(x[0]), float(x[1])], twist_deg=float(x[2]), focal_scale=1.0,
                    lens=tuple(float(v) for v in L(x)), R_bc=R(x), x=x[:3], resid_deg=v)

    def refine(self, lat, ro, top, mirror, x0=(0, 0, 0, 0)):
        """Mount tilt (roll, pitch), twist (deg) and focal scale for one orientation."""
        R = lambda x: make_R_bc(top, mirror, (x[0], x[1]), x[2])      # noqa: E731
        x, v = nelder_mead(lambda x: self.residual(lat, ro, R(x), 1.0 + x[3]), x0, [2.0, 2.0, 2.0, 0.05], 70)
        return dict(tilt_deg=[float(x[0]), float(x[1])], twist_deg=float(x[2]), focal_scale=float(1.0 + x[3]),
                    R_bc=R(x), x=x, resid_deg=v)


def _fit_lens(r, s, out):
    """solve() steps with fit_lens. Returns (coarse, top, mirror, m, delay, readout, residual), or None and out['why']."""
    q = r.sample(FIT_MATCHES)     # the search runs on q; the final residual uses all matches (D4)
    lens = (float(r.K[0, 0]), float(r.K[1, 1]), float(r.D[0]))
    for _ in range(2):            # orientation with the start lens, then the grid; again with the grid lens
        _, top, mirror, lat = q.coarse(lens)[0]
        *lens, edge = q.lens_grid(lat, make_R_bc(top, mirror))
    lens = tuple(lens)
    cam, need = r.cam_rates(make_R_bc(top, mirror)), 0.3 * s["calib_min_rate_dps"]
    cover = r.coverage(lens)[0]
    if min(cam[0], cam[1]) < need:
        out["why"] = (f"the drone turned too little about the picture's axes: {cam[0]:.0f} deg/s about x, {cam[1]:.0f} deg/s "
                      f"about y. Each axis needs {need:.0f} deg/s to find fx and fy. Turn the drone more, about all three axes.")
        return None
    if cover < COVER_MIN_DEG:
        out["why"] = (f"the tracked points reach only {cover:.0f} deg from the optical axis. fx and fy need "
                      f"{COVER_MIN_DEG:.0f} deg. Aim the camera at a scene with texture out to the picture edges.")
        return None
    fit_k1 = cover >= K1_MIN_DEG
    m = q.refine_lens(lat, 0.0, top, mirror, lens, fit_k1)
    for _ in range(2):
        lat, ro, _ = q.timing(m["R_bc"], lens=m["lens"])
        m = q.refine_lens(lat, ro, top, mirror, m["lens"], fit_k1, m["x"])
    lat, ro, _ = q.timing(m["R_bc"], lens=m["lens"])
    res = r.residual(lat, ro, m["R_bc"], lens=m["lens"])
    # Runner-up with the found lens; the found orientation goes first.
    coarse = sorted(q.coarse(m["lens"]), key=lambda c: (c[1], c[2]) != (top, mirror))
    cover, corner = r.coverage(m["lens"])
    h, w = r.imgs.shape[1:3]
    # D3: a lens on the grid's edge or outside the search range is not trusted (no bounds in Nelder-Mead).
    a, b, k1 = m["lens"][0] / (w / 2), m["lens"][1] / m["lens"][0], m["lens"][2]
    if edge or not (FOCAL_GRID[0] < a < FOCAL_GRID[-1] and ASPECT_GRID[0] < b < ASPECT_GRID[-1] and abs(k1) < K1_MAX):
        out["lens_edge"] = (f"the lens fit ended at the edge of its search range: fx / (w / 2) {a:.3f} (range "
                            f"{FOCAL_GRID[0]:.2f} to {FOCAL_GRID[-1]:.2f}), fy / fx {b:.3f} ({ASPECT_GRID[0]:.2f} to "
                            f"{ASPECT_GRID[-1]:.2f}), k1 {k1:+.4f} (-{K1_MAX} to +{K1_MAX}). Record again. If it fails "
                            "again, measure the lens with a checkerboard (tools/lens_calib.py --settings). Then run calib again.")
    # The centre and k2-k4 are not fitted: they stay as in the start profile.
    out.update(lens=dict(zip(("fx", "fy", "k1"), m["lens"]), cx=float(r.K[0, 2]), cy=float(r.K[1, 2]),
                         **{f"k{i + 1}": float(r.D[i]) for i in (1, 2, 3)}, w=w, h=h), k1_fitted=fit_k1, cover_deg=round(cover, 1),
               corner_deg=round(corner, 1), cam_rate_dps=[round(float(x), 1) for x in cam])
    if not fit_k1:
        out["k1_why"] = (f"the tracked points reach only {cover:.0f} deg from the optical axis. k1 needs {K1_MIN_DEG:.0f} "
                         f"deg. The measured lens keeps k1 {m['lens'][2]:+.4f} from the start profile. To find k1, aim the "
                         "camera at a scene with texture out to the picture corners and record again.")
    return coarse, top, mirror, m, lat, ro, res


def solve(rec, s, mask=None, fit_lens=False):
    """Everything from one recording. Returns a dict with ok (bool) and why (plain words when not ok).

    fit_lens: fx, fy and k1 as well (result 'lens' at the picture size, focal_scale 1, quality numbers).
    """
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
    if fit_lens:
        f = _fit_lens(r, s, out)
        if f is None:
            return out
        coarse, top, mirror, m, lat, ro, res = f
    else:
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
    if out.get("lens_edge"):               # first: a wrong lens spoils the tests below too
        out["why"] = out["lens_edge"]
    elif out["ratio"] < s["calib_min_ratio"]:
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


def profile(path, name):
    """The named profile (a dict) of a lens file."""
    p = next((m for m in json.load(open(path))["models"] if m["name"] == name), None)
    if p is None:
        raise ValueError(f"lens profile '{name}' is not in {path}. Set lens_name to a profile in that file.")
    return p


def add_profile(path, p):
    """Append profile p to the lens file through a temporary file and a rename. Refuses a name that exists."""
    d = json.load(open(path))
    if any(m["name"] == p["name"] for m in d["models"]):
        raise ValueError(f"lens profile '{p['name']}' exists in {path}. Pick another name.")
    d["models"].append(p)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def measured_lens(m, w, h):
    """K, D of a lens_measured dict for a w x h picture (whole frames, as hold.load_lens with field None).
    Refuses (ValueError) a picture of another shape: scaling would stretch the lens with no error."""
    pw, ph = m["image_size"]
    if not same_aspect((pw, ph), w, h):
        raise ValueError(f"lens_measured ({m['by']}) is for {pw}x{ph} pictures, but the picture is {w}x{h}. The shapes "
                         "differ. Set lens_measured to null in settings.json, then run calib for this camera.")
    sx, sy = w / pw, h / ph
    K = np.array([[m["fx"] * sx, 0, (m["cx"] + 0.5) * sx - 0.5], [0, m["fy"] * sy, (m["cy"] + 0.5) * sy - 0.5], [0, 0, 1.]])
    return K, np.array([m["k1"], m["k2"], m["k3"], m["k4"]], float)


def write(s, result, capture_mode):
    """Copy an accepted result into the settings dict and mark it calibrated.

    A fit_lens result becomes lens_measured ("hand turn"), with focal_scale 1; lens_name stays the start profile.
    A lens measured with a target stays, and focal_scale is measured against it. Otherwise focal_scale is measured
    against the lens_name profile, so lens_measured is cleared.
    """
    if "lens" in result:
        L = result["lens"]
        s["lens_measured"] = dict(by="hand turn", image_size=[int(L["w"]), int(L["h"])], **{k: float(L[k]) for k in LENS_KEYS},
                                  note=f"calib.py {time.strftime('%Y-%m-%d %H:%M')} from '{s['lens_name']}': residual "
                                       f"{result['resid_deg']:.3f} deg, points to {result['cover_deg']:.0f} deg off axis, k1 "
                                       f"{'fitted' if result['k1_fitted'] else 'kept'}; centre and k2-k4 kept")
    elif (s.get("lens_measured") or {}).get("by") != "target":
        s["lens_measured"] = None
    prev = s.get("calib_info") or {}
    for k in ("top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s", "video_delay_s"):
        s[k] = result[k]
    s["calibrated"], s["calib_capture"] = True, capture_mode
    s["calib_info"] = {k: result[k] for k in ("resid_deg", "ratio", "runner_up", "rate_dps", "n_matches")}
    s["calib_info"]["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
    s["calib_info"]["previous_delay_s"] = prev.get("delay_s")
    s["calib_info"]["delay_s"] = result["video_delay_s"]
    return s
