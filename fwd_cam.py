#!/usr/bin/env python3
"""The forward camera (Orqa Justice A, analog, about 114 x 86 deg) as an attitude reference. PROTOTYPE.

  find_horizon(img, lens)   roll and pitch of the horizon line in the camera frame, a quality value,
                            or ok=False ("no horizon") when the fit is poor. It refuses; it does not guess.
  heading_change(...)       the yaw change that remains after the gyro's attitude change, from picture points
                            far away (near or above the horizon) in two pictures.
  FwdRef                    takes a reference at one moment (the pilot's switch), then gives the change of roll,
                            pitch and yaw since that moment, with a quality value.

The mount tilt (0 to 30 deg up) is not known, so this camera cannot give the absolute pitch. It gives the change
since the reference. FwdRef.set_reference can take the FC's roll and pitch at the switch: it then solves the mount
tilt and mount roll from them. An error in them changes the measured change only in second order.

Angles are radians in the code. Camera frame: x right, y down, z forward (OpenCV). Body frame: FRD. World: NED.
Attitude: ZYX Euler (roll, pitch, yaw), as MAVLink ATTITUDE. No import from hold.py.
Tested on synthetic pictures only (tests_apb/test_fwd_cam.py, integ/fwdcam/). No real Justice A picture yet.

Method sources. Sky/ground split and a horizon line fit: S. Ettinger, M. Nechyba, P. Ifju, M. Waszak, "Vision-guided
flight stability and control for micro air vehicles", Advanced Robotics 17(7), 2003. The horizon as a great circle on
the view sphere of a wide lens: S. Thurrowgood et al., "A vision based system for attitude estimation of UAVs", IROS
2009. Rotation from pairs of unit vectors (Wahba's problem) by SVD: F. L. Markley, "Attitude determination using
vector observations and the singular value decomposition", J. Astronautical Sciences 36(3), 1988. Point tracking:
J.-Y. Bouguet, "Pyramidal implementation of the Lucas Kanade feature tracker", Intel, 2000 (cv2.calcOpticalFlowPyrLK).
"""
import json
import math
import os
from collections import namedtuple

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LENS_FILE = os.path.join(HERE, "justice_lens_profiles.json")
FWD_LENS = "Round 1 independent k2"     # the Justice 4:3 analog profile (the program's default analog lens)
UP_NED = np.array([0.0, 0.0, -1.0])


# ---------------------------------------------------------------- lens
class Lens:
    """Equidistant fisheye (the cv2.fisheye model: theta_d = theta (1 + k1 theta^2 + ... + k4 theta^8)) for one
    picture size. The picture can have non-square pixels (one video field: half the rows)."""

    def __init__(self, K, D, w, h):
        self.K, self.D = np.asarray(K, float), np.asarray(D, float).reshape(4)
        self.w, self.h = int(w), int(h)
        self._grid = None

    @classmethod
    def profile(cls, w, h, name=FWD_LENS, path=LENS_FILE):
        """A profile of justice_lens_profiles.json scaled to a w x h picture of the same extent
        (w x h/2 for one field). Pixel centres are integers: (u + 0.5) scales with the picture."""
        p = next((m for m in json.load(open(path))["models"] if m["name"] == name), None)
        if p is None or p.get("projection") != "equidistant_fisheye":
            raise ValueError(f"lens profile '{name}' is not an equidistant_fisheye profile in {path}")
        pw, ph = p["image_size"]
        sx, sy = w / pw, h / ph
        K = [[p["fx"] * sx, 0.0, (p["cx"] + 0.5) * sx - 0.5], [0.0, p["fy"] * sy, (p["cy"] + 0.5) * sy - 0.5],
             [0.0, 0.0, 1.0]]
        return cls(K, [p.get(k, 0.0) for k in ("k1", "k2", "k3", "k4")], w, h)

    def scaled(self, s):
        """The same lens for the picture resized by s (both axes)."""
        K = self.K.copy()
        K[0, 0] *= s
        K[1, 1] *= s
        K[0, 2] = (K[0, 2] + 0.5) * s - 0.5
        K[1, 2] = (K[1, 2] + 0.5) * s - 0.5
        return Lens(K, self.D, round(self.w * s), round(self.h * s))

    def rays(self, uv):
        """Unit rays (camera frame) of pixels uv (..., 2)."""
        uv = np.asarray(uv, np.float64)
        xy = cv2.fisheye.undistortPoints(uv.reshape(-1, 1, 2), self.K, self.D,
                                         criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-10))
        r = np.concatenate([xy.reshape(-1, 2), np.ones((xy.shape[0], 1))], 1)
        return (r / np.linalg.norm(r, axis=1, keepdims=True)).reshape(uv.shape[:-1] + (3,))

    def project(self, r):
        """Pixels (..., 2) of camera-frame directions r (..., 3); also valid for directions behind the lens plane."""
        r = np.asarray(r, float)
        x, y, z = r[..., 0], r[..., 1], r[..., 2]
        rho = np.hypot(x, y)
        th = np.arctan2(rho, z)
        t2 = th * th
        td = th * (1 + self.D[0] * t2 + self.D[1] * t2 ** 2 + self.D[2] * t2 ** 3 + self.D[3] * t2 ** 4)
        k = np.where(rho > 1e-12, td / np.maximum(rho, 1e-12), 1.0)
        return np.stack([self.K[0, 0] * k * x + self.K[0, 2], self.K[1, 1] * k * y + self.K[1, 2]], -1)

    def grid(self):
        """Rays of every pixel (h, w, 3), computed once."""
        if self._grid is None:
            uu, vv = np.meshgrid(np.arange(self.w, dtype=float), np.arange(self.h, dtype=float))
            self._grid = self.rays(np.stack([uu, vv], -1)).astype(np.float32)
        return self._grid


# ---------------------------------------------------------------- frames and angles
def euler_to_R(roll, pitch, yaw):
    """Body FRD -> NED, ZYX (MAVLink ATTITUDE)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_euler(R):
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def mount_R(alpha, rho=0.0):
    """Camera -> body for a forward camera tilted UP by alpha (about the body right axis) and turned by rho about
    its own optical axis. Columns: camera x, y, z in body FRD. alpha 0: z forward, x right, y down."""
    B = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    ca, sa, cr, sr = math.cos(alpha), math.sin(alpha), math.cos(rho), math.sin(rho)
    Ry = np.array([[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]])
    Rz = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    return Ry @ B @ Rz


def tilt_from_up(u_b):
    """(roll, pitch) of the body from the world-up direction in body axes."""
    return math.atan2(-u_b[1], -u_b[2]), math.asin(max(-1.0, min(1.0, u_b[0])))


def solve_mount(n_c, roll, pitch):
    """(alpha, rho) so that mount_R(alpha, rho) @ n_c is the world up of a body at (roll, pitch). n_c: world up in
    camera axes (the horizon normal). The mount yaw is not visible in the horizon; it is taken as 0."""
    u = euler_to_R(roll, pitch, 0.0).T @ UP_NED
    r, beta = math.hypot(n_c[0], n_c[1]), math.atan2(n_c[1], n_c[0])
    a = math.acos(max(-1.0, min(1.0, u[1] / max(r, 1e-9))))
    rho = min((wrap(a - beta), wrap(-a - beta)), key=abs)
    cr, sr = math.cos(rho), math.sin(rho)
    v = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]) @ np.array(
        [cr * n_c[0] - sr * n_c[1], sr * n_c[0] + cr * n_c[1], n_c[2]])
    return wrap(math.atan2(u[0], u[2]) - math.atan2(v[0], v[2])), rho


# ---------------------------------------------------------------- horizon
# Thresholds: ASSUMED, NOT TUNED (no evaluation ran on 2026-11-21). Tune them on seeds 1-3 only
# (integ/fwdcam/run_eval.py tuning), then run the report seeds 4-6 once and do not change them after that.
HZ = dict(work_w=320,       # px; the finder works on the picture resized to this width
          min_side=6,       # px; each side of a column split has at least this many rows
          col_step=2,       # px; every second column
          min_valid=0.7,    # a column with more of its rows masked than 1 - this is skipped
          grad_min=1.5,     # grey/px at work size; a weaker sky/ground step is no edge point
          ransac_it=150,
          inl_deg=0.5,      # deg; an edge point this close to the great circle is an inlier
          min_inl=20,       # edge points
          min_frac=0.5,     # inliers / columns with an edge point
          min_span_deg=40,  # deg of horizon covered by inliers
          max_rms_deg=0.25,  # deg; RMS distance of the inliers to the circle
          band_deg=8.0,     # deg; the region check compares the bands this wide above and below the line
          min_contrast=1.0,  # |mean sky - mean ground| / sqrt(var sky + var ground) in those bands
          max_sky_tex=0.6,  # texture above the line / texture below: sky is smoother than ground
          min_area=0.04)    # each band covers at least this share of the picture
Horizon = namedtuple("Horizon", "ok n roll pitch quality why rms_deg span_deg frac contrast sky_tex n_inl uv")


def _cache(obj, key, fn):
    c = obj.__dict__.setdefault("_fwd_cache", {})
    if key not in c:
        c[key] = fn()
    return c[key]


def _work(img, lens, work_w):
    """The picture resized to work_w wide, the scale, and the lens for that size (kept, so its ray grid is kept)."""
    if img.shape[:2] != (lens.h, lens.w):
        raise ValueError(f"picture {img.shape[1]}x{img.shape[0]} but lens {lens.w}x{lens.h}: use Lens.profile(w, h)")
    s = min(1.0, work_w / img.shape[1])
    if s >= 1.0:
        return img, 1.0, lens
    g = cv2.resize(img, (round(img.shape[1] * s), round(img.shape[0] * s)), interpolation=cv2.INTER_AREA)
    return g, s, _cache(lens, ("scaled", round(s, 9)), lambda: lens.scaled(s))


def find_horizon(img, lens, mask=None, cfg=None):
    """The horizon in one grey picture. img: uint8 (h, w) at the lens's size. mask: uint8, 0 = ignore (OSD text).
    Returns Horizon: ok, n (world up in camera axes, unit), roll (camera turned right-side-down is +), pitch (optical
    axis above the horizon is +), quality 0-1, why (None when ok), and the fit numbers."""
    c = dict(HZ, **(cfg or {}))
    g8, s, wl = _work(img, lens, c["work_w"])
    h, w = g8.shape
    g = cv2.GaussianBlur(g8.astype(np.float32), (0, 0), 1.0)
    valid = np.ones((h, w), bool) if mask is None else \
        cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    tex = cv2.GaussianBlur(np.sqrt(gx * gx + gy * gy), (0, 0), 2.0)
    no = lambda why: Horizon(False, None, None, None, 0.0, why, None, None, None, None, None, 0, None)  # noqa: E731

    # 1. each column: the split row that best separates the top (sky) from the bottom (ground): Fisher score of
    #    grey and texture.
    cols = np.arange(2, w - 2, c["col_step"])
    W = valid[:, cols].astype(np.float64)
    keep = W.mean(0) >= c["min_valid"]
    cols, W = cols[keep], W[:, keep]
    if len(cols) < c["min_inl"]:
        return no("too few usable columns (mask)")
    score = 0.0
    m_sign = None
    for F in (g[:, cols].astype(np.float64), tex[:, cols].astype(np.float64)):
        cw, cs, cq = np.cumsum(W, 0), np.cumsum(W * F, 0), np.cumsum(W * F * F, 0)
        n1, s1, q1 = cw[:-1], cs[:-1], cq[:-1]                         # rows above split y = 1 .. h-1
        n2, s2, q2 = cw[-1] - n1, cs[-1] - s1, cq[-1] - q1
        n1c, n2c = np.maximum(n1, 1), np.maximum(n2, 1)
        m1, m2 = s1 / n1c, s2 / n2c
        v1, v2 = np.maximum(q1 / n1c - m1 ** 2, 0), np.maximum(q2 / n2c - m2 ** 2, 0)
        score = score + (m1 - m2) ** 2 / (v1 + v2 + 1.0)
        if m_sign is None:
            m_sign = np.sign(m1 - m2)
    ys = np.arange(1, h)
    ok_rows = (ys >= c["min_side"]) & (ys <= h - c["min_side"])
    score = np.where(ok_rows[:, None], score, -1)
    yb = np.argmax(score, 0) + 1                                      # split: rows < yb above
    sgn = m_sign[yb - 1, np.arange(len(cols))]

    # 2. sub-pixel edge: strongest grey step of the right sign within 3 rows of the split
    d = np.zeros_like(g)
    d[1:-1] = (g[2:] - g[:-2]) / 2.0
    edge_v, edge_u, edge_s = [], [], []
    for j, x in enumerate(cols):
        lo, hi = max(1, yb[j] - 4), min(h - 2, yb[j] + 3)
        if hi - lo < 3:
            continue
        prof = -sgn[j] * d[lo:hi + 1, x]
        k = int(np.argmax(prof))
        if prof[k] < c["grad_min"] or not valid[lo + k, x]:
            continue
        off = 0.0
        if 0 < k < len(prof) - 1:
            a, b, cc = prof[k - 1], prof[k], prof[k + 1]
            den = a - 2 * b + cc
            off = 0.5 * (a - cc) / den if den < 0 else 0.0
        edge_v.append(lo + k + off)
        edge_u.append(x)
        edge_s.append(prof[k])
    if len(edge_u) < c["min_inl"]:
        return no(f"no sky/ground edge ({len(edge_u)} columns)")
    uv = np.c_[edge_u, edge_v].astype(float)
    r = wl.rays(uv)

    # 3. great circle (plane through the lens centre) by RANSAC, then least squares on the inliers
    rng = np.random.default_rng(0)
    thr = math.sin(math.radians(c["inl_deg"]))
    best, best_n = None, None
    m = len(r)
    for _ in range(c["ransac_it"]):
        i, j = rng.choice(m, 2, replace=False)
        if abs(uv[i, 0] - uv[j, 0]) < w / 5:
            continue
        nn = np.cross(r[i], r[j])
        nn /= np.linalg.norm(nn)
        inl = np.abs(r @ nn) < thr
        if best is None or inl.sum() > best.sum():
            best, best_n = inl, nn
    if best is None:
        return no("no line fit")
    nn = best_n
    for _ in range(2):
        inl = np.abs(r @ nn) < thr
        if inl.sum() < 3:
            break
        A = r[inl].T @ r[inl]
        nn = np.linalg.eigh(A)[1][:, 0]
    inl = np.abs(r @ nn) < thr
    if nn[1] > 0:                                                   # world up points to the picture's top
        nn = -nn
    n_inl = int(inl.sum())
    res = np.degrees(np.arcsin(np.clip(r[inl] @ nn, -1, 1)))
    rms = float(np.sqrt(np.mean(res ** 2))) if n_inl else 99.0
    ri = r[inl]
    span = 0.0
    if n_inl >= 2:
        a0, a1 = np.argmin(uv[inl, 0]), np.argmax(uv[inl, 0])
        span = math.degrees(math.acos(max(-1.0, min(1.0, float(ri[a0] @ ri[a1])))))
    frac = n_inl / m

    # 4. region check: the band above the line must differ from the band below, and be smoother
    G = _cache(wl, "grid", wl.grid)
    e = np.degrees(np.arcsin(np.clip(G @ nn.astype(np.float32), -1, 1)))
    A_ = valid & (e > 1.0) & (e < 1.0 + c["band_deg"])
    B_ = valid & (e < -1.0) & (e > -1.0 - c["band_deg"])
    area = min(A_.mean(), B_.mean())
    contrast, sky_tex = 0.0, 9.9
    if A_.any() and B_.any():
        ga, gb = g[A_], g[B_]
        contrast = float(abs(ga.mean() - gb.mean()) / math.sqrt(ga.var() + gb.var() + 1.0))
        sky_tex = float(tex[A_].mean() / (tex[B_].mean() + 1e-3))
    roll, pitch = math.atan2(-nn[0], -nn[1]), math.asin(max(-1.0, min(1.0, float(nn[2]))))
    q = (min(1.0, frac / 0.9) * min(1.0, span / 80.0) * max(0.0, 1.0 - rms / c["max_rms_deg"]) *
         min(1.0, contrast / (2 * c["min_contrast"])))
    why = None
    if n_inl < c["min_inl"]:
        why = f"too few points on one line ({n_inl})"
    elif frac < c["min_frac"]:
        why = f"edge points do not lie on one line ({frac:.0%})"
    elif span < c["min_span_deg"]:
        why = f"line too short ({span:.0f} deg)"
    elif rms > c["max_rms_deg"]:
        why = f"line too rough ({rms:.2f} deg RMS)"
    elif area < c["min_area"]:
        why = "horizon at the picture edge"
    elif contrast < c["min_contrast"]:
        why = f"low sky/ground contrast ({contrast:.2f})"
    elif sky_tex > c["max_sky_tex"]:
        why = f"no smooth sky above the line ({sky_tex:.2f})"
    return Horizon(why is None, nn, roll, pitch, q if why is None else 0.0, why, rms, span, frac, contrast, sky_tex,
                   n_inl, (uv[inl] + 0.5) / s - 0.5)


# ---------------------------------------------------------------- far points: rotation between two pictures
FP = dict(max_pts=200, min_dist=6, quality=0.01, win=15, levels=3, fb_px=0.5, inl_deg=0.3, min_inl=12,
          ransac_it=100, still_px=0.3, top_deg=45.0)
Rot = namedtuple("Rot", "ok M n_inl rms_deg spread_deg quality why")


def yaw_band_deg(height_m, yaw_tol_deg_per_m=0.15, lo=0.75, hi=10.0):
    """How far below the horizon a ground point may be used for the rotation. A ground point at depression b seen
    from height h moves by (sideways drift) * tan(b) / h rad when the aircraft drifts. The band keeps that under
    yaw_tol_deg_per_m for each metre of drift. Assumed (geometry, not tuned): 0.15 deg per m of drift."""
    if height_m is None or height_m <= 0:
        return 3.0
    return float(np.clip(math.degrees(math.atan(height_m * math.radians(yaw_tol_deg_per_m))), lo, hi))


def far_points(g8, wl, n_up, band_deg, valid=None, cfg=None):
    """Corners (work-size pixels, float32 (N, 1, 2)) from band_deg below the horizon up to top_deg above it."""
    c = dict(FP, **(cfg or {}))
    G = _cache(wl, "grid", wl.grid)
    e = np.degrees(np.arcsin(np.clip(G @ np.asarray(n_up, np.float32), -1, 1)))
    m = (e > -band_deg) & (e < c["top_deg"])
    if valid is not None:
        m &= valid
    m8 = cv2.erode(m.astype(np.uint8) * 255, np.ones((5, 5), np.uint8))
    pts = cv2.goodFeaturesToTrack(g8, c["max_pts"], c["quality"], c["min_dist"], mask=m8, blockSize=5)
    return np.zeros((0, 1, 2), np.float32) if pts is None else pts.astype(np.float32)


def wahba(r0, r1):
    """M (proper rotation) with r1 ~ M r0 in least squares (Markley 1988)."""
    U, _, Vt = np.linalg.svd(r1.T @ r0)
    return U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt


def track_rotation(g0, pts0, g1, wl, M_pred, cfg=None, rng_seed=0):
    """Camera rotation M (r1 = M r0) from picture g0 (points pts0) to picture g1, both work size uint8.
    M_pred: the rotation the gyro predicts; LK starts from the predicted pixels."""
    c = dict(FP, **(cfg or {}))
    if len(pts0) < c["min_inl"]:
        return Rot(False, None, 0, None, None, 0.0, f"too few far points ({len(pts0)})"), None
    r0 = wl.rays(pts0.reshape(-1, 2))
    pred = wl.project(r0 @ M_pred.T).astype(np.float32).reshape(-1, 1, 2)
    # Warp the key picture by the predicted rotation into the current view (pixel -> ray -> M_pred^T -> key pixel),
    # so LK sees patches with the same roll and the same lens distortion, and only the small rest of the turn.
    G = _cache(wl, "grid", wl.grid)
    src = wl.project(G @ M_pred.astype(np.float32)).astype(np.float32)
    warped = cv2.remap(g0, src[..., 0], src[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    lk = dict(winSize=(c["win"], c["win"]), maxLevel=c["levels"],
              criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01), flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(warped, g1, pred, pred.copy(), **lk)
    pwb, st2, _ = cv2.calcOpticalFlowPyrLK(g1, warped, p1, pred.copy(), **lk)
    p0, p1, pwb, pr = pts0.reshape(-1, 2), p1.reshape(-1, 2), pwb.reshape(-1, 2), pred.reshape(-1, 2)
    h, w = g1.shape
    m = 8                                                    # a patch must lie inside both pictures
    good = (st1.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(pwb - pr, axis=1) < c["fb_px"]) & \
        (p1[:, 0] >= m) & (p1[:, 0] <= w - 1 - m) & (p1[:, 1] >= m) & (p1[:, 1] <= h - 1 - m) & \
        (pr[:, 0] >= m) & (pr[:, 0] <= w - 1 - m) & (pr[:, 1] >= m) & (pr[:, 1] <= h - 1 - m)
    # points stuck to the picture (OSD text, dirt on the lens) while the gyro says the view moved
    still = (np.linalg.norm(p1 - p0, axis=1) < c["still_px"]) & (np.linalg.norm(pr - p0, axis=1) > 3.0)
    good &= ~still
    idx = np.flatnonzero(good)
    if len(idx) < c["min_inl"]:
        return Rot(False, None, len(idx), None, None, 0.0, f"too few tracked far points ({len(idx)})"), None
    a, b = r0[idx], wl.rays(p1[idx])
    thr = math.radians(c["inl_deg"])
    rng = np.random.default_rng(rng_seed)
    best = None
    for _ in range(c["ransac_it"]):
        i, j = rng.choice(len(idx), 2, replace=False)
        if float(a[i] @ a[j]) > math.cos(math.radians(5)):
            continue
        M = wahba(a[[i, j]], b[[i, j]])
        inl = np.linalg.norm(b - a @ M.T, axis=1) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < c["min_inl"]:
        return Rot(False, None, 0 if best is None else int(best.sum()), None, None, 0.0,
                   "far points do not agree on one rotation"), None
    inl = best
    for _ in range(2):
        M = wahba(a[inl], b[inl])
        inl = np.linalg.norm(b - a @ M.T, axis=1) < thr
        if inl.sum() < c["min_inl"]:
            return Rot(False, None, int(inl.sum()), None, None, 0.0, "far points do not agree on one rotation"), None
    M = wahba(a[inl], b[inl])
    res = np.degrees(np.linalg.norm(b[inl] - a[inl] @ M.T, axis=1))
    rms = float(np.sqrt(np.mean(res ** 2)))
    xs = p0[idx[inl], 0]
    spread = float((xs.max() - xs.min()) / w * 100.0)                 # share of the picture width, percent
    n = int(inl.sum())
    q = min(1.0, n / 40.0) * max(0.0, 1.0 - rms / c["inl_deg"]) * min(1.0, spread / 50.0)
    why = None if spread >= 25.0 else f"far points in a narrow part of the picture ({spread:.0f} %)"
    return Rot(why is None, M, n, rms, spread, q if why is None else 0.0, why), idx[inl]


def heading_change(img0, img1, lens, R_bc, att0, att1, n_up0=None, band_deg=3.0, mask=None, cfg=None):
    """The yaw change that remains after the gyro's attitude change, from points far away.
    img0, img1: grey pictures. att0, att1: (roll, pitch, yaw) of the gyro-integrated attitude at the two pictures.
    R_bc: camera -> body (mount_R). n_up0: world up in camera axes at img0 (find_horizon(img0).n); None: from att0.
    Returns (remaining yaw in rad or None, Rot)."""
    c = dict(FP, **(cfg or {}))
    g0, s, wl = _work(img0, lens, HZ["work_w"])
    g1, _, _ = _work(img1, lens, HZ["work_w"])
    Rw0, Rw1 = euler_to_R(*att0), euler_to_R(*att1)
    if n_up0 is None:
        n_up0 = R_bc.T @ Rw0.T @ UP_NED
    valid = None if mask is None else cv2.resize(mask, (g0.shape[1], g0.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    pts0 = far_points(g0, wl, n_up0, band_deg, valid, c)
    M_pred = R_bc.T @ Rw1.T @ Rw0 @ R_bc
    rot, _ = track_rotation(g0, pts0, g1, wl, M_pred, c)
    if not rot.ok:
        return None, rot
    Rw1_meas = Rw0 @ R_bc @ rot.M.T @ R_bc.T
    return wrap(R_to_euler(Rw1_meas)[2] - att1[2]), rot


# ---------------------------------------------------------------- the reference since the pilot's switch
FwdState = namedtuple("FwdState", "t tilt_ok droll dpitch q_tilt yaw_ok dyaw q_yaw droll_pts dpitch_pts n_pts why")


class FwdRef:
    """Change of roll, pitch and yaw since a reference picture (the pilot's switch).

    Tilt: the horizon of each picture (find_horizon), turned into body axes by the mount. The mount tilt alpha is
    solved at the reference from the FC's roll and pitch when given (else alpha_deg is assumed); the change since the
    reference does not depend on alpha to first order. Yaw (and a second tilt, droll_pts / dpitch_pts): far points
    tracked from a key picture (first the reference) to each picture, starting from the gyro's predicted rotation.
    A new key picture is taken when fewer than kf_keep of the key picture's points remain; each change adds a small
    random error. Nothing is guessed: a refused horizon gives tilt_ok False, a refused rotation yaw_ok False."""

    def __init__(self, lens, mask=None, alpha_deg=15.0, height_m=None, hz_cfg=None, fp_cfg=None, kf_keep=0.6):
        self.lens, self.mask, self.alpha0 = lens, mask, math.radians(alpha_deg)
        self.height, self.hz_cfg, self.fp_cfg, self.kf_keep = height_m, hz_cfg, fp_cfg, kf_keep
        self.ref = None

    def set_height(self, height_m):
        self.height = height_m

    def _valid(self, g):
        if self.mask is None:
            return None
        return cv2.resize(self.mask, (g.shape[1], g.shape[0]), interpolation=cv2.INTER_NEAREST) > 0

    def set_reference(self, img, t, att=None):
        """Take the reference. att: (roll, pitch, yaw) of the attitude at this picture (gyro or FC), or None.
        Returns None, or the reason the picture cannot be a reference (try again with the next picture)."""
        hz = find_horizon(img, self.lens, self.mask, self.hz_cfg)
        if not hz.ok:
            return "no horizon: " + hz.why
        if att is not None:
            alpha, rho = solve_mount(hz.n, att[0], att[1])
            if not (math.radians(-15) <= alpha <= math.radians(50)) or abs(rho) > math.radians(15):
                return (f"mount solved from the FC attitude is out of range (tilt {math.degrees(alpha):.1f} deg, "
                        f"roll {math.degrees(rho):.1f} deg): check the lens, the picture orientation and the attitude")
        else:
            alpha, rho = self.alpha0, 0.0
        self.alpha, self.rho = alpha, rho
        self.R_bc = mount_R(alpha, rho)
        self.tilt0 = tilt_from_up(self.R_bc @ hz.n)
        self.Rw0 = euler_to_R(self.tilt0[0], self.tilt0[1], 0.0)        # yaw counted from the reference
        self.gyro0 = None if att is None else euler_to_R(*att)
        g, s, wl = _work(img, self.lens, (self.hz_cfg or {}).get("work_w", HZ["work_w"]))
        self.wl, self.t0 = wl, t
        self.ref = True
        self._new_key(g, np.eye(3), self.gyro0, hz.n)
        self.fails = 0
        return None

    def _new_key(self, g, M_ref_key, gyro_key, n_up):
        pts = far_points(g, self.wl, n_up, yaw_band_deg(self.height), self._valid(g), self.fp_cfg)
        self.key = dict(g=g, pts=pts, M=M_ref_key, gyro=gyro_key, n0=None)

    def update(self, img, t, att=None):
        """img: grey picture. att: (roll, pitch, yaw) of the gyro-integrated attitude at this picture, or None (then
        the rotation search starts from no turn and works only for small turns). Returns FwdState (angles rad)."""
        if self.ref is None:
            raise RuntimeError("FwdRef.update before set_reference")
        hz = find_horizon(img, self.lens, self.mask, self.hz_cfg)
        g, _, _ = _work(img, self.lens, self.wl.w)
        droll = dpitch = None
        if hz.ok:
            r, p = tilt_from_up(self.R_bc @ hz.n)
            droll, dpitch = wrap(r - self.tilt0[0]), p - self.tilt0[1]
        gyro = None if att is None else euler_to_R(*att)
        k = self.key
        M_pred = np.eye(3) if (gyro is None or k["gyro"] is None) else self.R_bc.T @ gyro.T @ k["gyro"] @ self.R_bc
        rot, _ = track_rotation(k["g"], k["pts"], g, self.wl, M_pred, self.fp_cfg)
        dyaw = rp = pp = None
        why = None if hz.ok else "no horizon: " + hz.why
        if rot.ok:
            if k["n0"] is None:
                k["n0"] = rot.n_inl
            M = rot.M @ k["M"]                                            # reference -> now, camera axes
            Rw = self.Rw0 @ self.R_bc @ M.T @ self.R_bc.T
            rr, pp_, yy = R_to_euler(Rw)
            dyaw, rp, pp = yy, wrap(rr - self.tilt0[0]), pp_ - self.tilt0[1]
            self.fails = 0
            if rot.n_inl < self.kf_keep * k["n0"]:                        # the view moved on: this picture is the key
                n_up = hz.n if hz.ok else M @ self.key_n_up_ref()
                self._new_key(g, M, gyro, n_up)
        else:
            self.fails += 1
            why = (why + "; " if why else "") + "no rotation: " + rot.why
            if self.fails >= 3 and gyro is not None and k["gyro"] is not None:
                # the key picture is lost: start a new key here from the gyro (adds the gyro drift since the key)
                M = M_pred @ k["M"]
                n_up = hz.n if hz.ok else M @ self.key_n_up_ref()
                self._new_key(g, M, gyro, n_up)
                self.fails = 0
                why += "; new key picture from the gyro"
        return FwdState(t, hz.ok, droll, dpitch, hz.quality, rot.ok, dyaw, rot.quality, rp, pp, rot.n_inl, why)

    def key_n_up_ref(self):
        """World up in camera axes at the reference."""
        return self.R_bc.T @ self.Rw0.T @ UP_NED
