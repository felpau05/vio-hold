"""Independent geometry for the geometry review, written from first principles.

It does NOT import euler_R, R_BC_NADIR, RAYS_B or render from the project. It has its own
quaternions, its own camera mount built from a physical description, its own equidistant-fisheye
forward and inverse model, its own rolling-shutter projector and its own image renderer.
cv2 is used only to sample a texture (remap) and to blur; numpy for the arithmetic.
"""
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, "..", ".."))
LENS_JSON = os.path.join(PROJ, "justice_lens_profiles.json")
if PROJ not in sys.path:
    sys.path.insert(0, PROJ)

E_N, E_E, E_D = np.eye(3)           # NED unit vectors (also body F, R, D unit vectors)


# ---------------------------------------------------------------- quaternions (w, x, y, z)
def quat_axis_angle(axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis, axis=-1, keepdims=True)
    ang = np.asarray(ang, float)[..., None]
    return np.concatenate([np.cos(ang / 2), axis * np.sin(ang / 2)], -1)


def qmul(p, q):
    w1, x1, y1, z1 = np.moveaxis(p, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(q, -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)


def qconj(q):
    return q * np.array([1, -1, -1, -1.0])


def qrot(q, v):
    """Active rotation of v by unit quaternion q (q v q*)."""
    w, u = q[..., :1], q[..., 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def att_quat(roll, pitch, yaw):
    """Body -> NED, built from the physical meaning of the three angles.

    Start with the body axes on N, E, D. Turn the body about Down by yaw (nose toward East is +).
    Then turn it about its OWN right axis by pitch (nose up is +). Then turn it about its OWN
    forward axis by roll (right wing down is +). The axes are taken from the current body each time.
    """
    roll, pitch, yaw = np.broadcast_arrays(*(np.asarray(a, float) for a in (roll, pitch, yaw)))
    shp = roll.shape + (3,)
    q = quat_axis_angle(np.broadcast_to(E_D, shp), yaw)
    right = qrot(q, np.broadcast_to(E_E, shp))
    q = qmul(quat_axis_angle(right, pitch), q)
    fwd = qrot(q, np.broadcast_to(E_N, shp))
    q = qmul(quat_axis_angle(fwd, roll), q)
    return q


def quat_to_matrix(q):
    cols = [qrot(q, np.broadcast_to(e, q.shape[:-1] + (3,))) for e in (E_N, E_E, E_D)]
    return np.stack(cols, -1)


# ---------------------------------------------------------------- camera mount
def mount(top_toward=(1, 0, 0), mirrored=False):
    """Camera -> body matrix (columns: camera x, y, z in body FRD) for a nadir camera.

    The optical axis (camera +z) is body down. 'top_toward' is the body direction that the top
    edge of the image points to. Image down (+y) is the opposite. Image right (+x) = y cross z,
    so the camera frame is right-handed; a mirrored picture flips x.
    """
    z = E_D.copy()
    y = -np.asarray(top_toward, float)
    x = np.cross(y, z)
    if mirrored:
        x = -x
    return np.column_stack([x, y, z])


MOUNTS = {
    "top->nose (code's assumption)": mount((1, 0, 0)),
    "top->tail (turned 180 deg)": mount((-1, 0, 0)),
    "top->right wing (turned 90 deg)": mount((0, 1, 0)),
    "top->left wing (turned -90 deg)": mount((0, -1, 0)),
    "top->nose, picture mirrored L/R": mount((1, 0, 0), mirrored=True),
}


# ---------------------------------------------------------------- lens
def lens_profile(name):
    return next(m for m in json.load(open(LENS_JSON))["models"] if m["name"] == name)


def field_K(p, w, h):
    """Scale a calibration to a w x h picture with the same extent.

    Pixel centres are integers (OpenCV). A continuous edge coordinate is u + 0.5, and it scales
    with the picture: (u_new + 0.5) = (u_old + 0.5) * s.
    """
    pw, ph = p["image_size"]
    sx, sy = w / pw, h / ph
    K = np.array([[p["fx"] * sx, 0.0, (p["cx"] + 0.5) * sx - 0.5],
                  [0.0, p["fy"] * sy, (p["cy"] + 0.5) * sy - 0.5],
                  [0.0, 0.0, 1.0]])
    D = np.array([p.get("k1", 0), p.get("k2", 0), p.get("k3", 0), p.get("k4", 0)], float)
    return K, D


def project_cam(xyz, K, D):
    """Equidistant fisheye, distortion polynomial in the incidence angle theta."""
    X, Y, Z = np.moveaxis(np.asarray(xyz, float), -1, 0)
    th = np.arctan2(np.hypot(X, Y), Z)
    ph = np.arctan2(Y, X)
    t2 = th * th
    td = th * (1 + D[0] * t2 + D[1] * t2 ** 2 + D[2] * t2 ** 3 + D[3] * t2 ** 4)
    return K[0, 0] * td * np.cos(ph) + K[0, 2], K[1, 1] * td * np.sin(ph) + K[1, 2]


def unproject_cam(u, v, K, D):
    """Inverse of project_cam (Newton on theta). Returns unit rays in the camera frame."""
    xd, yd = (np.asarray(u, float) - K[0, 2]) / K[0, 0], (np.asarray(v, float) - K[1, 2]) / K[1, 1]
    td, ph = np.hypot(xd, yd), np.arctan2(yd, xd)
    th = td.copy()
    for _ in range(30):
        t2 = th * th
        f = th * (1 + D[0] * t2 + D[1] * t2 ** 2 + D[2] * t2 ** 3 + D[3] * t2 ** 4) - td
        fp = 1 + 3 * D[0] * t2 + 5 * D[1] * t2 ** 2 + 7 * D[2] * t2 ** 3 + 9 * D[3] * t2 ** 4
        th = th - f / fp
    return np.stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)], -1)


# ---------------------------------------------------------------- rolling-shutter timing
def row_time(v, t0, readout, H, reverse=False):
    """Time a (continuous) row v is read. Top-first: row 0 at t0. reverse: row H-1 at t0."""
    v = np.asarray(v, float)
    return t0 + ((H - 1 - v) if reverse else v) / H * readout


def project_ground(P, pos_fn, att_fn, h_fn, t0, readout, K, D, R_bc, H, reverse=False, iters=8):
    """Pixels (u, v) and read times of NED ground points P (n, 2) for a rolling-shutter camera.

    The row a point lands on sets its read time, and that time sets the pose: iterate to a fixed point.
    """
    P = np.asarray(P, float)
    n = len(P)
    t = np.full(n, t0 + readout / 2)
    for _ in range(iters):
        q = att_quat(*np.asarray(att_fn(t), float).T)
        C = pos_fn(t)
        d = np.c_[P - C, h_fn(t)]                      # camera -> ground point, NED (ground below)
        d_cam = qrot(qconj(q), d) @ R_bc               # body -> camera: R_bc^T d_body (row vectors)
        u, v = project_cam(d_cam, K, D)
        t = row_time(v, t0, readout, H, reverse)
    return np.c_[u, v], t


# ---------------------------------------------------------------- texture and renderer
def make_texture(n=4096, seed=7, water_half=None):
    """Multi-scale noise ground. water_half='west' makes the west half flat (no features)."""
    rng = np.random.default_rng(seed)
    t = np.zeros((n, n), np.float32)
    for k in range(3, 11):
        t += cv2.resize(rng.standard_normal((2 ** k, 2 ** k)).astype(np.float32), (n, n),
                        interpolation=cv2.INTER_CUBIC)
    lo, hi = np.percentile(t, [1, 99])
    t = cv2.GaussianBlur(np.clip((t - lo) / (hi - lo) * 255, 0, 255), (0, 0), 1.0)
    if water_half == "west":
        t[:, : n // 2] = 100.0
    return t


class Renderer:
    """Rolling-shutter nadir camera over a flat textured ground (own model end to end)."""

    def __init__(self, K, D, R_bc, W, H, tex, span_m, noise=3.0, seed=11):
        self.W, self.H, self.tex, self.m = W, H, tex, tex.shape[0] / span_m
        uu, vv = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
        self.rays_b = unproject_cam(uu, vv, K, D) @ R_bc.T          # body-frame ray per pixel
        self.noise, self.rng = noise, np.random.default_rng(seed)

    def frame(self, t0, readout, pos_fn, att_fn, h_fn, reverse=False):
        tr = row_time(np.arange(self.H), t0, readout, self.H, reverse)
        q = att_quat(*np.asarray(att_fn(tr), float).T)              # (H, 4)
        b = qrot(q[:, None, :], self.rays_b)                        # (H, W, 3) NED
        bz = np.maximum(b[..., 2], 0.05)
        P = pos_fn(tr)[:, None, :] + (h_fn(tr)[:, None] / bz)[..., None] * b[..., :2]
        n = self.tex.shape[0]
        img = cv2.remap(self.tex, (P[..., 1] * self.m + n / 2).astype(np.float32),
                        (-P[..., 0] * self.m + n / 2).astype(np.float32), cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REFLECT101)
        img = cv2.GaussianBlur(img, (3, 3), 0.8) + self.rng.normal(0, self.noise, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- flight-controller feeds
def feeds(att_fn, h_fn, T, att_hz=200.0, h_hz=20.0, att_err_fn=None, h_err_fn=None):
    """Exact (or deliberately corrupted) attitude and height, as hold.Series objects."""
    from hold import Series            # a plain timestamped container; no geometry in it
    att, hgt = Series(1e9), Series(1e9)
    ts = np.arange(-1.0, T + 1.0, 1.0 / att_hz)
    a = np.asarray(att_fn(ts), float)
    if att_err_fn is not None:
        a = a + att_err_fn(ts)
    for t, r in zip(ts, a):
        att.add(t, *r)
    ts = np.arange(-1.0, T + 1.0, 1.0 / h_hz)
    hv = h_fn(ts) + (h_err_fn(ts) if h_err_fn is not None else 0)
    for t, x in zip(ts, hv):
        hgt.add(t, x)
    return att, hgt


W, H, FPS = 360, 240, 30.0
K_FIELD, D_FIELD = field_K(lens_profile("Round 1 independent k2"), W, H)
