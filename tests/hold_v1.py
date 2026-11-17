"""Frozen copy of hold.py version 1 (2026-11-15), used only by tests/ as the "before" in before/after numbers."""
"""Keyframe-anchored nadir position hold for a rolling-shutter camera.

A ground point seen in the keyframe has a fixed NED position G. In a later frame the camera
sits at G minus that point's ray, scaled down to the ground plane by the height. Each ray is
rotated with the flight-controller attitude at the time its image row was read out, which
removes rolling-shutter skew. While the keyframe stays in view the fix does not drift.

No I/O here: apb_main.py feeds it live data, sim_test.py feeds it synthetic frames.
"""
import json
import math
from collections import namedtuple

import cv2
import numpy as np

# Nadir camera, image top toward the nose: image x = body right, image y = body back,
# optical axis = body down. Columns are the camera axes written in body FRD.
R_BC_NADIR = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])

# minEigThreshold drops points with too little texture: there LK returns "no motion", and stuck points
# turn every tilt into a fake move (seen in grey static). The default 1e-4 lets them through.
# shortcut: 3e-3 rejects sim static (grey, noise std 3) and keeps sim ground; retune on event-day video.
LK = dict(winSize=(21, 21), maxLevel=3, minEigThreshold=3e-3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

Est = namedtuple("Est", "t pos var n state recovered why cov yaw scale", defaults=(None, None, None))
# state: 'init' (new anchor, nothing to send), 'ok', 'degraded' (send, larger covariance),
#        'lost' (send nothing; caller re-anchors from the flight controller's estimate)
# Send a fix only when var is not None. var = trace of cov (kept for old callers).
# cov: 2x2 covariance of pos (m^2). yaw: yaw correction from the image (rad). scale: image scale
# against the keyframe (height now / height at the keyframe). Each is None when not estimated.


def load_lens(path, name, w, h):
    """K, D for a w x h image that shows the same picture extent as the profile's image."""
    p = next(m for m in json.load(open(path))["models"] if m["name"] == name)
    pw, ph = p["image_size"]
    sx, sy = w / pw, h / ph
    K = np.array([[p["fx"] * sx, 0, (p["cx"] + 0.5) * sx - 0.5],
                  [0, p["fy"] * sy, (p["cy"] + 0.5) * sy - 0.5],
                  [0, 0, 1.]])
    return K, np.array([p["k1"], p["k2"], p["k3"], p["k4"]], float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def euler_R(roll, pitch, yaw):
    """Body FRD -> NED, ZYX order like MAVLink ATTITUDE. Works on arrays."""
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.empty(np.shape(roll) + (3, 3))
    R[..., 0, 0], R[..., 0, 1], R[..., 0, 2] = cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr
    R[..., 1, 0], R[..., 1, 1], R[..., 1, 2] = sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr
    R[..., 2, 0], R[..., 2, 1], R[..., 2, 2] = -sp, cp * sr, cp * cr
    return R


_TOP = {"front": (1., 0.), "right": (0., 1.), "rear": (-1., 0.), "left": (0., -1.)}


def make_R_bc(top="front", mirror=False, tilt_deg=(0., 0.), twist_deg=0.):
    """Camera axes (x = image right, y = image down, z = optical axis) written in body FRD.

    top: the body direction that the picture's top edge points to. mirror: the picture is flipped
    left-right. tilt_deg: (roll, pitch) of the mount against straight down. twist_deg: turn about
    the optical axis. make_R_bc() equals R_BC_NADIR. With mirror=True the result is not a rotation.
    """
    ux, uy = _TOP[top]
    y = np.array([-ux, -uy, 0.])                 # image down = opposite of the picture's top
    z = np.array([0., 0., 1.])
    x = np.cross(y, z) * (-1. if mirror else 1.)
    R = np.stack([x, y, z], axis=1) @ rot_z(math.radians(twist_deg))
    return euler_R(math.radians(tilt_deg[0]), math.radians(tilt_deg[1]), 0.) @ R


class Series:
    """Timestamped samples written by one thread, read by another (one list append is atomic)."""

    def __init__(self, keep_s=5.0):
        self.rows, self.keep, self._cache = [], keep_s, (None, None)

    def add(self, t, *vals):
        self.rows.append((t, *vals))
        if len(self.rows) > 4000 and t - self.rows[0][0] > self.keep:
            del self.rows[:2000]

    def last_t(self):
        return self.rows[-1][0] if self.rows else -math.inf

    def array(self):
        key = (len(self.rows), self.last_t())
        if self._cache[0] != key:
            self._cache = (key, np.array(self.rows, float))
        return self._cache[1]

    def at(self, t):
        a = self.array()
        return np.interp(t, a[:, 0], a[:, 1])


def attitude_R(att, t):
    """Body->NED at times t, from a Series of (t, roll, pitch, yaw)."""
    a = att.array()
    yaw = np.unwrap(a[:, 3])
    return euler_R(np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]),
                   np.interp(t, a[:, 0], yaw))


class Hold:
    MIN_PTS = 20              # fewer inliers: frame unusable
    LOST_AFTER = 10           # unusable frames in a row (~0.33 s at 30 Hz) before 'lost'
    MAX_RATE = 2.0            # rad/s; faster rotation makes row timing errors dominate
    KF_RATE = 0.3             # rad/s; take keyframes only below this unless overlap is nearly gone
    MAX_OFF_NADIR = math.radians(55)   # slanted rays: poor lens model and flat-ground error
    ATT_NOISE = math.radians(0.15)     # FC attitude noise; sets the error floor h * ATT_NOISE
    # Baro offset (m). It scales every move from the anchor by HGT_ERR / h, which dominates down low.
    # shortcut: a guess; set it from the baro drift seen on the event day.
    HGT_ERR = 1.0

    def __init__(self, K, D, R_bc, readout_s, mask=None, **cfg):
        """mask: uint8 H x W, 0 = not ground (OSD text, drone parts). cfg: tuning overrides by keyword."""
        self.K, self.D, self.R_bc, self.readout = K, D, R_bc, readout_s
        self.mask, self.cfg = mask, cfg          # stub: stored, not used yet
        self.kf = None
        self.pos, self.vel, self.t, self.var_last = np.zeros(2), np.zeros(2), None, 0.0
        self.bad, self.lost, self.rate = 0, False, 0.0

    def reanchor(self, pos, var=None):
        """After a loss: restart from the flight controller's position so the EKF sees no jump."""
        self.pos, self.vel, self.t, self.kf = np.asarray(pos, float), np.zeros(2), None, None
        self.var_last = 0.0

    def _rays(self, px, t0, rows, att, hgt):
        """Horizontal camera-to-ground offsets (NED, m) for pixels px, plus row times and heights."""
        t = t0 + px[:, 1] / rows * self.readout
        n = cv2.fisheye.undistortPoints(px.reshape(-1, 1, 2).astype(np.float64), self.K, self.D)
        f = np.c_[n.reshape(-1, 2), np.ones(len(px))] @ self.R_bc.T
        b = np.einsum("nij,nj->ni", attitude_R(att, t), f)
        h = hgt.at(t)
        ok = b[:, 2] > math.cos(self.MAX_OFF_NADIR) * np.linalg.norm(b, axis=1)
        return h[:, None] * b[:, :2] / np.maximum(b[:, 2:3], 1e-6), t, ok, h

    def _check_inputs(self, t0, att, hgt):
        t1 = t0 + self.readout
        if att.last_t() < t1 - 0.005:
            return "attitude late"
        if hgt.last_t() < t0 - 0.5:
            return "height late"
        a = att.array()
        w = a[(a[:, 0] > t0 - 0.03) & (a[:, 0] < t1 + 0.03)]
        if len(w) >= 2:
            rate = np.diff(np.c_[w[:, 1], w[:, 2], np.unwrap(w[:, 3])], axis=0) / np.diff(w[:, 0])[:, None]
            self.rate = float(np.abs(rate).max())
            if self.rate > self.MAX_RATE:
                return "rotating too fast"
        return None

    def _new_kf(self, img, t0, att, hgt):
        px = cv2.goodFeaturesToTrack(img, 300, 0.01, 7)
        if px is None or len(px) < self.MIN_PTS:
            return False
        # Same texture test as tracking, so an anchor is never made from static or a blank picture.
        _, s, _ = cv2.calcOpticalFlowPyrLK(img, img, px, None, **LK)
        px = px[s.ravel() == 1].reshape(-1, 2).astype(np.float32)
        if len(px) < self.MIN_PTS:
            return False
        g, t, ok, _ = self._rays(px, t0, img.shape[0], att, hgt)
        if ok.sum() < self.MIN_PTS:
            return False
        tref = t0 + self.readout / 2
        G = self.pos + self.vel * (t - tref)[:, None] + g
        # A new anchor inherits the current fix's error: every later fix is measured from it.
        self.kf = dict(img=img, px=px[ok].reshape(-1, 1, 2), G=G[ok], n0=int(ok.sum()),
                       pos=self.pos.copy(), var=self.var_last)
        self.prev_img, self.prev_px = img, self.kf["px"].copy()
        return True

    def _fail(self, tref, why):
        self.bad += 1
        if self.bad >= self.LOST_AFTER:
            self.lost, self.kf = True, None
        return Est(tref, self.pos.copy(), None, 0, "lost" if self.lost else "degraded", False, why)

    def process(self, img, t0, att, hgt):
        """img: uint8 gray. t0: time the first image row was read (same clock as att/hgt)."""
        tref = t0 + self.readout / 2
        why = self._check_inputs(t0, att, hgt)
        if self.kf is None:
            if why:
                return self._fail(tref, why)
            if not self._new_kf(img, t0, att, hgt):
                return self._fail(tref, "not enough texture")
            return Est(tref, self.pos.copy(), None, self.kf["n0"], "init", False, None)

        # Previous frame gives the guess; matching back to the keyframe keeps the fix drift-free.
        kf = self.kf
        p1, s1, _ = cv2.calcOpticalFlowPyrLK(self.prev_img, img, self.prev_px, None, **LK)
        p2, s2, _ = cv2.calcOpticalFlowPyrLK(kf["img"], img, kf["px"], p1.copy(),
                                             flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **LK)
        pb, s3, _ = cv2.calcOpticalFlowPyrLK(img, kf["img"], p2, kf["px"].copy(),
                                             flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **LK)
        good = (s1 & s2 & s3).ravel().astype(bool)
        good &= np.linalg.norm((pb - kf["px"]).reshape(-1, 2), axis=1) < 0.5
        self.prev_img, self.prev_px = img, p2
        if why:
            return self._fail(tref, why)
        if good.sum() < self.MIN_PTS:
            return self._fail(tref, f"{good.sum()} points tracked")

        g, t, ok, h = self._rays(p2.reshape(-1, 2)[good], t0, img.shape[0], att, hgt)
        C = (kf["G"][good] - g + self.vel * (tref - t)[:, None])[ok]
        med = np.median(C, axis=0)
        r = np.linalg.norm(C - med, axis=1)
        hm = float(np.mean(h))
        inl = r < max(3 * 1.4826 * np.median(r), 0.003 * hm + 0.05)
        n = int(inl.sum())
        if n < self.MIN_PTS:
            return self._fail(tref, f"{n} consistent points")
        pos = C[inl].mean(axis=0)
        spread = float(np.sqrt(C[inl].var(axis=0).sum()))
        if self.t is not None:
            dt = tref - self.t
            if np.linalg.norm(pos - (self.pos + self.vel * dt)) > 1.0 + 0.05 * hm:
                return self._fail(tref, "jump")
            if dt > 0:
                self.vel = 0.7 * self.vel + 0.3 * (pos - self.pos) / dt
        self.pos, self.t = pos, tref
        var = (kf["var"] + spread ** 2 / n + (self.ATT_NOISE * hm) ** 2
               + (self.HGT_ERR / hm * np.linalg.norm(pos - kf["pos"])) ** 2)
        self.var_last = var
        # Scatter well above pixel noise (~0.3 % of height) means vibration, bad timing or bad ground.
        state = "ok" if spread < 0.02 * hm + 0.2 and n >= 2 * self.MIN_PTS else "degraded"
        recovered, self.lost, self.bad = self.lost, False, 0
        # Overlap shrinking: anchor a fresh keyframe, preferably while rotating slowly, because
        # every later fix inherits this frame's attitude and timing error (the gps-denied rule).
        if n < 2 * self.MIN_PTS or (n < 0.5 * kf["n0"] and self.rate < self.KF_RATE):
            self._new_kf(img, t0, att, hgt)
        return Est(tref, pos, var, n, state, recovered, None)
