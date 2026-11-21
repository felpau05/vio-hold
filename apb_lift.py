"""Floor tracking for the Stage 2 bench: height change (B3) and horizontal move (B1, B2) from the picture alone.

Why: indoors at the event the APB barometer wanders 0.3-0.6 m within 6 s (logs/live_125032.csv, 2026-11-21),
and B3 allows 0.15 m. Hold re-anchors at its last position after 0.4 s of refused frames, so a hand-carried move at
walking speed reads 17-53 % of the true distance (B1 runs, 2026-11-21); a slow 0.1 m/s walk read 93 %.

Model: in level-frame normalized coordinates a floor point G seen from the lens at C, height h, is n = (G - C) / h.
Between the seed frame 0 and now 1: n1 = (h0 / h1) n0 + (C0 - C1) / h1. A similarity fit (scale, turn, shift)
ref -> now gives scale = h0 / h1 and shift t, so h1 / h0 = 1 / scale and C1 - C0 = -t * h1. Each ray is first turned
level with the APB attitude (camera -> level frame): a hand tilt of a few degrees moves edge points about 1.7x as far
as centre points, and a plain fit would read that as a height change.

Height change = h_zero * (h_now / h_zero - 1), where h_zero is the lens height at the zero (the --height value).
With h_zero assumed 1.0 m, a true 0.5 m lift reads within 0.15 m for any true start height from 0.77 to 1.43 m.
The move scales with h_zero in the same way: a 10 % height error is a 10 % distance error.

Points are tracked frame to frame (LK, forward and backward), seeded in a central cone (no drone legs, less lens
distortion). When too few inliers stay, new points start and the scale and position so far are kept (chained).
A bad picture (covered lens, flat picture) or a lost track is a gap: the values are unknown during the gap, and the
chain then goes on from the last values (the move during the gap is lost). disp is (north, east) in m since the zero
when R holds the yaw (attitude_R @ R_bc), else camera axes.
"""
import math

import cv2
import numpy as np


class LiftScale:
    MAX_PTS = 120
    MIN_PTS = 25              # fewer inliers than this: no fit for this frame (a gap)
    RESEED_FRAC = 0.5         # new points when fewer than this part of the seeded points remain as inliers
    CONE_DEG = 40.0           # new points only within this angle of the optical axis
    KEEP_DEG = 50.0           # tracked points farther than this from the optical axis are dropped (legs at 55 deg)
    FB_PX = 1.0               # forward-backward LK error limit (px, at the reduced size)
    RANSAC_PX = 1.5           # similarity fit inlier limit (px, at the reduced size, near the centre)
    RANSAC_EDGE = 2.0         # x the limit: 1 px at 45 deg off-axis is (1 + tan^2) = 2x the normalized error at the centre
    DOWN = 2                  # track on a picture this many times smaller (CPU on the A53)
    MIN_EIG = 1.5e-3          # LK window min eigenvalue (OpenCV scale): a point is refused below it. Without it LK returns
                              # "no motion" on a covered lens, read as holding still. Measured 2026-11-21 on the real carpet
                              # (b_try3): still p50 3.4e-2; motion blur 10 px (full size) keeps 99 % of points at 1.5e-3
                              # (42 % at 4e-3, which gave 21-31 % NOT READY while walking in the C test); 20 px keeps 40 %.
                              # Covers: noise sd 3 at most 7e-4, white paper sd 4 at most 1.1e-3 (refused); a very noisy
                              # dark cover (sd 6, max 2.6e-3) can pass: then fine_detail/judge must catch it.
                              # It grows with floor contrast: check a plain or dark judging floor (gaps, NOT READY).

    def __init__(self, K, D, shape, h0=1.0):
        """K, D: fisheye lens of pictures of shape (h, w), as Hold gets them. h0: lens height (m) at the zero."""
        self.h0 = float(h0)
        self.K, self.D = K.astype(np.float64), D.astype(np.float64)
        self.f = float(K[0, 0]) / self.DOWN
        h, w = shape
        self.small = (w // self.DOWN, h // self.DOWN)
        u, v = np.meshgrid(np.arange(self.small[0]) * self.DOWN, np.arange(self.small[1]) * self.DOWN)
        n = self._undist(np.c_[u.ravel(), v.ravel()].astype(np.float64))
        r = np.hypot(n[:, 0], n[:, 1]).reshape(self.small[1], self.small[0])
        self.mask = (r < math.tan(math.radians(self.CONE_DEG))).astype(np.uint8) * 255
        self.zero()

    def zero(self):
        """Height ratio 1 and move 0 from the next good picture on."""
        self.prev, self.pts, self.ref, self.n_seed = None, None, None, 0
        self.base, self.pos_seed = 1.0, np.zeros(2)            # values at the current seed frame
        self.last = None                                       # (ratio, disp) of the last good frame
        self.ratio, self.disp, self.n, self.gaps = None, None, 0, 0

    def _undist(self, px):
        """Full-size pixels -> undistorted normalized camera coordinates (x / z, y / z)."""
        px = np.asarray(px, np.float64).reshape(-1, 2)
        if not len(px):                 # cv2 returns None for no points (board error 2026-11-21: 'NoneType' .reshape)
            return np.zeros((0, 2))
        return cv2.fisheye.undistortPoints(px.reshape(-1, 1, 2), self.K, self.D).reshape(-1, 2)

    @staticmethod
    def _level(n, R):
        """Camera normalized coordinates -> level-frame normalized coordinates. R: camera -> level (z down)."""
        if R is None:
            return n
        b = np.c_[n, np.ones(len(n))] @ np.asarray(R, np.float64).T
        return b[:, :2] / np.maximum(b[:, 2:3], 1e-6)

    def _seed(self, small, R):
        p = cv2.goodFeaturesToTrack(small, self.MAX_PTS, 0.003, 7, mask=self.mask)
        if p is None or len(p) < self.MIN_PTS:
            self.pts, self.ref, self.n_seed = None, None, 0
            return False
        self.pts = p.reshape(-1, 2).astype(np.float32)
        self.ref = self._level(self._undist(self.pts.astype(np.float64) * self.DOWN), R)
        self.n_seed = len(self.pts)
        return True

    def _gap(self):
        """No values now; the next good picture starts new points from the last good values."""
        if self.last is not None and (self.prev is not None or self.pts is not None):
            self.gaps += 1
            self.base, self.pos_seed = 1.0 / self.last[0], self.last[1].copy()
        self.prev, self.pts = None, None
        self.ratio, self.disp, self.n = None, None, 0

    def bad_picture(self):
        """A covered or flat picture."""
        self._gap()

    def update(self, img, R=None):
        """img: grey picture as Hold gets it. R: camera -> level frame for this picture (attitude @ R_bc), or None
        (no tilt correction). Returns h_now / h_zero, or None (a gap)."""
        small = cv2.resize(img, self.small, interpolation=cv2.INTER_AREA)
        if self.prev is None or self.pts is None:
            # New points (start, after a gap, or after a failed reseed): no value until one good fit to the next
            # picture confirms them. Sensor noise on a covered lens gives "corners" that never track, so a cover
            # stays a gap on every frame (review E1-2: READY must not come back from a seed alone).
            self.prev = small
            self._seed(small, R)
            self.ratio, self.disp, self.n = None, None, 0
            return None
        # the backward call has the new picture as its template, so a cover fails on its first frame
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev, small, self.pts, None, winSize=(15, 15), maxLevel=3,
                                             minEigThreshold=self.MIN_EIG)
        p0, st2, _ = cv2.calcOpticalFlowPyrLK(small, self.prev, p1, None, winSize=(15, 15), maxLevel=3,
                                              minEigThreshold=self.MIN_EIG)
        good = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(p0 - self.pts, axis=1) < self.FB_PX)
        if good.sum() < self.MIN_PTS:                   # LK refused (nearly) every point: a gap, no fit to try
            self._gap()
            return self.update(img, R)
        ref, p1 = self.ref[good], p1[good]
        nc = self._undist(p1.astype(np.float64) * self.DOWN)
        keep = np.hypot(nc[:, 0], nc[:, 1]) < math.tan(math.radians(self.KEEP_DEG))
        ref, p1, nc = ref[keep], p1[keep], nc[keep]
        A = inl = None
        if len(p1) >= self.MIN_PTS:
            A, inl = cv2.estimateAffinePartial2D(ref, self._level(nc, R), method=cv2.RANSAC,
                                                 ransacReprojThreshold=self.RANSAC_EDGE * self.RANSAC_PX / self.f)
        if A is None or inl is None or inl.sum() < self.MIN_PTS:
            self._gap()                                 # the track is lost: start again from the last values
            return self.update(img, R)                  # seed on this picture at once
        m = inl.ravel() == 1                            # keep the inliers only: new points start when they run out
        self.prev, self.pts, self.ref, self.n = small, p1[m], ref[m], int(m.sum())
        scale = float(math.hypot(A[0, 0], A[1, 0]))
        self.ratio = 1.0 / (self.base * scale)          # the floor shrinks as the drone rises
        self.disp = self.pos_seed - A[:, 2] * (self.h0 * self.ratio)
        self.last = (self.ratio, self.disp.copy())
        if self.n < self.RESEED_FRAC * self.n_seed:
            self.base, self.pos_seed = 1.0 / self.ratio, self.disp.copy()   # keep the values so far, new points here
            self._seed(small, R)
        return self.ratio
