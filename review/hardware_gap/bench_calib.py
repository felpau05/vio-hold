"""Reference implementation of the apb_main.sweep spec, plus a mounting and focal check.

One recording, made by turning the drone by hand (props off) while the camera sees a scene, gives:
  - the total time offset between the frame stamp and the first-row exposure (latency),
  - the rolling-shutter readout time,
  - the camera-to-body rotation: picture turned 0/90/180/270 deg, mirrored or not, plus small tilts,
  - a focal-length scale against the lens profile.

It uses only rotation: the direction (in NED) of a scene point seen from a camera that only turns must
not change between two frames. That test does not need the height, and camera translation adds
error only as (translation / scene distance). So on the event day, aim the camera at a scene far
away (buildings or trees > 50 m: roll the drone 90 deg onto its side), or at the floor with the
camera turned about its own centre (as in sim_test.timing_test).

Recording format (same as sim_test.timing_test writes):
  imgs (N, rows, cols) uint8, t_grab (N,) companion clock, att (M, 4) [t, roll, pitch, yaw] in the
  same clock, K, D (lens of the capture size), hgt (unused here).
"""
import math

import cv2
import numpy as np

from harness import R_BC_NADIR, euler_R, rot_x, rot_y, rot_z

LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def track_pairs(imgs, max_pts=60, max_age=60, min_alive=25):
    """Keyframe tracks: each frame's points are matched back to the keyframe they were found in.

    A long baseline matters: with consecutive frames only, a time error shows up as
    (change of rotation rate over 33 ms) x error, which is too small to measure.
    Returns (keyframe index, frame index, keyframe pixel, frame pixel) per match.
    """
    IA, IB, P0, P1 = [], [], [], []
    kf, pk, prev, alive = 0, None, None, None
    for i in range(len(imgs)):
        if pk is None:
            p = cv2.goodFeaturesToTrack(imgs[i], max_pts, 0.01, 7)
            if p is None:
                continue
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
    return np.concatenate(IA), np.concatenate(IB), np.concatenate(P0), np.concatenate(P1)


def nelder_mead(f, x0, step, iters=160):
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


class Bench:
    def __init__(self, path_or_dict):
        d = np.load(path_or_dict) if isinstance(path_or_dict, str) else path_or_dict
        self.imgs, self.tg, self.att = d["imgs"], np.asarray(d["t_grab"], float), np.asarray(d["att"], float)
        self.K, self.D = np.asarray(d["K"], float), np.asarray(d["D"], float)
        self.rows, self.cols = self.imgs.shape[1:3]
        self.ia, self.ib, self.p0, self.p1 = track_pairs(self.imgs)
        self._yaw = np.unwrap(self.att[:, 3])
        self._ray_cache = {}

    def _rays(self, fscale, mirror):
        key = (round(fscale, 6), mirror)
        if key not in self._ray_cache:
            K = self.K.copy()
            K[0, 0] *= fscale
            K[1, 1] *= fscale
            out = []
            for px in (self.p0, self.p1):
                px = px.astype(np.float64).copy()
                if mirror:
                    px[:, 0] = self.cols - 1 - px[:, 0]
                n = cv2.fisheye.undistortPoints(px.reshape(-1, 1, 2), K, self.D).reshape(-1, 2)
                f = np.c_[n, np.ones(len(n))]
                out.append(f / np.linalg.norm(f, axis=1, keepdims=True))
            if len(self._ray_cache) > 64:
                self._ray_cache.clear()
            self._ray_cache[key] = out
        return self._ray_cache[key]

    def _R(self, t):
        a = self.att
        return euler_R(np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]), np.interp(t, a[:, 0], self._yaw))

    def residual(self, lat, ro, R_bc=R_BC_NADIR, fscale=1.0, mirror=False):
        """Median angle (deg) between the NED directions of each matched point in two frames."""
        c0, c1 = self._rays(fscale, mirror)
        ta = self.tg[self.ia] - lat + self.p0[:, 1] / self.rows * ro
        tb = self.tg[self.ib] - lat + self.p1[:, 1] / self.rows * ro
        n0 = np.einsum("nij,nj->ni", self._R(ta), c0 @ R_bc.T)
        n1 = np.einsum("nij,nj->ni", self._R(tb), c1 @ R_bc.T)
        return float(np.degrees(np.median(np.linalg.norm(np.cross(n0, n1), axis=1))))

    def timing(self, R_bc=R_BC_NADIR, fscale=1.0, mirror=False, lat_max=0.25):
        lats, ros = np.arange(0, lat_max + 1e-9, 0.01), np.arange(0, 0.0401, 0.005)
        g = np.array([[self.residual(l, r, R_bc, fscale, mirror) for r in ros] for l in lats])
        il, ir = np.unravel_index(np.argmin(g), g.shape)
        lat, ro = lats[il], ros[ir]
        lats2, ros2 = lat + np.arange(-0.008, 0.0081, 0.002), ro + np.arange(-0.004, 0.0041, 0.002)
        g2 = np.array([[self.residual(l, r, R_bc, fscale, mirror) for r in ros2] for l in lats2])
        il, ir = np.unravel_index(np.argmin(g2), g2.shape)
        return float(lats2[il]), float(max(ros2[ir], 0.0)), float(g2[il, ir])

    def refine(self, lat, ro, k, mirror):
        """Small tilts (deg, about body x, y, z) and focal scale for one picture orientation."""
        R0 = R_BC_NADIR @ rot_z(k * math.pi / 2)

        def f(x):
            R = rot_x(math.radians(x[0])) @ rot_y(math.radians(x[1])) @ rot_z(math.radians(x[2])) @ R0
            return self.residual(lat, ro, R, 1.0 + x[3], mirror)
        x, v = nelder_mead(f, [0, 0, 0, 0], [2.0, 2.0, 2.0, 0.05], iters=70)
        R = rot_x(math.radians(x[0])) @ rot_y(math.radians(x[1])) @ rot_z(math.radians(x[2])) @ R0
        return dict(turn_deg=90 * k, mirror=mirror, roll_deg=x[0], pitch_deg=x[1], yaw_deg=x[2],
                    fscale=1.0 + x[3], resid_deg=v, R_bc=R)

    def full_check(self):
        """Orientation, latency, readout, small tilts and focal scale, in that order."""
        coarse = []
        for mirror in (False, True):
            for k in range(4):
                R0 = R_BC_NADIR @ rot_z(k * math.pi / 2)
                for lat in np.arange(0, 0.2501, 0.025):
                    coarse.append((self.residual(lat, 0.016, R0, 1.0, mirror), k, mirror, lat))
        coarse.sort(key=lambda c: c[0])
        _, k, mirror, lat = coarse[0]
        m = self.refine(lat, 0.016, k, mirror)
        lat, ro, _ = self.timing(m["R_bc"], m["fscale"], mirror)
        m = self.refine(lat, ro, k, mirror)
        lat, ro, res = self.timing(m["R_bc"], m["fscale"], mirror)
        m.update(lat=lat, ro=ro, resid_deg=res,
                 runner_up=dict(turn_deg=90 * coarse[1][1], mirror=coarse[1][2], resid_deg=coarse[1][0]))
        best_other = next(c for c in coarse if (c[1], c[2]) != (k, mirror))
        m["other_orientation_resid_deg"] = best_other[0]
        return m


def sweep(path, R_bc):
    """Spec for apb_main.sweep: (latency s, readout s) from a hand-rotation recording.

    latency = t_grab - (time the first image row was read), in the attitude clock. Feed
    Hold.process with t0 = t_grab - latency and Hold(..., readout_s=readout).
    """
    b = Bench(path)
    lat, ro, _ = b.timing(R_bc)
    return lat, ro
