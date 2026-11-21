#!/usr/bin/env python3
"""Build hold.py variants for this study from integ/diag10/hold_base.py (or hold_patch.py) by exact text replacement.
  nadir  the whole-picture path uses only keyframe pixels whose ray is within DENSE_NADIR (default MAX_OFF_NADIR, 55 deg)
         of straight down, with the keyframe's attitude (Hold._make_kf stores the mask; Hold._dense uses it). Why: on a
         drone tilted 15-25 deg the fisheye sees ground up to 75 deg off nadir on one side; there one ground metre is a
         few px, the texture-weighted lever of a yaw error is long, and the cubic map is extrapolated (integ/lowh).
         'keyframe out of view' compares with the keyframe's own usable share.
  yaw    the whole-picture path also solves a turn about the vertical (yaw) besides the drone move, in the same
         Lucas-Kanade step; the points move by J dl + Jy dyaw, so the fit sees the turn (phi) and the attitude's yaw
         error no longer turns into a position error. The halves test still compares the move only.
Run: ~/.venvs/vio/bin/python integ/diag10/make_variants.py <src.py> <out.py> nadir [yaw] [nadir_deg=40]
"""
import sys

PARTS = {
    "nadir": [
        ("""                    pending=pending, n_first=None, var=2 * (acc + chain["carried"]), virt=virt,""",
         """                    pending=pending, n_first=None, var=2 * (acc + chain["carried"]), virt=virt, nadir=self._nadir(img, t0, att),"""),
        ("""    def _activate(self, kf, tref):""",
         """    def _nadir(self, img, t0, att):
        \"\"\"Pixels of the whole-picture size whose ray is within DENSE_NADIR of straight down (bool), at this frame's
        attitude (integ/diag10).\"\"\"
        s = self.DENSE_SCALE
        hs, ws = round(img.shape[0] * s), round(img.shape[1] * s)
        ys, xs = np.mgrid[0:hs, 0:ws].astype(np.float64)
        q = np.c_[xs.ravel(), ys.ravel()] / s
        f = np.c_[self._undistort(q), np.ones(len(q))] @ self.R_bc.T
        b = np.einsum("nij,nj->ni", attitude_R(att, t0 + q[:, 1] / img.shape[0] * self.readout), f)
        return (b[:, 2] > math.cos(self.DENSE_NADIR) * np.linalg.norm(b, axis=1)).reshape(hs, ws)

    def _activate(self, kf, tref):"""),
        ("""            a = self._small(kf["img"])
            v = np.isfinite(a)
            a[~v] = float(a[v].mean())""",
         """            a = self._small(kf["img"])
            v = np.isfinite(a) & kf["nadir"] if "nadir" in kf else np.isfinite(a)
            a[~v] = float(a[v].mean())"""),
        ("""        if v.mean() < 0.5:
            return "whole picture: keyframe out of view\"""",
         """        if v.sum() < 0.5 * z["v"].sum():
            return "whole picture: keyframe out of view\""""),
        ("""    DENSE_SIG_PX = 0.15 """,
         """    DENSE_NADIR = math.radians(55)  # whole-picture path: only pixels within this of straight down (integ/diag10)
    DENSE_SIG_PX = 0.15 """),
    ],
    "yaw": [
        # the warp gets a third column: px per rad of turn about the vertical through the drone
        ("""        P = np.linalg.lstsq(_cubic(kf["px"][idx], img.shape), np.c_[guess - kf["px"][idx], J.reshape(-1, 4)], rcond=None)[0]
        M = (z["B"] @ P).astype(np.float32).reshape(z["a"].shape + (6,)) * s
        base, JN, JE = z["xy"] + M[..., :2], M[..., [2, 4]], M[..., [3, 5]]     # small px; small px per m""",
         """        Jy = self._Jy if getattr(self, "_Jy", None) is not None else np.zeros((len(idx), 2))
        P = np.linalg.lstsq(_cubic(kf["px"][idx], img.shape), np.c_[guess - kf["px"][idx], J.reshape(-1, 4), Jy], rcond=None)[0]
        M = (z["B"] @ P).astype(np.float32).reshape(z["a"].shape + (8,)) * s
        base, JN, JE = z["xy"] + M[..., :2], M[..., [2, 4]], M[..., [3, 5]]     # small px; small px per m
        JY = M[..., 6:8]                                                          # small px per rad of yaw"""),
        ("""        def warp(dl):     # warp(dl)(x) = current(map(x) + J(x) dl), NaN outside the picture and on the mask
            m = base + JN * np.float32(dl[0]) + JE * np.float32(dl[1])""",
         """        def warp(dl):     # warp(dl)(x) = current(map(x) + J(x) dl[:2] + Jy(x) dl[2]), NaN outside the picture and on the mask
            m = base + JN * np.float32(dl[0]) + JE * np.float32(dl[1]) + JY * np.float32(dl[2] if len(dl) > 2 else 0.0)"""),
        ("""        jN = z["gx"] * JN[..., 0] + z["gy"] * JN[..., 1]   # grey per m of drone move north (keyframe gradients)
        jE = z["gx"] * JE[..., 0] + z["gy"] * JE[..., 1]""",
         """        jN = z["gx"] * JN[..., 0] + z["gy"] * JN[..., 1]   # grey per m of drone move north (keyframe gradients)
        jE = z["gx"] * JE[..., 0] + z["gy"] * JE[..., 1]
        jY = z["gx"] * JY[..., 0] + z["gy"] * JY[..., 1]   # grey per rad of yaw"""),
        ("""        Jc = P[0, 2:].reshape(2, 2)                      # full px per m at the picture centre (columns N, E)""",
         """        Jc = P[0, 2:6].reshape(2, 2)                     # full px per m at the picture centre (columns N, E)"""),
        ("""        dl0 = np.linalg.solve(s * Jc, np.array(d))       # the shift at the centre, as a drone move""",
         """        dl0 = np.r_[np.linalg.solve(s * Jc, np.array(d)), 0.0]   # the shift at the centre, as a drone move; no turn"""),
        ("""                e, j1, j2 = (w[rows] - z["a"][rows])[v], jN[rows][v], jE[rows][v]
                A = np.array([[j1 @ j1, j1 @ j2], [j1 @ j2, j2 @ j2]], float)
                if len(e) < 100 or np.linalg.eigvalsh(A)[0] < 1e-6 * len(e):
                    return "whole picture: too flat"
                dl = dl - np.linalg.solve(A, [j1 @ e, j2 @ e])
            ds.append(dl)
            cov.append(float(e @ e) / len(e) * np.linalg.inv(A))""",
         """                e, j1, j2, j3 = (w[rows] - z["a"][rows])[v], jN[rows][v], jE[rows][v], jY[rows][v]
                A3 = np.array([[j1 @ j1, j1 @ j2, j1 @ j3], [j1 @ j2, j2 @ j2, j2 @ j3], [j1 @ j3, j2 @ j3, j3 @ j3]], float)
                A = A3[:2, :2]
                if len(e) < 100 or np.linalg.eigvalsh(A)[0] < 1e-6 * len(e):
                    return "whole picture: too flat"
                if np.linalg.eigvalsh(A3)[0] > 1e-9:        # the turn is measurable (texture off the turn axis)
                    dl = dl - np.linalg.solve(A3, [j1 @ e, j2 @ e, j3 @ e])
                else:
                    dl = dl - np.r_[np.linalg.solve(A, [j1 @ e, j2 @ e]), 0.0]
            ds.append(dl[:2])
            dys.append(dl[2])
            cov.append(float(e @ e) / len(e) * np.linalg.inv(A))"""),
        ("""        half, ds, cov = z["a"].shape[0] // 2, [], []""",
         """        half, ds, cov, dys = z["a"].shape[0] // 2, [], [], []"""),
        ("""        half, ds, cov, P = z["a"].shape[0] // 2, [], [], 0.0""",
         """        half, ds, cov, P, dys = z["a"].shape[0] // 2, [], [], 0.0, []"""),
        ("""        dl = (ds[0] + ds[1]) / 2
        w, v = warp(dl)""",
         """        dl = np.r_[(ds[0] + ds[1]) / 2, (dys[0] + dys[1]) / 2]
        w, v = warp(dl)"""),
        ("""        p2 = guess + J @ dl
        lk = np.isfinite(p_lk[:, 0])""",
         """        p2 = guess + J @ dl[:2] + Jy * dl[2]
        self.dyaw = float(dl[2])
        lk = np.isfinite(p_lk[:, 0])"""),
        # Jy per point: px per rad of yaw about the vertical through the drone, at the current attitude
        ("""            r = self._dense(kf, img, idx, guess, J, p_lk)""",
         """            c_, s_ = math.cos(1e-3), math.sin(1e-3)
            Rz = np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])
            self._Jy = (self._project(v @ Rz.T, Rt) - self._project(v @ Rz, Rt)) / 2e-3
            r = self._dense(kf, img, idx, guess, J, p_lk)
            self._Jy = None"""),
    ],
}


def main():
    src, out, *names = sys.argv[1:]
    s = open(src).read()
    deg = None
    for n in names:
        if n.startswith("nadir_deg="):
            deg = float(n.split("=")[1])
            continue
        for old, new in PARTS[n]:
            k = s.count(old)
            if k == 0 and n == "yaw" and "half, ds, cov" in old:
                continue                       # one of the two forms (base or patched) is present
            if k != 1:
                raise SystemExit(f"{n}: text found {k} times: {old[:70]!r}")
            s = s.replace(old, new)
    if deg is not None:
        s = s.replace("DENSE_NADIR = math.radians(55)", f"DENSE_NADIR = math.radians({deg:g})")
    open(out, "w").write(s)
    print("wrote", out)


if __name__ == "__main__":
    main()
