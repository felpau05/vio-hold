#!/usr/bin/env python3
"""Build hold.py variants for the low-height study from the repository's hold.py (text replacements, each must match once).

Parts (combine them by name, e.g. geo+local):
  geo     whole-picture warp taken at the last fix (Hold.pos) instead of the keyframe position, with J from a central
          step of 1 % of the height instead of a 1 m forward step (Hold._track1)
  geo2    as geo, but only J: the warp keeps its old start (the attitude prediction plus the reference frame's offset)
  local   halves gap measured in px of each half's own texture instead of at the picture centre's scale (Hold._dense)
  mask55  the whole-picture path uses only pixels within MAX_OFF_NADIR (55 deg) of the optical axis, where its cubic
          map is fitted (every keyframe point lies within it, _make_kf); 'keyframe out of view' then compares with the
          keyframe's own usable share (Hold._dense)
Run: ~/.venvs/vio/bin/python integ/lowh/make_variants.py OUTDIR geo local mask55 geo+local geo+local+mask55 ...
Writes OUTDIR/hold_<name>.py (name with + replaced by _).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))

PARTS = {
    "geo": [(
        """            v = np.c_[kf["G"][idx] - kf["pos"], ht]       # px per m of drone move north and east, per point
            J = np.stack([self._project(v - [1.0, 0, 0], Rt) - pred, self._project(v - [0, 1.0, 0], Rt) - pred], -1)
            r = self._dense(kf, img, idx, guess, J, p_lk)""",
        """            # The warp is linear in the drone move, so take it where the drone is now: at the last fix (self.pos), with
            # a central step of 1 % of the height (px per m of drone move north and east, per point). Taken at the
            # keyframe with a 1 m forward step, it missed how a fisheye's image motion changes with the move (integ/lowh).
            v, e = np.c_[kf["G"][idx] - self.pos, ht], 0.01 * ht[:, None]
            at = self._project(v, Rt)
            J = np.stack([self._project(v - e * [1.0, 0, 0], Rt) - self._project(v + e * [1.0, 0, 0], Rt),
                          self._project(v - e * [0, 1.0, 0], Rt) - self._project(v + e * [0, 1.0, 0], Rt)], -1) / (2 * e[:, :, None])
            r = self._dense(kf, img, idx, at, J, p_lk)""")],
    "geo2": [(
        """            v = np.c_[kf["G"][idx] - kf["pos"], ht]       # px per m of drone move north and east, per point
            J = np.stack([self._project(v - [1.0, 0, 0], Rt) - pred, self._project(v - [0, 1.0, 0], Rt) - pred], -1)""",
        """            # px per m of drone move north and east, per point: taken where the drone is now (the last fix, self.pos),
            # with a central step of 1 % of the height, not at the keyframe with a 1 m forward step (integ/lowh)
            v, e = np.c_[kf["G"][idx] - self.pos, ht], 0.01 * ht[:, None]
            J = np.stack([self._project(v - e * [1.0, 0, 0], Rt) - self._project(v + e * [1.0, 0, 0], Rt),
                          self._project(v - e * [0, 1.0, 0], Rt) - self._project(v + e * [0, 1.0, 0], Rt)], -1) / (2 * e[:, :, None])""")],
    "local": [(
        """        half, ds, cov = z["a"].shape[0] // 2, [], []""",
        """        half, ds, cov, P = z["a"].shape[0] // 2, [], [], 0.0"""), (
        """            cov.append(float(e @ e) / len(e) * np.linalg.inv(A))
        gap = float(np.linalg.norm(Jc @ (ds[0] - ds[1])))""",
        """            cov.append(float(e @ e) / len(e) * np.linalg.inv(A))
            P = P + np.linalg.inv(A) * float(np.sum(z["gx"][rows][v] ** 2 + z["gy"][rows][v] ** 2)) / 2   # m^2 per px^2
        # The gap in px of each half's own texture, not at the picture centre's scale (equal when J is the same
        # everywhere). A fisheye moves its edge less per metre, so a picture shift of 1-3 px (camera vibration the FC
        # does not see) in texture near the edge read as 4-8 px at the centre's scale (5 m, coarse ground; integ/lowh).
        dd = ds[0] - ds[1]
        gap = math.sqrt(2 * float(dd @ np.linalg.solve(P, dd))) / s""")],
    "mask55": [(
        """            a = self._small(kf["img"])
            v = np.isfinite(a)
            a[~v] = float(a[v].mean())
            ys, xs = np.mgrid[0:a.shape[0], 0:a.shape[1]].astype(np.float32)""",
        """            a = self._small(kf["img"])
            ys, xs = np.mgrid[0:a.shape[0], 0:a.shape[1]].astype(np.float32)
            # only where the cubic map is fitted: every keyframe point lies within MAX_OFF_NADIR (_make_kf), and beyond it
            # the extrapolated map is off by pixels that grow with the drift (integ/lowh)
            n = self._undistort(np.c_[xs.ravel(), ys.ravel()] / self.DENSE_SCALE)
            v = np.isfinite(a) & (np.hypot(n[:, 0], n[:, 1]) < math.tan(self.MAX_OFF_NADIR)).reshape(a.shape)
            a[~v] = float(a[v].mean())"""), (
        """        if v.mean() < 0.5:
            return "whole picture: keyframe out of view\"""",
        """        if v.sum() < 0.5 * z["v"].sum():
            return "whole picture: keyframe out of view\"""")],
}


def build(names, src):
    for p in names:
        for old, new in PARTS[p]:
            if src.count(old) != 1:
                raise SystemExit(f"part {p}: the text to replace is found {src.count(old)} times")
            src = src.replace(old, new)
    return src


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    src = open(os.path.join(PROJ, "hold.py")).read()
    for v in sys.argv[2:]:
        p = os.path.join(out, f"hold_{v.replace('+', '_')}.py")
        open(p, "w").write(build(v.split("+"), src))
        print(p)


if __name__ == "__main__":
    main()
