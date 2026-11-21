"""Shared parts of the low-height study (integ/lowh): load a hold.py by path, the Justice D camera, the grounds.

Pictures come from px4sim/camera.py (its own geometry, not hold.py's). Hold gets the lens a hand-turn calibration
can give (camera.calib_lens of the true lens: fx +0.5 %, fy -0.5 %, k1 +0.01, centre at the picture centre), as
integ/justice_d/open_loop.py and the SITL runs do.

Grounds:
  ottawa2022  photo, 0.293 m per photo pixel (VALIDATION site; the SITL runs used ottawa2019fall, the same scale)
  farmland    photo, 0.586 m per photo pixel (VALIDATION site)
  lowh_fine   NEW synthetic texture made here (not a tuning or hold-out site): multi-scale noise from 6 mm to
              about 25 m, 6 mm per texture pixel (8192 x 8192 = 49 m square, mirrored beyond). One texture pixel is
              smaller than one sensor pixel at 5 m (crop lens: 7.9 mm, full-sensor lens: 10.3 mm). Grey 120,
              std 25 (assumed: a gravel or dry-grass look).
"""
import importlib.util
import json
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(PROJ, "px4sim")]
import camera as C   # noqa: E402

LENSES = {"crop": "sim Justice D near crop", "full": "sim Justice D near full sensor"}
HOLD_CFG = {"repeat_diff": 0.01, "h_min": 2.0}     # tools/profiles/justice_d.json hold_cfg + the SITL cases' h_min
JD = dict(video="JUSTICE_D", readout_s=0.014, exposure_s=0.004, noise=1.0, noise_corr_px=0.0, vib_deg=0.2,
          focal_scale=1.0, pp_shift=(0.0, 0.0), cam_top="front", cam_mirror=False, cam_tilt=(0.0, 0.0),
          cam_twist=0.0)                           # px4sim/sitl.py DEFAULTS + VIDEO_DEFAULTS["JUSTICE_D"]
FINE_CACHE = os.path.join(os.environ.get("LOWH_CACHE", tempfile.gettempdir()), "lowh_fine_8192_6mm.npy")


def load_hold(path, name):
    """hold.py at path as a module called name (two versions can live side by side)."""
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def fine_ground():
    if os.path.exists(FINE_CACHE):
        img = np.load(FINE_CACHE)
    else:
        rng = np.random.default_rng(20261119)
        t = C._octaves(rng, 8192, 1, 13, 0.1)
        img = np.clip(120.0 + 25.0 * t, 0, 255).astype(np.uint8)
        np.save(FINE_CACHE, img)
    return C.Ground(img, 0.006)


def ground(name):
    if name == "lowh_fine":
        return fine_ground()
    if name not in ("ottawa2022", "farmland"):
        raise SystemExit("grounds: ottawa2022, farmland (VALIDATION) or lowh_fine (made here)")
    return C.photo_ground(name)


def camera(ground_name, lens="crop", seed=11, vib=0.2, gr=None):
    c = dict(JD, true_lens=LENSES[lens], seed=seed, vib_deg=vib)
    return C.Camera(c, gr if gr is not None else ground(ground_name)), c


def est_lens(holdmod, cam, c, exact=False):
    """K, D for Hold at the picture size: calib-like lens (default) or the true lens (diagnostic)."""
    if exact:
        return C.lens(c["true_lens"], cam.W, cam.H)
    m = C.calib_lens(c["true_lens"])
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "lens.json")
        json.dump({"models": [m]}, open(p, "w"))
        return holdmod.load_lens(p, m["name"], cam.W, cam.H)


def quat(roll, pitch, yaw):
    E_N, E_E, E_D = np.eye(3)
    return C.qmul(C.q_axis(E_D, yaw), C.qmul(C.q_axis(E_E, pitch), C.q_axis(E_N, roll)))


def load1():
    return float(open("/proc/loadavg").read().split()[0])


def local_gap(AB, ds, s):
    """Halves gap in px where each half has its texture (candidate metric): sqrt(2 d^T (P_top + P_bottom)^-1 d) / s with
    d = dl_top - dl_bottom and P = A^-1 x (gradient energy) / 2, the m^2 per px^2 of the half's own texture. It equals
    |Jc d| when both halves see the same J = Jc everywhere."""
    if AB is None or len(AB) != 2:
        return None
    P = sum(np.linalg.inv(A) * (b / 2) for A, b in AB)
    d = np.asarray(ds[0]) - np.asarray(ds[1])
    return float(math.sqrt(2 * d @ np.linalg.solve(P, d)) / s)


def capture_ds(holdmod):
    """Instrument Hold._dense of this module: after the halves gap is computed, keep the two halves' drone moves
    (self._ds, m north/east) and the centre scale (self._Jc). The source of hold.py is not changed: the method is
    re-made from its own text with one line added after 'self.dq[1] = gap'."""
    import inspect
    import textwrap
    src = textwrap.dedent(inspect.getsource(holdmod.Hold._dense))
    key = "self.dq[1] = gap\n"
    assert src.count(key) == 1, "hold.py _dense changed: the instrument line has no anchor"
    i = src.index(key) + len(key)
    indent = src[:src.index(key)].split("\n")[-1]
    src = src[:i] + indent + "self._ds, self._Jc = [x.copy() for x in ds], Jc.copy()\n" + src[i:]
    # per half: the LK normal matrix A (grey^2 per m^2) and the gradient energy of the half (grey^2 per small px^2)
    key2 = "cov.append("
    if src.count(key2) == 1:
        j = src.index(key2)
        ind2 = src[:j].split("\n")[-1]
        line_end = src.index("\n", j) + 1
        src = (src[:line_end] + ind2 + "self._AB.append((A.copy(), float(np.sum(z['gx'][rows][v] ** 2 + z['gy'][rows][v] ** 2))))\n"
               + src[line_end:])
        k = src.index("half, ds, cov")
        ind3 = src[:k].split("\n")[-1]
        src = src[:k] + "self._AB = []\n" + ind3 + src[k:]
    ns = {}
    exec(compile(src, holdmod.__file__ + ":_dense(instrumented)", "exec"), holdmod.__dict__, ns)
    holdmod.Hold._dense = ns["_dense"]
