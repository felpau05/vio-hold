"""Downward camera for the closed-loop PX4 test, rendered from the TRUE pose.

The geometry here is written from first principles (adapted from review/geometry/indep.py). It does
NOT import euler_R, R_BC_NADIR, make_R_bc or RAYS from hold.py or sim_test.py, so a frame or mount
convention error in hold.py shows up as a closed-loop error instead of cancelling out.

Two kinds of camera (c["video"]):
- analog (VIDEO: PAL, NTSC): one field, width halved, analog noise correlated along x, break-up possible.
- digital (DIGITAL: JUSTICE_D): the whole sensor picture is rendered (1280x720 progressive, rolling shutter
  per sensor row), white sensor noise, 8-bit, then shrunk by the capture's whole factor with cv2.INTER_AREA
  (as capture.DigitalSource does with video_reduce). No break-up. Its true lens is one of TRUE_LENSES:
  these live only here, in no lens file, so the estimator cannot read them.

Conventions: NED world, FRD body, camera x = image right, y = image down, z = optical axis.
Quaternions are (w, x, y, z) and rotate body vectors into NED.
"""
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
LENS_JSON = os.path.join(PROJ, "justice_lens_profiles.json")
E_N, E_E, E_D = np.eye(3)


# ------------------------------------------------------------------ rotations
def q_axis(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[math.cos(ang / 2), axis * math.sin(ang / 2)]


def qmul(p, q):
    w1, x1, y1, z1 = np.moveaxis(np.asarray(p, float), -1, 0)
    w2, x2, y2, z2 = np.moveaxis(np.asarray(q, float), -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)


def qrot(q, v):
    """Active rotation of v by unit quaternion q (q v q*)."""
    q = np.asarray(q, float)
    w, u = q[..., :1], q[..., 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def q_matrix(q):
    """3x3 matrices whose columns are the rotated N, E, D unit vectors (body -> NED)."""
    q = np.asarray(q, float)
    return np.stack([qrot(q, np.broadcast_to(e, q.shape[:-1] + (3,))) for e in (E_N, E_E, E_D)], -1)


def q_roll_pitch(q):
    """Roll and pitch (rad) from the physical meaning: pitch = nose elevation above the horizon,
    roll = how far the right wing has gone below the horizon, measured in the body's own plane."""
    fwd = qrot(q, np.broadcast_to(E_N, np.shape(q)[:-1] + (3,)))
    right = qrot(q, np.broadcast_to(E_E, np.shape(q)[:-1] + (3,)))
    down = qrot(q, np.broadcast_to(E_D, np.shape(q)[:-1] + (3,)))
    pitch = np.arcsin(np.clip(-fwd[..., 2], -1, 1))
    roll = np.arctan2(right[..., 2], down[..., 2])
    return roll, pitch


def q_yaw(q):
    """Heading of the nose projected on the horizontal plane (rad, 0 = north, + toward east)."""
    fwd = qrot(q, np.broadcast_to(E_N, np.shape(q)[:-1] + (3,)))
    return np.arctan2(fwd[..., 1], fwd[..., 0])


TOP = {"front": E_N, "right": E_E, "rear": -E_N, "left": -E_E}   # body FRD directions


def mount(top="front", mirror=False, tilt_deg=(0.0, 0.0), twist_deg=0.0):
    """Camera -> body matrix (columns: camera x, y, z written in body FRD).

    The optical axis points along body down. 'top' is the body direction that the top edge of the
    picture points to, so image down is the opposite. Image right = image down x optical axis keeps
    the camera frame right-handed; a mirrored picture flips image right.
    twist: right-hand turn of the camera about its own optical axis.
    tilt (roll, pitch): the mount is tilted the way a drone is by that roll and pitch: first a turn
    about body forward by roll, then about body right by pitch (pitch + = camera looks forward).
    """
    z = E_D.copy()
    y = -np.asarray(TOP[top], float)
    x = np.cross(y, z) * (-1.0 if mirror else 1.0)
    qt = q_axis(z, math.radians(twist_deg))
    x, y = qrot(qt, x), qrot(qt, y)
    qm = qmul(q_axis(E_E, math.radians(tilt_deg[1])), q_axis(E_N, math.radians(tilt_deg[0])))
    return np.column_stack([qrot(qm, x), qrot(qm, y), qrot(qm, z)])


# ------------------------------------------------------------------ lens (equidistant fisheye)
# True lenses of the Justice D (digital, 1280x720), for the simulator only. The real lens is not known:
# justice_lens_profiles.json holds three assumed profiles (full sensor fx = fy = 482.5, crop fx = fy = 643.3,
# anamorphic fx 643.3 / fy 482.5). Each true lens here is near one of them but is none of them: fx and fy differ
# by 1-2 %, k1 is -0.035 or -0.018, k2 is small, and the centre is about 11 px from the picture centre.
# All numbers are chosen (assumed), not measured.
TRUE_LENSES = {
    # fx +1.5 %, fy -0.25 % against 482.5 (fx / fy 1.017); centre (+9, -6) px
    "sim Justice D near full sensor": dict(projection="equidistant_fisheye", image_size=[1280, 720], fx=489.7,
                                           fy=481.3, cx=648.5, cy=353.5, k1=-0.035, k2=0.003, k3=0.0, k4=0.0),
    # fx -1.3 %, fy +0.3 % against 643.3 (fy / fx 1.016); centre (-7, +8) px
    "sim Justice D near crop": dict(projection="equidistant_fisheye", image_size=[1280, 720], fx=634.9,
                                    fy=645.1, cx=632.5, cy=367.5, k1=-0.018, k2=-0.002, k3=0.0, k4=0.0),
}


def profile(name):
    """A lens profile: a dict as given, one of TRUE_LENSES, or a profile of justice_lens_profiles.json."""
    if isinstance(name, dict):
        return name
    if name in TRUE_LENSES:
        return TRUE_LENSES[name]
    return next(m for m in json.load(open(LENS_JSON))["models"] if m["name"] == name)


def calib_lens(true_name, dfx=0.005, dfy=-0.005, dk1=0.01, name=None):
    """The lens a hand-turn calibration (calib.solve fit_lens) can give for a true lens, as a lens-file profile:
    fx and fy off by dfx and dfy (0.5 %), k1 off by dk1, no k2-k4, the centre at the picture centre (calib does
    not fit the centre). The error signs are a choice (assumed), the sizes are those of the task."""
    p = profile(true_name)
    w, h = p["image_size"]
    return dict(name=name or f"calib-like {true_name}", projection="equidistant_fisheye", image_size=[w, h],
                fx=p["fx"] * (1 + dfx), fy=p["fy"] * (1 + dfy), cx=(w - 1) / 2, cy=(h - 1) / 2,
                b2=0.0, p1=0.0, p2=0.0, k1=p["k1"] + dk1, k2=0.0, k3=0.0, k4=0.0,
                note=f"simulator: {true_name} with fx {dfx:+.1%}, fy {dfy:+.1%}, k1 {dk1:+.3f}, no k2, centre at the picture centre")


def lens(name, w, h, focal_scale=1.0, pp_shift=(0.0, 0.0)):
    """K, D for a w x h picture with the profile's picture extent. Pixel centres are integers, so
    an edge coordinate u + 0.5 scales with the picture."""
    p = profile(name)
    sx, sy = w / p["image_size"][0], h / p["image_size"][1]
    K = np.array([[p["fx"] * sx * focal_scale, 0, (p["cx"] + 0.5) * sx - 0.5 + pp_shift[0]],
                  [0, p["fy"] * sy * focal_scale, (p["cy"] + 0.5) * sy - 0.5 + pp_shift[1]],
                  [0, 0, 1.0]])
    # shortcut: tangential terms p1, p2 of a profile are ignored (|p| < 4e-4 in the Justice profiles).
    return K, np.array([p.get(k, 0.0) for k in ("k1", "k2", "k3", "k4")], float)


def _poly(th, D):
    t2 = th * th
    return th * (1 + D[0] * t2 + D[1] * t2 ** 2 + D[2] * t2 ** 3 + D[3] * t2 ** 4)


def project(xyz, K, D):
    """Camera-frame points -> pixels. Distorted radius = polynomial in the incidence angle."""
    X, Y, Z = np.moveaxis(np.asarray(xyz, float), -1, 0)
    th, ph = np.arctan2(np.hypot(X, Y), Z), np.arctan2(Y, X)
    td = _poly(th, D)
    return K[0, 0] * td * np.cos(ph) + K[0, 2], K[1, 1] * td * np.sin(ph) + K[1, 2]


def unproject(u, v, K, D):
    """Pixels -> unit rays in the camera frame (Newton on the incidence angle)."""
    xd, yd = (np.asarray(u, float) - K[0, 2]) / K[0, 0], (np.asarray(v, float) - K[1, 2]) / K[1, 1]
    td, ph = np.hypot(xd, yd), np.arctan2(yd, xd)
    th = td.copy()
    for _ in range(30):
        t2 = th * th
        th = th - (_poly(th, D) - td) / (1 + 3 * D[0] * t2 + 5 * D[1] * t2 ** 2 + 7 * D[2] * t2 ** 3 + 9 * D[3] * t2 ** 4)
    return np.stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)], -1)


# ------------------------------------------------------------------ ground
class Ground:
    """A flat ground texture centred on the take-off point (north up), with a pyramid so that
    the level used matches the ground size of one camera pixel."""

    def __init__(self, img, m_per_px):
        self.pyr, self.mpp = [img], m_per_px
        while self.pyr[-1].shape[0] > 256:
            self.pyr.append(cv2.pyrDown(self.pyr[-1]))

    def sample(self, N, E, gsd):
        lvl = min(len(self.pyr) - 1, max(0, int(math.floor(math.log2(max(gsd, 1e-6) / self.mpp)))))
        tex, m = self.pyr[lvl], 1.0 / (self.mpp * 2 ** lvl)
        return cv2.remap(tex, (E * m + tex.shape[1] / 2).astype(np.float32),
                         (-N * m + tex.shape[0] / 2).astype(np.float32), cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REFLECT101).astype(np.float32)


def photo_ground(name):
    sites = json.load(open(os.path.join(PROJ, "ground", "sites.json")))
    img = cv2.imread(os.path.join(PROJ, "ground", name + ".jpg"), cv2.IMREAD_GRAYSCALE)
    return Ground(img, sites[name]["m_per_px"])


def snow_ground(tarps=True, helipad=True, spacing_m=15.0, texture=1.5, seed=3):
    """Plain snow. texture: grey-level std of the snow's own surface (assumed: nearly featureless).
    tarps: three 2 x 3 m tarps in an L, spacing_m apart. helipad: a 1.5 m 'H' pad at the hold point.
    Colours: assumed grey levels for a blue tarp (90) and a dark pad (50) on snow (235)."""
    n, mpp = 4096, 0.125                                  # 512 m square
    rng = np.random.default_rng(seed)
    t = np.zeros((n, n), np.float32)
    for k in range(4, 11):
        t += cv2.resize(rng.standard_normal((2 ** k, 2 ** k)).astype(np.float32), (n, n),
                        interpolation=cv2.INTER_CUBIC)
    img = 235.0 + texture * t / t.std()

    def px(N, E):
        return int(round(E / mpp + n / 2)), int(round(-N / mpp + n / 2))
    if tarps:
        for N, E in ((spacing_m, 0.0), (spacing_m, spacing_m), (0.0, spacing_m)):
            (x0, y0), (x1, y1) = px(N + 1.5, E - 1.0), px(N - 1.5, E + 1.0)
            img[y0:y1, x0:x1] = 90.0
    if helipad:
        c, r = px(0, 0), int(0.75 / mpp)
        cv2.circle(img, c, r, 50.0, -1)
        s = int(0.25 / mpp)
        cv2.rectangle(img, (c[0] - 2 * s, c[1] - 2 * s), (c[0] - s, c[1] + 2 * s), 235.0, -1)
        cv2.rectangle(img, (c[0] + s, c[1] - 2 * s), (c[0] + 2 * s, c[1] + 2 * s), 235.0, -1)
        cv2.rectangle(img, (c[0] - s, c[1] - s // 2), (c[0] + s, c[1] + s // 2), 235.0, -1)
    return Ground(np.clip(img, 0, 255).astype(np.uint8), mpp)


def night_ground(spacing_m=15.0, texture=1.0, seed=3):
    """Unlit ground at night with six point lights in an L (four along north, three along east, the corner
    shared), spacing_m apart, around the hold point. Grey levels assumed: ground 6 with texture std 1, a
    light 1 m across at 255 with a glow of 120 (sigma 1 m: the bloom of a lamp that saturates). Use with a longer exposure
    and more noise, e.g. exposure_s=0.02 noise=10."""
    n, mpp = 4096, 0.125
    rng = np.random.default_rng(seed)
    t = cv2.resize(rng.standard_normal((256, 256)).astype(np.float32), (n, n), interpolation=cv2.INTER_CUBIC)
    img = 6.0 + texture * t / t.std()
    lights = [(-15.0 + spacing_m * k, -15.0) for k in range(4)] + [(-15.0, -15.0 + spacing_m * k) for k in (1, 2)]
    ys, xs = np.mgrid[-24:25, -24:25] * mpp
    glow = 120.0 * np.exp(-(xs ** 2 + ys ** 2) / (2 * 1.0 ** 2))
    glow[xs ** 2 + ys ** 2 <= 0.5 ** 2] = 255.0
    for N, E in lights:
        x, y = int(round(E / mpp + n / 2)), int(round(-N / mpp + n / 2))
        img[y - 24:y + 25, x - 24:x + 25] = np.maximum(img[y - 24:y + 25, x - 24:x + 25], glow)
    return Ground(np.clip(img, 0, 255).astype(np.uint8), mpp)


def _octaves(rng, n, k_lo, k_hi, amp_exp=0.0):
    """Sum of upsampled random grids 2^k_lo .. 2^k_hi cells across (multi-scale noise), unit std."""
    t = np.zeros((n, n), np.float32)
    for k in range(k_lo, k_hi + 1):
        t += (2.0 ** (-amp_exp * k)) * cv2.resize(rng.standard_normal((2 ** k, 2 ** k)).astype(np.float32), (n, n),
                                                 interpolation=cv2.INTER_CUBIC)
    return t / max(float(t.std()), 1e-6)


FLOOR_MPP, FLOOR_N = 0.004, 4096   # 4 mm texels, 16.4 m square (mirrored beyond); 2 m height sees 1 cm per pixel


def indoor_floor(kind, seed=5):
    """Indoor floors with centimetre detail. Every grey level, size and contrast below is assumed (typical
    look of such floors on a small analog camera), not measured.
    TUNING floors: concrete, tiles, carpet, wood, shiny (shiny also needs reflect=True for lamp reflections).
    HOLD-OUT floors (final numbers only, never used to choose a setting): holdout_floor_a, holdout_floor_b."""
    n, mpp = FLOOR_N, FLOOR_MPP
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32) * mpp           # metres from the texture corner
    if kind == "concrete":        # trowelled concrete: blotches from 5 cm to 1 m, aggregate speckle, a few cracks
        img = 140.0 + 7.0 * _octaves(rng, n, 4, 9, 0.3) + 5.0 * _octaves(rng, n, 10, 11)
        for _ in range(12):
            p = rng.uniform(0, n, 2)
            for _ in range(400):
                q = p + rng.normal(0, 3.0, 2) + rng.uniform(-1, 1, 2) * 2
                cv2.line(img, tuple(int(v) for v in p), tuple(int(v) for v in q), 95.0, 1)
                p = q
    elif kind == "tiles":         # 30 cm ceramic tiles, 5 mm grout: a regular grid of identical corners
        T, g = 0.30, 0.005
        tid = (np.floor(xx / T) * 997 + np.floor(yy / T)).astype(np.int64)
        lvl = rng.normal(0, 6.0, 4096)[tid % 4096]
        img = 185.0 + lvl + 2.5 * _octaves(rng, n, 9, 11)
        img[((xx % T) < g) | ((yy % T) < g)] = 80.0
    elif kind == "carpet":        # loop-pile fibre (2-4 mm) plus a 25 cm diamond pattern
        img = 110.0 + 12.0 * _octaves(rng, n, 10, 11) + 4.0 * _octaves(rng, n, 4, 7)
        img += 9.0 * np.sign(np.sin(2 * np.pi * (xx + yy) / 0.25) * np.sin(2 * np.pi * (xx - yy) / 0.25))
    elif kind == "wood":          # 12 cm planks, 0.8-1.5 m long and staggered, grain along the plank
        W = 0.12
        row = np.floor(yy / W).astype(np.int64)
        off = rng.uniform(0, 1.5, 4096)[row % 4096]
        L = rng.uniform(0.8, 1.5, 4096)[row % 4096]
        pid = row * 131 + np.floor((xx + off) / L).astype(np.int64)
        grain = cv2.resize(rng.standard_normal((n // 2, 64)).astype(np.float32), (n, n), interpolation=cv2.INTER_CUBIC)
        img = 150.0 + rng.normal(0, 14.0, 8192)[pid % 8192] + 8.0 * grain / grain.std() + 2.0 * _octaves(rng, n, 10, 11)
        img[((yy % W) < 0.003) | (((xx + off) % L) < 0.003)] = 70.0
    elif kind == "shiny":         # polished sealed concrete: very little texture (the reflections come from render)
        img = 120.0 + 2.5 * _octaves(rng, n, 4, 11, 0.2)
    elif kind == "holdout_floor_a":   # HOLD-OUT: terrazzo, stone chips 3-15 mm in a light binder
        img = 170.0 + 3.0 * _octaves(rng, n, 6, 10)
        for _ in range(60000):
            x, y = rng.integers(0, n, 2)
            cv2.circle(img, (int(x), int(y)), int(rng.integers(1, 4)), float(rng.choice([60.0, 100.0, 220.0, 130.0])), -1)
    elif kind == "holdout_floor_b":   # HOLD-OUT: sports hall vinyl, mottled, with painted court lines and scuffs
        img = 135.0 + 4.0 * _octaves(rng, n, 5, 10, 0.2)
        for k in range(-3, 4):
            img[np.abs(xx - n * mpp / 2 - 2.3 * k) < 0.025] = 225.0
        cv2.circle(img, (n // 2 + 300, n // 2 - 200), int(1.8 / mpp), 225.0, int(0.05 / mpp))
        for _ in range(300):
            p = rng.uniform(0, n, 2)
            q = p + rng.normal(0, 40, 2)
            cv2.line(img, tuple(int(v) for v in p), tuple(int(v) for v in q), 95.0, 2)
    else:
        raise ValueError(f"unknown floor {kind!r}")
    return Ground(np.clip(img, 0, 255).astype(np.uint8), mpp)


def make_ground(name):
    """'snow', 'snow_tarps', 'snow_tarps_pad', 'night_lights', 'floor_<kind>' (indoor_floor), 'holdout_floor_a/b',
    or a photo site in ground/."""
    if name.startswith("snow"):
        return snow_ground(tarps="tarps" in name, helipad="pad" in name)
    if name.startswith("night"):
        return night_ground()
    if name.startswith("floor_"):
        return indoor_floor(name[6:])
    if name.startswith("holdout_floor"):
        return indoor_floor(name)
    return photo_ground(name)


# ------------------------------------------------------------------ OSD
def osd_layout(w, h):
    """Text items (text, x, y) of a typical FPV OSD, scaled to the picture. With font scale 0.85 the mask
    covers 17-20 % of the picture (measured on the real video: 19 %)."""
    s = w / 360
    return [("13.1V", 6 * s, 24 * s), ("RSSI 99", w / 2 - 50 * s, 24 * s), ("TIME", w - 90 * s, 24 * s),
            ("ALT", 6 * s, h / 2), ("SPD 0", w - 90 * s, h / 2), ("1.2A 450mAh", 6 * s, h - 10 * s),
            ("ACRO", w / 2 - 30 * s, h / 2 + 60 * s), ("GPS 0", w - 90 * s, h - 10 * s),
            ("+", w / 2 - 8 * s, h / 2 + 8 * s)]


def osd_draw(img, t, alt, scale=0.85):
    """White text with a black edge, burned into the picture like an analog OSD."""
    for txt, x, y in osd_layout(img.shape[1], img.shape[0]):
        txt = {"TIME": f"{int(t) // 60:02d}:{int(t) % 60:02d}", "ALT": f"ALT {alt:.0f}"}.get(txt, txt)
        org = (int(x), int(y))
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale, 10, 3, cv2.LINE_AA)
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale, 245, 1, cv2.LINE_AA)
    return img


def osd_mask(w, h, scale=0.85):
    """uint8 mask, 0 = OSD text area (the widest text each item can show, grown by 4 px)."""
    m = np.zeros((h, w), np.uint8)
    for txt, x, y in osd_layout(w, h):
        txt = {"TIME": "00:00", "ALT": "ALT 000"}.get(txt, txt)
        cv2.putText(m, txt, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, 255, 3, cv2.LINE_AA)
    m = cv2.dilate(m, np.ones((9, 9), np.uint8))
    return np.where(m > 0, 0, 255).astype(np.uint8)


# ------------------------------------------------------------------ camera
VIDEO = {"PAL": (360, 288, 25.0), "NTSC": (360, 240, 30.0)}   # one field, width halved (as Hold is fed)
# Digital: sensor picture width, height, frames per second, and the whole factor the capture shrinks it by
# (settings video_reduce, cv2.INTER_AREA). Orqa Justice D (organizer; APB manual CSI 1): 1280x720 progressive at
# 60 fps; the onboard program gives Hold 640x360 (tools/profiles/justice_d.json video_reduce 2).
DIGITAL = {"JUSTICE_D": (1280, 720, 60.0, 2)}
SKY_GREY = 200.0     # digital only: a ray above the horizon sees flat sky of this grey. assumed (bright overcast)


def picture_size(video):
    """(width, height, fps) of the picture as Hold gets it."""
    if video in DIGITAL:
        w, h, fps, k = DIGITAL[video]
        return w // k, h // k, fps
    return VIDEO[video]


class Camera:
    """Rolling-shutter downward camera. Sensor row r is exposed at t0 + r/rows * readout (readout < 0:
    the sensor reads bottom to top), where t0 is the mid-exposure time of row 0. W, H, fps: the picture
    as Hold gets it. SW, SH: the rendered sensor picture (the same as W, H for an analog field)."""

    def __init__(self, c, ground):
        self.digital = c["video"] in DIGITAL
        if self.digital:
            self.SW, self.SH, self.fps, self.reduce = DIGITAL[c["video"]]
            self.W, self.H = self.SW // self.reduce, self.SH // self.reduce
        else:
            self.W, self.H, self.fps = VIDEO[c["video"]]
            self.SW, self.SH, self.reduce = self.W, self.H, 1
        self.K, self.D = lens(c["true_lens"], self.SW, self.SH, c["focal_scale"], c["pp_shift"])   # sensor pixels
        self.R_bc = mount(c["cam_top"], c["cam_mirror"], c["cam_tilt"], c["cam_twist"])
        uu, vv = np.meshgrid(np.arange(self.SW, dtype=float), np.arange(self.SH, dtype=float))
        self.rays_b = (unproject(uu, vv, self.K, self.D) @ self.R_bc.T).astype(np.float32)
        cx, cy = (self.SW - 1) / 2, (self.SH - 1) / 2
        self.centre_b = unproject(np.array([cx]), np.array([cy]), self.K, self.D)[0] @ self.R_bc.T
        self.ground, self.c = ground, c
        self.rng = np.random.default_rng(c["seed"] + 7)
        self.readout, self.exposure = c["readout_s"], c["exposure_s"]
        if self.digital and c.get("breakup"):
            raise ValueError("A digital camera has no analog break-up. Set breakup to 0.")

    def vib(self, t):
        """Motor vibration that the camera feels and the flight controller does not (body-frame turn)."""
        a = math.radians(self.c["vib_deg"])
        if a == 0:
            return None
        rx, ry = a * np.sin(2 * np.pi * 140.0 * t), a * np.sin(2 * np.pi * 97.0 * t + 1.0)  # assumed: as sim_ground.py
        return np.stack([np.ones_like(t), rx / 2, ry / 2, np.zeros_like(t)], -1) / np.sqrt(1 + (rx * rx + ry * ry) / 4)[:, None]

    def render(self, t0, pose):
        """pose(t) -> (N, E, height, q) arrays. Returns a float picture before noise (sensor size SW x SH)."""
        n_sub = 1 + int(self.exposure / 0.005)          # shortcut: motion blur from <= 3 sub-frames per exposure
        out = 0.0
        for dt in (np.linspace(-0.5, 0.5, n_sub) * self.exposure if n_sub > 1 else [0.0]):
            tr = t0 + dt + np.arange(self.SH) / self.SH * self.readout
            N, E, h, q = pose(tr)
            qv = self.vib(tr)
            R = q_matrix(q if qv is None else qmul(q, qv)).astype(np.float32)
            Nf, Ef = N[:, None].astype(np.float32), E[:, None].astype(np.float32)
            if self.digital and not (self.c.get("reflect") or self.c.get("people")):
                out = out + self._ground_rows(R, Nf, Ef, h)
                continue
            b = np.matmul(self.rays_b, R.transpose(0, 2, 1))          # row r: R_r @ ray, for every pixel
            bz = np.maximum(b[..., 2], 0.05)
            s = h[:, None].astype(np.float32) / bz
            g = self.ground.sample(Nf + s * b[..., 0], Ef + s * b[..., 1], float(np.mean(h)) / self.K[0, 0])
            g = self._indoor(g, tr, Nf, Ef, h[:, None].astype(np.float32), b, bz)
            if self.digital:          # a wide digital lens can see past 90 deg off nadir: sky, not ground
                g[b[..., 2] <= 0.0] = SKY_GREY
            out = out + g
        out = out / n_sub
        if self.c.get("flicker_hz"):
            # Mains lamps: brightness 1 + m cos(2 pi f t) (f = 100 or 120 Hz), averaged over the exposure (sinc), per
            # row time: horizontal bands that move when f is not a multiple of the field rate.
            f, m = self.c["flicker_hz"], self.c.get("flicker_depth", 0.5)
            tr = t0 + np.arange(self.SH) / self.SH * self.readout
            out = out * (1 + m * np.sinc(f * self.exposure) * np.cos(2 * np.pi * f * tr + 0.7))[:, None]
        if self.c.get("props"):
            out = self._props(out, t0)
        return out

    BLOCKS = 4        # digital: rows are rendered and noised in this many blocks on threads (numpy and cv2 release the GIL)

    def _blocks(self, fn):
        if not hasattr(self, "_pool"):
            from concurrent.futures import ThreadPoolExecutor
            self._pool = ThreadPoolExecutor(self.BLOCKS)
            edges = np.linspace(0, self.SH, self.BLOCKS + 1).astype(int)
            self._rows = [slice(a, b) for a, b in zip(edges[:-1], edges[1:])]
        list(self._pool.map(fn, range(self.BLOCKS)))

    def _ground_rows(self, R, N, E, h):
        """The same ground rays as render() (clamp, one pyramid level for the whole frame), per row block on threads,
        plus sky above the horizon."""
        out = np.empty((self.SH, self.SW), np.float32)
        gsd = float(np.mean(h)) / self.K[0, 0]
        hf = h[:, None].astype(np.float32)

        def blk(i):
            r = self._rows[i]
            b = np.matmul(self.rays_b[r], R[r].transpose(0, 2, 1))
            s = hf[r] / np.maximum(b[..., 2], 0.05)
            g = self.ground.sample(N[r] + s * b[..., 0], E[r] + s * b[..., 1], gsd)
            g[b[..., 2] <= 0.0] = SKY_GREY
            out[r] = g
        self._blocks(blk)
        return out

    def _noise_8bit(self, img):
        """img + white noise (c["noise"] grey per pixel), clipped to 8 bits; one generator per row block, so the
        result does not depend on the thread order."""
        if not hasattr(self, "_gens"):
            self._gens = [np.random.default_rng(s) for s in np.random.SeedSequence(self.c["seed"] + 7).spawn(self.BLOCKS)]
        out = np.empty(img.shape, np.uint8)

        def blk(i):
            r = self._rows[i]
            x = img[r] + self.c["noise"] * self._gens[i].standard_normal(img[r].shape, dtype=np.float32)
            out[r] = np.clip(x, 0, 255, out=x)
        self._blocks(blk)
        return out

    def _indoor(self, g, tr, N, E, h, b, bz):
        """Shiny-floor lamp reflections and people walking (indoor scenes); returns g unchanged when both are off."""
        c = self.c
        if c.get("reflect"):
            # A polished floor mirrors the ceiling lamps: their images sit ceiling_m below the floor, so they move
            # with the camera, not with the floor. assumed: lamps on a 4 m grid, ceiling 3 m above the drone (at
            # least 8 m above the floor), reflection 60 grey at the core with a 0.5 m soft edge.
            hc = c.get("ceiling_m") or max(8.0, float(np.mean(h)) + 3.0)
            s2 = (h + hc) / bz
            pn, pe = (N + s2 * b[..., 0]) % 4.0 - 2.0, (E + s2 * b[..., 1]) % 4.0 - 2.0
            g = g + 60.0 * np.exp(-(pn * pn + pe * pe) / (2 * 0.5 ** 2))
        k = int(c.get("people", 0))
        if k:
            # People walking back and forth across the room at 1.3 m/s on straight lines, seen as their shoulders: a
            # 0.25 m disc min(1.5 m, half the height) above the floor (parallax, independent motion). assumed sizes.
            ph = np.minimum(1.5, 0.5 * h)
            s3 = (h - ph) / bz
            pn, pe = N + s3 * b[..., 0], E + s3 * b[..., 1]
            rng = np.random.default_rng(int(c.get("seed", 1)) + 31)
            R = 1.4 * float(np.mean(h)) + 1.0                    # assumed: about the camera's view radius on the floor
            for _ in range(k):
                ang, lat, ph0 = rng.uniform(0, np.pi), rng.uniform(-0.7, 0.7) * R, rng.uniform(0, 1)
                grey = float(rng.uniform(40, 200))
                L = 2 * R + 2.0
                x = (1.3 * tr[:, None] / L + ph0) % 2.0
                x = (np.where(x < 1, x, 2 - x) - 0.5) * L          # triangle wave along the line
                cn = x * math.cos(ang) - lat * math.sin(ang)
                ce = x * math.sin(ang) + lat * math.cos(ang)
                d2 = (pn - cn) ** 2 + (pe - ce) ** 2
                a = np.clip((0.25 - np.sqrt(d2)) / 0.03 + 0.5, 0, 1)   # 3 cm soft edge
                g = g * (1 - a) + grey * a
        return g

    def _props(self, out, t0):
        """Blurred spinning propellers in the 4 picture corners (propellers below the arms seen by a wide lens), and
        the arm toward each one. assumed: disc radius 0.32 x picture height around a point just outside each corner,
        2 blades at 120 rev/s, dark (grey 30), seen through at 60-80 %; rolling shutter leaves faint curved streaks."""
        H, W = out.shape
        if not hasattr(self, "_pg"):
            yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
            self._pg = []
            for cx, cy in ((-0.04 * W, -0.05 * H), (1.04 * W, -0.05 * H), (-0.04 * W, 1.05 * H), (1.04 * W, 1.05 * H)):
                r, th = np.hypot(xx - cx, yy - cy), np.arctan2(yy - cy, xx - cx)
                arm = (np.abs((xx - cx) * (H / 2 - cy) - (yy - cy) * (W / 2 - cx)) / math.hypot(W / 2 - cx, H / 2 - cy) < 5) \
                    & (r < 0.45 * H)
                self._pg.append((r < 0.32 * H, th, arm))
        tr = (t0 + np.arange(H) / H * self.readout)[:, None]
        out = out.copy()
        for disc, th, arm in self._pg:
            a = np.where(disc, 0.3 + 0.1 * np.cos(2 * (th - 2 * np.pi * 120.0 * tr)), 0.0)
            out = out * (1 - a) + 30.0 * a
            out[arm] = 25.0
        return out

    def finish(self, img):
        """Lens blur, then analog noise correlated along x (measured std 3.7 grey on the real video).
        Digital: lens blur, white sensor noise per sensor pixel (c["noise"] grey), 8 bits, then the capture's
        INTER_AREA shrink to W x H (capture.DigitalSource)."""
        if self.digital:
            img = cv2.GaussianBlur(np.asarray(img, np.float32), (3, 3), 0.8)   # assumed: lens + demosaic blur, 0.8 sensor px
            if not hasattr(self, "_rows"):
                self._blocks(lambda i: None)
            img = self._noise_8bit(img)
            return cv2.resize(img, (self.W, self.H), interpolation=cv2.INTER_AREA)
        img = cv2.GaussianBlur(img, (3, 3), 0.8)
        n = cv2.GaussianBlur(self.rng.standard_normal(img.shape).astype(np.float32), (0, 0),
                             sigmaX=self.c["noise_corr_px"], sigmaY=0.01)
        img = img + self.c["noise"] * n / max(float(n.std()), 1e-6)
        return np.clip(img, 0, 255).astype(np.uint8)

    def breakup(self, img):
        """Radio-style break-up: a band of rows torn sideways and drowned in noise."""
        r0 = int(self.rng.integers(0, self.H // 2))
        r1 = int(self.rng.integers(r0 + self.H // 4, self.H + 1))
        band = img[r0:r1].astype(np.float32)
        for i in range(len(band)):
            band[i] = np.roll(band[i], int(self.rng.integers(-40, 41)))
        band += self.rng.normal(0, 30, band.shape)
        img = img.copy()
        img[r0:r1] = np.clip(band, 0, 255).astype(np.uint8)
        return img

    def ground_point(self, ray_b, N, E, h, q):
        """Where a body-frame ray hits the ground (NED north, east)."""
        b = qrot(q, ray_b)
        s = h / np.maximum(b[..., 2], 1e-3)
        return N + s * b[..., 0], E + s * b[..., 1]

    def pixel_of(self, P, N, E, h, q):
        """Pixel (of the W x H picture) where ground point P = (north, east) appears for the pose (N, E, h, q);
        None if behind."""
        d_b = qrot(q * np.array([1, -1, -1, -1.0]), np.array([P[0] - N, P[1] - E, h]))
        d_c = self.R_bc.T @ d_b
        if d_c[2] <= 0:
            return None
        u, v = project(d_c, self.K, self.D)
        if self.reduce > 1:           # sensor pixel -> pixel of the shrunk picture (INTER_AREA: block centres)
            u, v = (u + 0.5) / self.reduce - 0.5, (v + 0.5) / self.reduce - 0.5
        return float(u), float(v)


class Frames:
    """Camera frames in simulator time. Frame k starts its exposure at k / fps and reaches the
    estimator video_delay after its last row is read. The newest ready frame is handed out; older
    ones are dropped, as a capture queue of depth one does."""

    def __init__(self, cam, link, c, render=True):
        self.cam, self.link, self.c, self.render = cam, link, c, render
        self.lag = abs(cam.readout) + cam.exposure + c["video_delay_s"]
        self.prev, self.burst, self.n_out, self.judge = None, 0, 0, None
        self.mask = osd_mask(cam.W, cam.H) if c["osd"] else None

    def t_start(self, k):
        return k / self.cam.fps

    def __iter__(self):
        if self.cam.digital:
            yield from self._digital()
            return
        c, cam, link, k = self.c, self.cam, self.link, None
        burst_p = c["breakup"] / (4.0 * max(1e-9, 1 - c["breakup"]))   # assumed: bursts of 4 frames on average
        while not link.done:
            kk = int(math.floor((link.now() - self.lag) * cam.fps))
            if k is not None and kk <= k:
                link.wait_until(self.t_start(k + 1) + self.lag)
                continue
            k = kk
            t0 = self.t_start(k) + cam.exposure / 2 + (abs(cam.readout) if cam.readout < 0 else 0.0)
            img = None
            if self.render:
                frozen = c["freeze"] and c["freeze"][0] <= t0 - link.t_engage < c["freeze"][0] + c["freeze"][1]
                if frozen and c.get("freeze_noise") and getattr(self, "clean", None) is not None:
                    img = cam.finish(self.clean)          # frozen picture, fresh analog noise on it (known gap)
                else:
                    self.clean = cam.render(t0, link.truth_at)
                    img = cam.finish(self.clean)
                lost = c.get("outage") and c["outage"][0] <= t0 - link.t_engage < c["outage"][0] + c["outage"][1]
                lost = lost or t0 < getattr(link, "t_video", -math.inf)     # still_start: static until the hover
                if lost:      # video link lost: the receiver shows analog static (the OSD is made on the drone)
                    n = cam.rng.normal(0, 1, (cam.H, cam.W)).astype(np.float32)
                    img = np.clip(128 + 45 * cv2.GaussianBlur(n, (0, 0), sigmaX=1.0, sigmaY=0.01) / 0.53, 0, 255).astype(np.uint8)
                elif self.prev is not None and ((frozen and not c.get("freeze_noise")) or cam.rng.random() < c["repeat"]):
                    img = self.prev
                else:
                    if self.burst == 0 and cam.rng.random() < burst_p and t0 - link.t_engage >= c.get("breakup_from", -math.inf):
                        self.burst = int(cam.rng.geometric(0.25))
                    if self.burst > 0:
                        self.burst -= 1
                        img = cam.breakup(img)
                    if c["osd"]:
                        img = osd_draw(img, t0, link.h_now())
                self.prev = img
                if self.judge is not None:
                    self.judge.write(img, t0, k)
            self.n_out += 1
            self.t_avail = self.t_start(k) + self.lag
            yield img, t0 + c["stamp_err_s"]

    READY_S = 0.006   # digital: render once the truth covers the frame's last row by this much (truth samples 4-5 ms apart)

    def _digital(self):
        """Digital camera frames. A 1280x720 render takes about as long as one 60 fps frame on this laptop, so it
        starts as soon as the truth covers the frame's last row, and the frame is handed over at its stamp time
        (end of readout + video_delay_s), not before: the render time does not add to the frame age as long as it
        is under video_delay_s - READY_S. No break-up; a lost link gives no frames (CSI: no receiver static)."""
        c, cam, link, k = self.c, self.cam, self.link, None
        ready = abs(cam.readout) + cam.exposure + self.READY_S
        self.late = []                                    # sim s each frame was handed over after its stamp
        while not link.done:
            kk = int(math.floor((link.now() - ready) * cam.fps))
            if k is not None and kk <= k:
                link.wait_until(self.t_start(k + 1) + ready)
                continue
            # The next frame in turn; a frame is dropped only when a newer one is already past its stamp (a capture
            # queue of depth one). Taking the newest ready frame instead skipped every other frame when video_delay_s
            # is longer than a frame (run jdcal_apb_100m_w20g40_s11: 30 of 60 fps).
            k = kk if k is None else max(k + 1, int(math.floor((link.now() - self.lag) * cam.fps)))
            t0 = self.t_start(k) + cam.exposure / 2 + (abs(cam.readout) if cam.readout < 0 else 0.0)
            img = None
            if self.render:
                rel = t0 - link.t_engage
                lost = (c.get("outage") and c["outage"][0] <= rel < c["outage"][0] + c["outage"][1]) or \
                    t0 < getattr(link, "t_video", -math.inf)
                frozen = c["freeze"] and c["freeze"][0] <= rel < c["freeze"][0] + c["freeze"][1]
                if lost:
                    continue
                if self.prev is not None and (frozen or (c["repeat"] > 0 and cam.rng.random() < c["repeat"])):
                    img = self.prev                       # a frozen stream repeats the buffer exactly
                else:
                    img = cam.finish(cam.render(t0, link.truth_at))
                    if c["osd"]:
                        img = osd_draw(img, t0, link.h_now())
                self.prev = img
            link.wait_until(self.t_start(k) + self.lag)
            self.late.append(link.now() - (self.t_start(k) + self.lag))
            if self.judge is not None and img is not None:
                self.judge.write(img, t0, k)
            self.n_out += 1
            self.t_avail = self.t_start(k) + self.lag
            yield img, t0 + c["stamp_err_s"]


class JudgeVideo:
    """Judge view: the camera picture with a crosshair at its centre, a mark on the ground at the hold
    point, and a line with the height and the true drift."""

    def __init__(self, path, cam, link, hold_pt, label):
        import subprocess
        import imageio_ffmpeg
        self.cam, self.link, self.hold, self.label, self.last_k = cam, link, np.asarray(hold_pt), label, None
        self.w, self.h = 2 * cam.W, 2 * cam.H
        self.p = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "rawvideo",
                                   "-pix_fmt", "bgr24", "-s", f"{self.w}x{self.h}", "-r", f"{cam.fps:g}", "-i", "-",
                                   "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24", path],
                                  stdin=subprocess.PIPE)

    def write(self, img, t0, k):
        cam = self.cam
        N, E, h, q = (a[0] for a in self.link.truth_at(np.array([t0 + cam.readout / 2])))
        v = cv2.resize(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), (self.w, self.h), interpolation=cv2.INTER_NEAREST)
        cx, cy = self.w // 2, self.h // 2
        cv2.line(v, (cx - 14, cy), (cx + 14, cy), (0, 255, 0), 1)
        cv2.line(v, (cx, cy - 14), (cx, cy + 14), (0, 255, 0), 1)
        px = cam.pixel_of(self.hold, N, E, h, q)
        if px is not None:
            c = (int(2 * px[0] + 1), int(2 * px[1] + 1))
            cv2.circle(v, c, 9, (40, 40, 255), 2)
            cv2.line(v, (c[0] - 5, c[1] - 5), (c[0] + 5, c[1] + 5), (40, 40, 255), 2)
            cv2.line(v, (c[0] - 5, c[1] + 5), (c[0] + 5, c[1] - 5), (40, 40, 255), 2)
        drift = math.hypot(N - self.hold[0], E - self.hold[1])
        txt = f"{self.label}  t {t0 - self.link.t_engage:5.1f} s  h {h:5.1f} m  drift {drift:5.2f} m"
        cv2.rectangle(v, (0, 0), (self.w, 24), (0, 0, 0), -1)
        cv2.putText(v, txt, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        reps = 1 if self.last_k is None else max(1, min(k - self.last_k, 10))   # keep video time = sim time
        self.last_k = k
        for _ in range(reps):
            self.p.stdin.write(v.tobytes())

    def close(self):
        self.p.stdin.close()
        self.p.wait()
