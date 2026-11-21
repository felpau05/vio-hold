"""Synthetic forward-camera pictures for the fwd_cam tests. SYNTHETIC ONLY.

Own scene model, independent of fwd_cam.py: the lens projection, the quaternions and the attitude come from
review/geometry/indep.py (written from first principles for the geometry review); fwd_cam.py uses cv2.fisheye and
its own Euler code. The ground is an aerial photo (ground/*.jpg, 2400 m, repeated by reflection beyond that).

Scene: flat ground with a level-of-detail texture (far ground is blurred as a camera would average it), the
horizon dipped by sqrt(2 h / R) (R = 7/6 of the Earth radius for refraction), a sky that is brighter near the
horizon, and these options, each one step harder:
  noise     analog noise, sigma in grey, slightly correlated along the row (analog bandwidth)
  haze      visibility in m: L = L_obj exp(-d/V) + L_horizon_sky (1 - exp(-d/V))
  skyline   trees and buildings on a ring of radius 250 m around the hover point, up to 10 deg high seen from
            the ring centre, random shape; they hide the true horizon. Parallax with the drift is real.
  text      white OSD text with a black edge in the four corners (fixed in the picture)
  field     one video field: every second row (half the vertical resolution)
Global shutter (no rolling shutter). Truth attitude: the one the picture is drawn with.
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(PROJ, "review", "geometry"))
import indep as I          # noqa: E402

R_EARTH_REFR = 6371e3 * 7.0 / 6.0
SKY_HORIZON, SKY_ZENITH = 205.0, 150.0


def fwd_mount_true(alpha_deg, rho_deg=0.0, yaw_deg=0.0):
    """Camera -> body from a physical description: optical axis forward, turned up by alpha about the body right
    axis, turned left/right by yaw_deg, and the picture turned by rho_deg about the optical axis. Columns: camera
    x (picture right), y (picture down), z (optical axis) in body FRD. Built from quaternions, not from fwd_cam."""
    qy = I.quat_axis_angle(I.E_D, math.radians(yaw_deg))
    right = I.qrot(qy, I.E_E)
    q = I.qmul(I.quat_axis_angle(right, math.radians(alpha_deg)), qy)          # nose up turn about the right axis
    z = I.qrot(q, I.E_N)                                                         # optical axis
    q = I.qmul(I.quat_axis_angle(z, math.radians(rho_deg)), q)
    x, y = I.qrot(q, I.E_E), I.qrot(q, I.E_D)                                    # picture right, picture down
    return np.column_stack([x, y, z])


def ground_texture(site):
    t = cv2.imread(os.path.join(PROJ, "ground", f"{site}.jpg"), cv2.IMREAD_GRAYSCALE)
    if t is None:
        raise FileNotFoundError(f"ground/{site}.jpg (run fetch_ground.py)")
    return t.astype(np.float32)


class FwdRenderer:
    def __init__(self, K, D, R_bc, W, H, tex, span_m=2400.0, noise=0.0, haze_m=None, skyline=False, text=False,
                 field=False, seed=0):
        self.W, self.H, self.R_bc = W, H, R_bc
        uu, vv = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
        rc = I.unproject_cam(uu, vv, K, D)
        self.rays_b = rc @ R_bc.T
        self.pix_rad = 1.0 / K[0, 0]                                              # angle of one pixel (centre)
        self.pyr = [tex * 0.75 + 15.0]
        while self.pyr[-1].shape[0] > 32:
            self.pyr.append(cv2.pyrDown(self.pyr[-1]))
        self.m = tex.shape[0] / span_m
        self.noise, self.haze, self.text, self.field = noise, haze_m, text, field
        self.rng = np.random.default_rng(seed + 1000)
        self.sky_az = np.random.default_rng(seed + 2000).uniform(0, 2 * np.pi)
        self.skyline = self._make_skyline(np.random.default_rng(seed + 3000)) if skyline else None
        self.centre = np.zeros(2)                       # the skyline ring's centre (set it to the hover point)

    # ------------------------------------------------------------ skyline: ring of trees and buildings
    def _make_skyline(self, rng, n=3600):
        az = np.linspace(0, 2 * np.pi, n, endpoint=False)
        top = np.zeros(n)
        for _ in range(40):                                                       # tree clumps: smooth bumps
            c, wdt, hgt = rng.uniform(0, 2 * np.pi), rng.uniform(0.02, 0.25), rng.uniform(1.0, 8.0)
            d = np.angle(np.exp(1j * (az - c)))
            top = np.maximum(top, hgt * np.exp(-0.5 * (d / wdt) ** 2) * (1 + 0.15 * rng.standard_normal(n)))
        for _ in range(12):                                                       # buildings: flat tops
            c, wdt, hgt = rng.uniform(0, 2 * np.pi), rng.uniform(0.01, 0.06), rng.uniform(2.0, 10.0)
            d = np.abs(np.angle(np.exp(1j * (az - c))))
            top = np.where(d < wdt, np.maximum(top, hgt), top)
        top = np.clip(cv2.GaussianBlur(top.reshape(1, -1).astype(np.float32), (0, 0), 1.5).ravel(), 0.0, 10.0)
        tex = cv2.GaussianBlur(rng.normal(0, 1, (400, n)).astype(np.float32), (0, 0), 2.0)
        tex = 55.0 + 18.0 * tex / (tex.std() + 1e-6)
        return dict(R=250.0, az=az, top=np.radians(top), tex=tex)

    # ------------------------------------------------------------ one picture
    def frame(self, pos_ne, h, att):
        """pos_ne: (north, east) m; h: height m; att: (roll, pitch, yaw) rad. Returns uint8 (H or H/2, W)."""
        q = I.att_quat(*att)
        Rwb = I.quat_to_matrix(q)
        b = self.rays_b @ Rwb.T                                                   # NED rays (H, W, 3)
        bxy = np.hypot(b[..., 0], b[..., 1]) + 1e-12
        el = np.arctan2(-b[..., 2], bxy)                                          # above the local horizontal: +
        dip = math.sqrt(2 * h / R_EARTH_REFR)
        ground = el < -dip
        tan_d = np.tan(np.maximum(-el, dip))
        d = np.where(ground, h / tan_d, np.inf)                                   # horizontal distance to ground
        ux, uy = b[..., 0] / bxy, b[..., 1] / bxy
        # level of detail: footprint of a pixel on the ground (geometric mean of across and along range)
        rng_m = np.where(ground, d, 1e6)
        foot = rng_m * self.pix_rad / np.sqrt(np.maximum(np.sin(np.maximum(-el, dip)), 1e-4))
        lod = np.clip(np.log2(np.maximum(foot * self.m, 1e-6)), 0, len(self.pyr) - 1)
        Pn = pos_ne[0] + np.where(ground, d, 0) * ux
        Pe = pos_ne[1] + np.where(ground, d, 0) * uy
        img = np.zeros(el.shape, np.float32)
        wsum = np.zeros(el.shape, np.float32)
        for k, lev in enumerate(self.pyr):
            wk = np.clip(1.0 - np.abs(lod - k), 0, 1).astype(np.float32) * ground
            if not wk.any():
                continue
            mk = self.m / 2 ** k
            n = lev.shape[0]
            mx = (Pe * mk + n / 2).astype(np.float32)
            my = (-Pn * mk + n / 2).astype(np.float32)
            img += wk * cv2.remap(lev, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT101)
            wsum += wk
        img = np.where(ground, img / np.maximum(wsum, 1e-6), 0)
        az = np.arctan2(uy, ux)
        sky = SKY_ZENITH + (SKY_HORIZON - SKY_ZENITH) * np.exp(-np.maximum(el, 0) / math.radians(12)) \
            + 8.0 * np.cos(az - self.sky_az) * np.exp(-np.maximum(el, 0) / math.radians(30))
        dist = np.where(ground, np.sqrt(d ** 2 + h ** 2), np.inf)
        if self.skyline is not None:
            S = self.skyline
            pr = np.asarray(pos_ne, float) - self.centre
            pu = pr[0] * ux + pr[1] * uy
            s = -pu + np.sqrt(np.maximum(pu ** 2 - (pr[0] ** 2 + pr[1] ** 2) + S["R"] ** 2, 0))
            hn, he = pr[0] + s * ux, pr[1] + s * uy
            azh = np.mod(np.arctan2(he, hn), 2 * np.pi)
            ia = (azh / (2 * np.pi) * len(S["az"])).astype(int) % len(S["az"])
            ztop = S["R"] * np.tan(S["top"][ia])                                  # top above the camera, m
            zray = s * np.tan(el)
            obj = (zray < ztop) & ~(ground & (d < s))
            zrel = np.clip((zray + h) / (ztop + h + 1e-6), 0, 1)                  # 0 at the foot, 1 at the top
            ot = cv2.remap(S["tex"], (azh / (2 * np.pi) * S["tex"].shape[1]).astype(np.float32),
                           ((1 - zrel) * (S["tex"].shape[0] - 1)).astype(np.float32), cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_WRAP)
            img = np.where(obj, ot, img)
            dist = np.where(obj, s, dist)
            ground = ground | obj
        if self.haze:
            tr = np.exp(-dist / self.haze)
            img = np.where(ground, img * tr + SKY_HORIZON * (1 - tr), sky)
        else:
            img = np.where(ground, img, sky)
        img = cv2.GaussianBlur(img.astype(np.float32), (0, 0), 0.8)
        if self.text:
            img = self._text(img)
        if self.noise:
            nz = cv2.blur(self.rng.normal(0, 1, img.shape).astype(np.float32), (3, 1))
            img = img + nz * (self.noise / nz.std())
        out = np.clip(img, 0, 255).astype(np.uint8)
        return out[0::2] if self.field else out

    def _text(self, img):
        if not hasattr(self, "_txt"):
            W, H = self.W, self.H
            sc = H / 480.0
            val, alpha = np.zeros((H, W), np.uint8), np.zeros((H, W), np.uint8)
            lines = [("12.4V  RSSI 87", (int(14 * sc), int(32 * sc))),
                     ("00:42  ACRO", (W - int(190 * sc), int(32 * sc))),
                     ("ALT 30M", (int(14 * sc), H - int(18 * sc))),
                     ("HOME 0M  3D", (W - int(200 * sc), H - int(18 * sc)))]
            for txt, org in lines:
                for th, v in ((int(5 * sc) + 1, 0), (int(2 * sc) + 1, 245)):
                    cv2.putText(val, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8 * sc, v, th, cv2.LINE_AA)
                    cv2.putText(alpha, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8 * sc, 255, th, cv2.LINE_AA)
            self._txt = (val.astype(np.float32), alpha.astype(np.float32) / 255.0)
        v, a = self._txt
        return img * (1 - a) + v * a


# ------------------------------------------------------------------- the hover with a known attitude
def hover(h0, seed, T=21.0, yaw_amp_deg=20.0):
    """Small drift and a tilt wobble like a windy hover (common_hover.hover_path, wobble 1 deg: roll and pitch up to
    about 6 deg), a slow yaw swing of +-yaw_amp_deg, a random start heading and a random place on the photo."""
    sys.path.insert(0, os.path.join(PROJ, "review", "geometry"))
    from common_hover import hover_path
    pos, att, hf = hover_path(h0, wobble_deg=1.0, yaw_amp_deg=yaw_amp_deg)
    r = np.random.default_rng(seed)
    yaw0, off, ph = r.uniform(-np.pi, np.pi), r.uniform(-600, 600, 2), r.uniform(0, 10)
    pos_fn = lambda t: pos(np.atleast_1d(t) + ph) - pos(np.array([ph])) + off             # noqa: E731
    att_fn = lambda t: att(np.atleast_1d(t) + ph) + np.array([0.0, 0.0, yaw0])             # noqa: E731
    return pos_fn, att_fn, hf, off
