"""Keyframe-anchored nadir position hold for a rolling-shutter camera (version 2).

A ground point seen in the keyframe has a fixed NED position G. In a later frame the camera
sits where the rays to those points, scaled to the ground by the height, land on G. Each ray is
rotated with the flight-controller attitude at the time its image row was exposed, which
removes rolling-shutter skew. A robust 2-D similarity fit of G to the rays gives the position,
a yaw correction and a scale. While the keyframe stays in view the fix does not drift.

Safety first: a frame gives nothing (var None) when the picture repeats, is broken up, shows no
ground (patch match quality), or moves differently from what the attitude predicts (frozen
picture, static overlay, wrong mount). A new anchor sends nothing until the picture has proved
that it is live.

No I/O here: apb_main.py feeds it live data, sim_test.py and sim_ground.py feed it synthetic frames.
"""
import json
import math
from collections import namedtuple

import cv2
import numpy as np

# OpenCL off: the first ORB call (the re-find after a loss) started OpenCL and stalled 0.16 s alone and
# 2.7 s in the PX4 SITL loop (integ/orb_first_call.txt, integ/diag_refind2.txt); with OpenCL off it takes
# 3 ms. Hold runs on the CPU; an OpenCL start on the i.MX8M Plus GPU mid-flight is not tested.
cv2.ocl.setUseOpenCL(False)

# Nadir camera, image top toward the nose: image x = body right, image y = body back,
# optical axis = body down. Columns are the camera axes written in body FRD.
R_BC_NADIR = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])

# LK settings of version 1, kept for clip_tool.py and sim_view.py. Hold takes its window and pyramid
# levels from its own LK_WIN / LK_LEVELS (cfg). minEigThreshold drops points with too little texture.
LK = dict(winSize=(21, 21), maxLevel=3, minEigThreshold=3e-3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

Est = namedtuple("Est", "t pos var n state recovered why cov yaw scale", defaults=(None, None, None))
# state: 'init' (new anchor, nothing to send yet), 'ok', 'degraded' (send, larger covariance; with
#        var None it is a failed frame), 'lost' (send nothing; the caller may re-anchor).
# Send a fix only when var is not None. var = trace of cov.
# cov: 2x2 covariance of pos (m^2). yaw: add it to the FC yaw to agree with the keyframe's heading
# (rad). scale: height now / height at the keyframe, as the picture sees it. None when not estimated.

_PATCH = np.arange(-5, 6, dtype=np.float32)      # 11 x 11 patch for the match-quality (NCC) test


def load_lens(path, name, w, h, field=None):
    """K, D for a w x h image that shows the same picture extent as the profile's image.

    field: None for a resized frame. 0 or 1 when the image is one analog field (frame lines 2r or
    2r + 1) of h rows: then the row centre is cy = ((cy_c + 0.5) / s - 0.5 - field) / 2 with
    s = profile rows / (2 h) (G8). D has 4 values for an equidistant fisheye profile and 5 values
    (k1, k2, p1, p2, k3, OpenCV order) for a 'frame' (Brown) profile; Hold picks the model by len(D).
    """
    p = next(m for m in json.load(open(path))["models"] if m["name"] == name)
    pw, ph = p["image_size"]
    sx, sy = w / pw, h / ph
    cy = (p["cy"] + 0.5) * sy - 0.5 if field is None else ((p["cy"] + 0.5) * 2 * sy - 0.5 - field) / 2
    K = np.array([[p["fx"] * sx, 0, (p["cx"] + 0.5) * sx - 0.5], [0, p["fy"] * sy, cy], [0, 0, 1.]])
    if p["projection"] == "equidistant_fisheye":
        if p.get("p1") or p.get("p2") or p.get("b2"):
            raise ValueError(f"lens '{name}': cv2.fisheye has no p1, p2 or b2 term; pick another profile")
        return K, np.array([p["k1"], p["k2"], p["k3"], p["k4"]], float)
    if p["projection"] == "frame":
        if p.get("b2") or p.get("k4"):
            raise ValueError(f"lens '{name}': the OpenCV model has no b2 or k4 term")
        # Metashape's P1 multiplies (r^2 + 2x^2) in x, which is OpenCV's p2; P2 is OpenCV's p1.
        return K, np.array([p["k1"], p["k2"], p["p2"], p["p1"], p["k3"]], float)
    raise ValueError(f"lens '{name}': unknown projection {p['projection']!r}")


def same_aspect(profile_size, w, h, tol=0.01):
    """True when a w x h picture has the width / height ratio of a profile's image_size (within tol).

    A digital picture reduced by an integer factor keeps the ratio. An analog field (360x288 from a
    960x720 profile) does not, and load_lens still serves it, so load_lens does not call this.
    """
    return abs((w / h) / (profile_size[0] / profile_size[1]) - 1) <= tol


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
        self.rows, self.keep, self._arr, self._first = [], keep_s, None, None

    def add(self, t, *vals):
        self.rows.append((t, *vals))
        if len(self.rows) > 4000 and t - self.rows[0][0] > self.keep:
            del self.rows[:2000]

    def last_t(self):
        return self.rows[-1][0] if self.rows else -math.inf

    def array(self):
        """All rows as an array; only rows added since the last call are converted (E9)."""
        rows = self.rows
        n, a = len(rows), self._arr
        if a is None or not n or rows[0] is not self._first or len(a) > n:
            self._arr, self._first = np.array(rows[:n], float), rows[0] if n else None
        elif len(a) < n:
            self._arr = np.concatenate([a, np.array(rows[len(a):n], float)])
        return self._arr

    def at(self, t):
        a = self.array()
        return np.interp(t, a[:, 0], a[:, 1])


def unwrapped_yaw(att, a=None):
    """Yaw column of att without 2 pi steps (computed once per new attitude sample).
    a: the att.array() the caller already holds. Another thread can add a row between two array()
    calls, so the yaw must come from the same array as the times (np.interp: different lengths)."""
    a = att.array() if a is None else a
    if getattr(att, "_yaw_of", None) is not a:
        att._yaw_of, att._yaw = a, np.unwrap(a[:, 3])
    return att._yaw


def attitude_R(att, t):
    """Body->NED at times t, from a Series of (t, roll, pitch, yaw[, rates])."""
    a = att.array()
    return euler_R(np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]),
                   np.interp(t, a[:, 0], unwrapped_yaw(att, a)))


def _med(x):
    """Lower median along axis 0 (np.median costs ~80 us a call; this is called ~10 times a frame)."""
    k = (len(x) - 1) // 2
    return np.partition(x, k, axis=0)[k]


def _rot2(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _similarity(A, B):
    """C, s, phi minimising sum |A_i - (C + s R(phi) B_i)|^2 (2-D Umeyama; t5b_similarity_fix.py)."""
    ma, mb = A.mean(0), B.mean(0)
    a, b = A - ma, B - mb
    phi = math.atan2(np.sum(b[:, 0] * a[:, 1] - b[:, 1] * a[:, 0]), np.sum(b[:, 0] * a[:, 0] + b[:, 1] * a[:, 1]))
    R = _rot2(phi)
    s = np.sum(a * (b @ R.T)) / max(np.sum(b * b), 1e-12)
    return ma - s * R @ mb, s, phi


def _cubic(px, shape):
    """Cubic monomials of pixel positions (N x 2) normalised to the picture (N x 10)."""
    u, v = px[:, 0] / shape[1] - 0.5, px[:, 1] / shape[0] - 0.5
    return np.stack([np.ones_like(u), u, v, u * u, u * v, v * v, u ** 3, u * u * v, u * v * v, v ** 3], 1)


def robust_fit(A, B, floor):
    """A_i ~ C + s R(phi) B_i with outliers removed (E3, G3).

    Starts at the median shift, gates each axis at 2.5 robust sigma (1.4826 MAD, at least floor)
    around the median residual, refits on the inliers, three rounds. Returns C, s, phi, inliers,
    residuals. Hold also drops a keyframe point for good after OUT_STRIKES outlier frames in a row,
    so a moving object loses its points before it can hold half of them.
    """
    C, s, phi = _med(A - B), 1.0, 0.0
    inl = np.ones(len(A), bool)
    for _ in range(3):
        r = A - C - s * B @ _rot2(phi).T
        med = _med(r[inl])
        sig = np.maximum(1.4826 * _med(np.abs(r[inl] - med)), floor)
        inl = (np.abs(r - med) < 2.5 * sig).all(axis=1)
        if inl.sum() < 3:
            break
        C, s, phi = _similarity(A[inl], B[inl])
    return C, s, phi, inl, A - C - s * B @ _rot2(phi).T


def corner_mask(shape, corner=0.0, border=0):
    """Mask for Hold (uint8 H x W, 0 = not ground, 255 = ground) that ignores the picture corners and a border:
    propellers or arms that a wide downward lens sees. corner: radius of a quarter disc at each corner, as a share of
    the picture height (0 = none). border: width of a band along all four edges (px, 0 = none). Combine it with
    another mask by np.minimum."""
    H, W = shape[:2]
    m = np.full((H, W), 255, np.uint8)
    if corner > 0:
        r = corner * H
        yy, xx = np.mgrid[0:H, 0:W]
        for cx, cy in ((0, 0), (W - 1, 0), (0, H - 1), (W - 1, H - 1)):
            m[(xx - cx) ** 2 + (yy - cy) ** 2 < r * r] = 0
    if border > 0:
        b = int(border)
        m[:b], m[-b:], m[:, :b], m[:, -b:] = 0, 0, 0, 0
    return m


def build_static_mask(frames):
    """Not-ground mask from a short recording of grey frames (port of clip_tool.build_mask).

    frames: sequence of uint8 H x W pictures taken while the drone moves over ground. Returns a
    uint8 H x W mask for Hold: 0 = not ground (OSD text, crosshair, airframe), 255 = ground.
    Not ground = edges that stay in place while the ground moves, or dark (< 40) for 6 % of the
    frames; grown by 3 px. clip_tool.build_mask also used near-white and red pixels; with grey only,
    bright ground looks like white text (30 % of the sim's photo ground), so that test is left out.
    """
    n = len(frames)
    gx_sum = gy_sum = g_abs = dark = 0.0
    for g in frames:
        gf = g.astype(np.float32)
        gx, gy = cv2.Sobel(gf, cv2.CV_32F, 1, 0), cv2.Sobel(gf, cv2.CV_32F, 0, 1)
        gx_sum, gy_sum, g_abs, dark = gx_sum + gx, gy_sum + gy, g_abs + np.hypot(gx, gy), dark + (g < 40)
    edge = np.hypot(gx_sum, gy_sum) / n                       # moving ground averages out
    lg = np.log10(edge + 1)
    lg8 = (255 * lg / max(lg.max(), 1e-6)).astype(np.uint8)
    t, _ = cv2.threshold(lg8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # shortcut: thresholds tuned on clip-1 and on a 5 s sim hover (overlay ratio >= 0.95, ground p99
    # 0.54 there); retune on event-day video.
    static = (lg8 > t) & (np.hypot(gx_sum, gy_sum) / np.maximum(g_abs, 1e-6) > 0.8)
    ell = lambda d: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (d, d))
    dk = cv2.morphologyEx((dark / n > 0.06).astype(np.uint8), cv2.MORPH_OPEN, ell(7))   # airframe in view
    m = (static | (dk > 0)).astype(np.uint8)
    m = cv2.dilate(cv2.morphologyEx(m, cv2.MORPH_CLOSE, ell(5)), ell(7))
    return np.where(m > 0, 0, 255).astype(np.uint8)


class Hold:
    """Tuning: every UPPER-CASE constant below can be changed by Hold(..., lower_case_name=value)."""
    MIN_PTS = 20              # fewer usable points: frame unusable
    LOST_S = 0.4              # s of failed frames before 'lost' (10 frames at 25 Hz)
    MAX_RATE = 2.0            # rad/s; faster rotation makes row timing errors dominate
    KF_RATE = 0.3             # rad/s; take an unforced keyframe only below this rate
    KF_GAP = 1.0              # s; at most one unforced keyframe change per KF_GAP (E6)
    KF_LOW = 0.8              # a forced change (points under MIN_PTS + 10) also needs n under this share of the first fix's n
    MAX_OFF_NADIR = math.radians(55)   # slanted rays: poor lens model and flat-ground error
    H_MIN = 5.0               # m; lower or non-positive height: 'lost', nothing sent (H9). assumed: ground effect, baro offset on the pad
    MAX_PTS = 150             # keyframe points (E9: 120-150 keep accuracy and cut compute)
    GRID = (6, 4)             # corners are picked per cell of this grid (E2, E3)
    LK_WIN = 15               # LK window (px); E9: 15 x 15 and 2 levels keep the accuracy of 21 x 21 and 3
    LK_LEVELS = 2             # LK pyramid levels above the full picture
    FB_EVERY = 1              # run the backward check on every n-th frame (1 = always)
    NCC_MIN = 0.6             # point match quality against the keyframe patch (E1: ground 0.96, static 0.06)
    NCC_FRAME = 0.5           # median match quality below this: no ground in the picture
    REPEAT_DIFF = 1.0         # mean |frame - previous frame| (grey) below this: repeated picture. assumed: DVR repeats 0.0-0.3, live noise ~4
    BREAK_Z = 6.0             # robust z of impulse noise or brightness above this: broken-up picture (clip_tool.py)
    BREAK_HIST = 50           # accepted frames in the cue history of the breakup test (2 s at 25 Hz)
    BREAK_K = 6               # frames that must hold a new level steady (spread < 1/4 of the change) to be accepted
    BREAK_AFTER = 1           # frames also refused after a broken-up one. assumed: clip-1 bursts run on past the detected frame
    IMP_FLOOR = 0.003         # floor of the impulse-noise sigma (share of a band's pixels). assumed
    BRIGHT_FLOOR = 0.5        # floor of the band-brightness sigma (grey). assumed: tuned on clip-1 (tests/clip_replay.txt)
    V_MAX = 3.0               # m/s; fastest drift between two checked frames. assumed: position hold; raise it if the hold is loose
    MOVE_TOL = 0.3            # share of the attitude-predicted image motion that may be wrong (timing, lens). assumed
    MOVE_PX = 1.5             # px of image motion that proves the picture is live. assumed
    LIVE_STILL_PX = 0.3       # px; attitude and picture both this still since the last proof: a still hover, also live
                              # (a frozen picture is caught once the attitude moves). assumed: about 0.1 deg of tilt
    LIVE_STILL_PIC_PX = 1.5   # px; picture motion allowed in a "still" proof. Was LIVE_STILL_PX (0.3): camera vibration that
                              # the FC does not see (sim 0.2 deg = 0.7 px) failed it in calm indoor hovers, and gaps without
                              # fixes of up to 2.2 s followed. A frozen picture passes the picture part either way (it
                              # does not move); only the attitude part protects, and it is unchanged. Open-loop replay
                              # of in_tiles_10m_s1 (integ/indoor/diag_live_ab.txt): gaps 5.7 -> 3.9 s per 40 s; frozen
                              # pictures sent 26 / 75 (moving / perfectly still hover) before and after.
    STILL_PX = 0.3            # px; a point that moves less while the attitude predicts motion is static (overlay)
    LIVE_S = 1.0              # s; send only within this time after the last proof that the picture is live. assumed
    LIVE_MOVE_PX = 0.5        # px; "moved" proof: the corner points that LK measured in this frame (each passed the LK,
                              # back-check, mask and NCC tests; on the whole-picture path its own corner check) moved this
                              # much (median point: shift, turn, scale or rolling-shutter wobble) since the last "moved"
                              # proof or the reference frame. Only measured points: on the whole-picture path every point
                              # moves by one estimated shift plus the attitude prediction, and on a static overlay under a
                              # turning attitude that model error gave false proofs (integ/gap/results/live). Noise-only
                              # points fail the NCC test. 0.5 px is one LK noise floor (FIT_FLOOR_PX); a frozen picture with
                              # fresh noise moves its measured points 0.02 px (digital) or 0.08 px (analog) at most
                              # (integ/gap). It keeps the reference frame, so the still proof and the frozen test work as
                              # before. Why: in a hover by hand the still proof ends after 0.3 px of attitude motion and
                              # MOVE_PX needs 0.47 m of drift at 100 m, so fixes stopped for seconds before the switch.
    LIVE_MOVED_S = 0.5        # s; a "moved" proof shows that one frame was live, so it lasts only this long: the age of a
                              # frozen fix that the safety requirement accepts (tests_apb/test_live.py, freeze with fresh
                              # noise). A moving live picture gives a new proof on most frames (integ/gap)
    FIT_FLOOR_PX = 0.5        # px; floor of the per-axis residual sigma in the fit. assumed: LK noise
    OUT_STRIKES = 3           # a keyframe point that is a fit outlier in this many frames in a row is dropped
    # Variance model (E4, review/estimator/honest.py), per axis.
    SIG_TH = math.radians(0.13)   # per-frame attitude error the camera sees (sim 0.12-0.14 deg); raised online from the fix jitter
    SIG_BIAS = math.radians(0.2)  # FC tilt-bias wander. assumed: e7_long_hover test value; measure in a 60 s static FC log
    TAU_BIAS = 30.0               # s, time constant of that wander. assumed: as SIG_BIAS
    BETA_UNC = math.radians(0.6)  # uncalibrated mount / FC level error (sim 0.58 deg). assumed: mount tolerance
    HGT_ERR = 1.0                 # m baro offset; scales every move from the keyframe. assumed: set from the event-day baro drift
    TIME_ERR = 0.003              # s; frame-to-attitude timing error left after calibration (H13: h x rate x this). assumed
    N_EFF = 3.0                   # tracked points per independent error (E4: n_eff ~ n / 3)
    DEGRADED_K = 4.0              # covariance factor for 'degraded' fixes (E10)
    LOST_DRIFT = 2.0              # m/s; drift allowance per second without a fix, added to a self-made anchor. assumed
    N_KF = 4                      # older keyframes kept for revisits, besides the home keyframe
    HOME_EVERY = 1.0              # s; how often to try the home keyframe again
    # Whole-picture path (low texture, a few strong marks): used when the corner points are too few. It
    # measures one shift of the whole picture against the keyframe, after the attitude-predicted motion is
    # removed, and moves a grid of virtual keyframe points by it. Rotation and scale come from the attitude
    # and the height, not from the picture.
    DENSE_SCALE = 0.25            # the whole-picture path works on a picture of this size (0.5: same error, 2x the compute)
    DENSE_GRID = (8, 6)           # virtual keyframe points, spread over the central 80 % of the picture
    DENSE_RESP = 0.4              # phase-correlation peak under this: no fix (ground 1 %: 0.53-0.93; static noise and
                                  # NO SIGNAL screen 99 %: 0.23-0.27)
    DENSE_NCC = 0.25              # match quality of the aligned whole picture under this: no ground (ground 1 %: 0.88,
                                  # 6 lights at night at 100 m: 0.39; static noise and NO SIGNAL screen 99 %: 0.06-0.17)
    DENSE_AGREE_PX = 4.0          # px; top and bottom halves must agree to this (ground 95 %: 2.0-4.3, camera vibration)
    DENSE_PTS_PX = 2.0            # px; corner points that LK tracked must agree with the shift to this. assumed: LK and
                                  # the shift each err under 1 px on ground (fit floor 0.5 px)
    DENSE_SIG_PX = 0.15           # px (full picture); floor of the shift's sigma
    DENSE_YAW = math.radians(20)  # a whole-picture keyframe is replaced after this yaw change: the path does not
                                  # measure rotation, so an FC yaw error turns into a position error. assumed: as the
                                  # ~15 deg revisit limit of the corner path
    # Brightness gate: > 0 = no unforced keyframe change (and no keyframe asked by request_keyframe) until the
    # mean picture brightness stayed within KF_BRIGHT grey for KF_BRIGHT_S. The Justice D has auto exposure (WDR)
    # and switches from colour to black and white at night (Justice D datasheet v1.0); a keyframe taken while the
    # exposure still moves is assumed to match the next frames worse. 0 = off. assumed: not measured on the camera.
    KF_BRIGHT = 0.0               # grey
    KF_BRIGHT_S = 0.5             # s
    # > 0: after more than GAP_JUMP_S s with no picture (a video outage), the break-up test has no previous frame
    # to compare, so its jump cue counts as unknown for that one frame (the level cue still counts). Off (0): the
    # brightness change of the whole gap looks like a jump, the first frame after the gap is "broken up", Hold is
    # lost at once (t_good is older than LOST_S) and Pipe re-anchors. ArduCopter SITL, 3 s drop at 60 m
    # (apsim/runs/rec_60m_s1_onboard.csv): "band brightness z 13", two re-anchors, the fix error shifted 0.3-0.6 m.
    GAP_JUMP_S = 0.0              # s

    ROW_NORM = False              # divide each picture row by its (smoothed) mean grey before tracking: removes the moving
                                  # bands of mains-lamp flicker on a rolling shutter (indoors). Off by default: tested only
                                  # on the indoor floors (integ/indoor/diag_flicker_people.txt); the picture checks
                                  # (repeat, break-up) still see the raw picture
    # DENSE_LOCAL: the low-height fix of the whole-picture path (integ/lowh/proposed.patch, integ/kfmap/README.txt).
    # True: J (px per m of drone move) is taken at the last fix with a central step of 1 % of the height, and the
    # halves gap is measured in px of each half's own texture. False: J at the keyframe with a 1 m forward step, gap
    # at the picture centre's scale (the code before 2026-11-21).
    DENSE_LOCAL = False
    # KF_MAP: a keyframe map and relocalization (integ/kfmap/README.txt). True: reanchor() keeps the keyframes of the
    # chain that ends (home, older ones, the last one) as a stored map with their anchor positions (at most MAP_KEEP).
    # While Hold sends fixes on a new chain, every RELOC_EVERY s it tries one stored keyframe with the re-find of the
    # 'lost' path (deep track, then ORB): the stored home, or the newest stored keyframe when the home was taken at
    # another height (the climb). A try passes when the fit is 'ok' (points, inlier share, spread), its yaw and scale
    # are near 1 and 0 (RELOC_YAW, RELOC_SCALE), and the step between the map position and this fix is under
    # max(RELOC_STEP_M, RELOC_STEP_H x h). RELOC_N passes in a row whose steps agree to RELOC_AGREE_M + RELOC_AGREE_H x h
    # switch Hold to the stored keyframe; the step is then ramped to zero at RELOC_RATE m/s (the fix does not jump),
    # with the ramp's size added to the variance. Each try is logged (reloc_log, note). False: no map (as before).
    KF_MAP = False
    MAP_KEEP = 6                  # stored keyframes, the stored home included. assumed
    RELOC_EVERY = 0.5             # s between two tries. assumed
    RELOC_N = 3                   # passing tries in a row. assumed
    RELOC_AGREE_M = 0.15          # m; the steps of these tries agree to this + RELOC_AGREE_H x h. assumed
    RELOC_AGREE_H = 0.02
    RELOC_STEP_M = 1.0            # m; largest step accepted: max(RELOC_STEP_M, RELOC_STEP_H x h). The EKF refused a 20 m
    RELOC_STEP_H = 0.2            # step for 7 s (apsim/safety_results.txt case d); the ramp never sends a step. assumed
    RELOC_RATE = 0.3              # m/s; speed of the ramp. assumed
    RELOC_YAW = math.radians(3.0)  # largest fit yaw of a passing try (rad). assumed
    RELOC_SCALE = 0.05            # largest |fit scale - 1| of a passing try. assumed
    RELOC_HGT = 0.15              # a stored keyframe is tried only within this share of its height. assumed (the fit
                                  # refuses a scale change over 0.2)

    def __init__(self, K, D, R_bc, readout_s, mask=None, **cfg):
        """mask: uint8 H x W, 0 = not ground (OSD text, drone parts). cfg: tuning overrides by keyword."""
        for k, v in cfg.items():
            if not hasattr(Hold, k.upper()):
                raise ValueError(f"Hold: unknown setting {k!r}")
            setattr(self, k.upper(), v)
        self.K, self.D, self.R_bc, self.readout = np.asarray(K, float), np.asarray(D, float), R_bc, readout_s
        self.mask, self.keep = mask, None
        if mask is not None:          # no point whose LK window touches a masked pixel
            w = 2 * (self.LK_WIN // 2) + 1
            self.keep = cv2.erode((np.asarray(mask) > 0).astype(np.uint8) * 255, np.ones((w, w), np.uint8))
        self.lk = dict(winSize=(self.LK_WIN, self.LK_WIN), maxLevel=self.LK_LEVELS,
                       minEigThreshold=LK["minEigThreshold"], criteria=LK["criteria"])
        self.vel = np.zeros(2)        # always zero: the velocity filter is gone (E11); kept for old scripts
        self.rate, self.frames, self.cues, self.recent, self.last_img, self.break_left = 0.0, 0, [], [], None, 0
        self.prev_px = np.zeros((0, 1, 2), np.float32)
        self.sig2 = self.SIG_TH ** 2  # per-frame attitude error seen in the fix jitter (rad^2)
        self.lost, self.t_good = False, None
        self._dm = {}                 # ground mask at the whole-picture size, by shape
        self.bright = []              # (tref, mean grey) of recent good pictures, for KF_BRIGHT
        self.t_pic = None             # tref of the last picture that _picture saw, for GAP_JUMP_S
        # KF_MAP state (unused when KF_MAP is False): stored keyframes and their home, whether the current chain is in
        # the map's frame, passing tries in a row, the ramp offset (m, None = none), the log of tries, this frame's note
        self.map, self.map_home, self.in_map, self.reloc_run, self.reloc_log = [], None, True, [], []
        self.ramp, self.t_ramp, self.t_reloc, self.n_reloc, self.note = None, None, -math.inf, 0, None
        self.reanchor(np.zeros(2))

    def reanchor(self, pos, var=None):
        """Start a new anchor chain at pos (NED north, east, m), e.g. the FC's estimate after a loss.

        var: the uncertainty of pos, as the trace of its 2x2 covariance (m^2, like Est.var) or a 2x2 array.
        Forgets every keyframe; the first keyframe made after this becomes the home keyframe.
        KF_MAP: the keyframes of the chain that ends are kept as a stored map first (_store_map).
        """
        if self.KF_MAP:
            self._store_map()
        self.pos, self.t, self.kf, self.home, self.last, self.kfs = np.asarray(pos, float).copy(), None, None, None, None, []
        self.carried = 0.0 if var is None else float(np.trace(var) if np.ndim(var) == 2 else var) / 2
        self.var_axis, self.t_fix = self.carried, None
        self.t_live, self.t_switch, self.t_home, self.t_moved = -math.inf, -math.inf, -math.inf, -math.inf
        self.kf_wanted = False        # the first keyframe of a new anchor is the home keyframe anyway

    def request_keyframe(self):
        """Take a new keyframe at the next good frame that turns slower than KF_RATE (and passes the KF_BRIGHT
        gate), and make it the home keyframe. For the pilot's switch: the pilot steadies the aircraft, then engages
        the hold (Stage 1 package, rev 1.3, page 2), so the hold measures against a picture of the hold point. The
        change keeps the position continuous, as every keyframe change does: no re-anchor, no wait for liveness."""
        self.kf_wanted = True

    def _bright_settled(self, tref):
        """True when the picture brightness stayed within KF_BRIGHT grey for the last KF_BRIGHT_S (or the gate is off)."""
        if self.KF_BRIGHT <= 0:
            return True
        w = [b for t, b in self.bright if t >= tref - self.KF_BRIGHT_S]
        covered = self.bright and self.bright[0][0] <= tref - 0.9 * self.KF_BRIGHT_S
        return bool(covered and w) and max(w) - min(w) <= self.KF_BRIGHT

    # ---- keyframe map (KF_MAP) ----
    def _store_map(self):
        """KF_MAP, called by reanchor(): when the chain that ends is in the map's frame (the first chain, or a chain
        that relocalized), its keyframes join the map with their anchor positions. A chain that started at a
        re-anchor and never relocalized is in another frame: its keyframes are dropped, as before. Except when no stored
        keyframe can be tried at the height of that chain's newest keyframe (a map made in the climb): that chain then
        replaces the map."""
        new = getattr(self, "last", None) or getattr(self, "kf", None)
        if not self.in_map and new is not None and not new["pending"] and \
                not any(abs(m["h"] / new["h"] - 1) <= self.RELOC_HGT for m in self.map):
            self.map, self.map_home, self.in_map = [], None, True
        if self.in_map and hasattr(self, "kfs"):          # (no chain yet at the first reanchor of __init__)
            ks = []
            for k in [self.home] + self.kfs + [self.last, self.kf]:
                if k is not None and not k["pending"] and all(k is not x for x in ks):
                    ks.append(k)
            self.map = [m for m in self.map if all(m is not k for k in ks)] + ks
            if self.home is not None and not self.home["pending"]:
                self.map_home = self.home
            rest = [m for m in self.map if m is not self.map_home]
            rest = rest[-(self.MAP_KEEP - 1):] if self.MAP_KEEP > 1 else []
            self.map = ([self.map_home] if self.map_home is not None and self.MAP_KEEP > 0 else []) + rest
        self.in_map = not self.map
        self.reloc_run, self.ramp, self.t_reloc = [], None, -math.inf

    def _reloc_log(self, row):
        self.reloc_log = self.reloc_log[-1999:] + [row]
        self.note = (f"reloc {'ACCEPTED' if row['accepted'] else 'pass' if row['ok'] else 'refused'}: "
                     f"step {row['step']} m, n {row['n']}" + (f", {row['why']}" if row["why"] else ""))

    def _reloc(self, img, t0, att, hgt, tref, h, pos_now):
        """KF_MAP: one try of a stored keyframe on this frame, which gave a good fix (pos_now) on the current chain.
        Accepts after RELOC_N passing tries in a row (see KF_MAP). Returns True when Hold switched to the map."""
        self.t_reloc = tref
        row = dict(t=round(float(tref), 3), kf_t=None, kf_h=None, ok=False, accepted=False, step=None, n=None,
                   phi_deg=None, scale=None, why=None)
        cand = [k for k in self.map if abs(k["h"] / h - 1) <= self.RELOC_HGT]
        k0 = self.map_home if any(self.map_home is k for k in cand) else (cand[-1] if cand else None)
        if k0 is None:
            self.reloc_run, row["why"] = [], f"no stored keyframe within {self.RELOC_HGT:.0%} of {h:.1f} m"
            self._reloc_log(row)
            return False
        row.update(kf_t=round(float(k0["chain"]["t"]), 3), kf_h=round(k0["h"], 2), home=k0 is self.map_home)
        k = dict(k0, alive=k0["alive"].copy(), out=k0["out"].copy())    # a try does not change the stored keyframe
        rr = self._track(k, img, t0, att, hgt, tref, deep=True)          # the re-find of the 'lost' path
        # (ORB start only for a corner keyframe: _track gives a whole-picture keyframe no start, so a second call there
        # would repeat the first at the cost of two ORB passes)
        if not (isinstance(rr, dict) and rr["n"] >= 2 * self.MIN_PTS) and not k["dense"]:
            start = self._orb_guess(k, img)
            rr = self._track(k, img, t0, att, hgt, tref, deep=True, start=start) if start is not None else rr
        for key in ("dz", "orb"):                                         # caches of the keyframe's own picture
            if key in k and key not in k0:
                k0[key] = k[key]
        lim = max(self.RELOC_STEP_M, self.RELOC_STEP_H * h)
        if isinstance(rr, str):
            why = rr
        else:
            spread = math.sqrt(np.trace(rr["res_cov"]))
            step = np.asarray(pos_now, float) - rr["pos"]
            row.update(n=rr["n"], step=round(float(np.linalg.norm(step)), 3), phi_deg=round(math.degrees(rr["phi"]), 2),
                       scale=round(float(rr["s"]), 4), dense=rr["dvar"] is not None, pos=[round(float(x), 3) for x in rr["pos"]])
            why = (f"{rr['n']} points (need {2 * self.MIN_PTS})" if rr["n"] < 2 * self.MIN_PTS else
                   f"inlier share {rr['frac']:.2f}" if rr["frac"] < 0.6 else
                   f"spread {spread:.2f} m" if spread >= 0.02 * h + 0.2 else
                   f"yaw {math.degrees(rr['phi']):.1f} deg" if abs(rr["phi"]) > self.RELOC_YAW else
                   f"scale {rr['s']:.3f}" if abs(rr["s"] - 1) > self.RELOC_SCALE else
                   f"step over {lim:.2f} m" if np.linalg.norm(step) >= lim else None)
        if why is not None:
            self.reloc_run, row["why"] = [], why
            self._reloc_log(row)
            return False
        self.reloc_run = (self.reloc_run + [step])[-self.RELOC_N:]
        row["ok"] = True
        if len(self.reloc_run) < self.RELOC_N:
            row["why"] = f"pass {len(self.reloc_run)} of {self.RELOC_N}"
            self._reloc_log(row)
            return False
        st = np.array(self.reloc_run)
        dev = float(np.linalg.norm(st - st.mean(0), axis=1).max())
        tol = self.RELOC_AGREE_M + self.RELOC_AGREE_H * h
        row["agree_m"] = round(dev, 3)
        if dev > tol:
            self.reloc_run = self.reloc_run[1:]
            row["why"] = f"tries disagree ({dev:.2f} m over {tol:.2f} m)"
            self._reloc_log(row)
            return False
        # Accepted: the stored keyframe (with the try's point state) becomes the current one; the map's home stays the
        # home; the keyframes of the new chain (another frame) are dropped. The output keeps the current position and
        # moves to the map's frame at RELOC_RATE (process: self.ramp).
        k0["alive"], k0["out"] = k["alive"], k["out"]
        k0["ref"] = dict(t=tref, px=rr["p2"], pred=rr["pred"], seen=rr["meas"])
        self.home = self.map_home if self.map_home is not None else k0
        others = [m for m in self.map if m is not self.home and m is not k0]
        self.kfs = others[-self.N_KF:] if self.N_KF > 0 else []
        self.kf, self.t_switch, self.last = k0, tref, None
        self.ramp, self.t_ramp = np.asarray(pos_now, float) - rr["pos"], tref
        self.in_map, self.reloc_run, self.n_reloc = True, [], self.n_reloc + 1
        row["accepted"] = True
        self._reloc_log(row)
        return True

    # ---- geometry ----
    def _undistort(self, px):
        p = np.asarray(px, np.float64).reshape(-1, 1, 2)
        n = cv2.fisheye.undistortPoints(p, self.K, self.D) if len(self.D) == 4 else cv2.undistortPoints(p, self.K, self.D)
        return n.reshape(-1, 2)

    def _project(self, v, R_nb):
        """Pixels of NED camera-to-point vectors v (N x 3) with body->NED rotations R_nb (N x 3 x 3)."""
        c = (np.einsum("nji,nj->ni", R_nb, v) @ self.R_bc).reshape(-1, 1, 3)
        z = np.zeros(3)
        p = (cv2.fisheye.projectPoints(c, z, z, self.K, self.D)[0] if len(self.D) == 4 else
             cv2.projectPoints(c, z, z, self.K, self.D)[0])
        return p.reshape(-1, 2)

    def _rays(self, px, t0, rows, att, hgt):
        """Horizontal camera-to-ground offsets (NED, m) for pixels px, plus row times and heights.

        Row r is taken at t0 + r / rows * readout: t0 is the mid-exposure time of row 0 (G4), and a
        negative readout means the sensor reads bottom to top (G6).
        """
        t = t0 + px[:, 1] / rows * self.readout
        f = np.c_[self._undistort(px), np.ones(len(px))] @ self.R_bc.T
        b = np.einsum("nij,nj->ni", attitude_R(att, t), f)
        h = hgt.at(t)
        ok = b[:, 2] > math.cos(self.MAX_OFF_NADIR) * np.linalg.norm(b, axis=1)
        return h[:, None] * b[:, :2] / np.maximum(b[:, 2:3], 1e-6), t, ok, h

    def _patches(self, img, p):
        """11 x 11 patches around p, zero mean and unit norm, for the NCC test."""
        n = len(p)
        mx = (p[:, 0, None, None] + np.zeros((1, 11, 1), np.float32) + _PATCH[None, None, :]).reshape(-1, 11)
        my = (p[:, 1, None, None] + _PATCH[None, :, None] + np.zeros((1, 1, 11), np.float32)).reshape(-1, 11)
        v = cv2.remap(img, mx.astype(np.float32), my.astype(np.float32), cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_REPLICATE).reshape(n, 121).astype(np.float32)
        v -= v.mean(1, keepdims=True)
        return v / (np.linalg.norm(v, axis=1, keepdims=True) + 1e-6)

    # ---- picture and input checks ----
    def _picture(self, img, tref=None):
        """Why this picture cannot be used (repeated or broken up), else None (E2, H4, real video)."""
        gap = self.GAP_JUMP_S > 0 and tref is not None and self.t_pic is not None and tref - self.t_pic > self.GAP_JUMP_S
        self.t_pic = tref if tref is not None else self.t_pic
        last, self.last_img = self.last_img, img
        if last is not None and last.shape == img.shape and \
                cv2.norm(img, last, cv2.NORM_L1) < self.REPEAT_DIFF * img.size:
            return "repeated frame"
        # Breakup: impulse noise (pixels far from their 3x3 median) in the worst of 8 bands, and the
        # brightness of each band. Broken up = some cue is off its level AND some cue jumped from the
        # previous frame, each by BREAK_Z robust sigma against the last BREAK_HIST accepted frames
        # (motion changes the level slowly, a breakup changes it at once). A change that holds steady
        # for BREAK_K frames is a new level (light, exposure), not a breakup.
        hb = img.shape[0] // 8 * 8
        imp = cv2.absdiff(img, cv2.medianBlur(img, 3))[:hb] > 25
        x = np.r_[imp.reshape(8, -1).mean(1).max(), img[:hb].reshape(8, -1).mean(1)]
        dx = np.abs(x - self.recent[-1]) if self.recent and not gap else np.zeros_like(x)
        self.recent = self.recent[-(self.BREAK_K - 1):] + [x]
        if len(self.cues) >= 10:
            h = np.array(self.cues)
            floor = np.r_[self.IMP_FLOOR, np.full(8, self.BRIGHT_FLOOR)]
            z = [np.abs(v - m) / np.maximum(1.4826 * np.median(np.abs(a - m), axis=0), floor)
                 for v, a in ((x, h[:, 0]), (dx, h[:, 1])) for m in [np.median(a, axis=0)]]
            if (z[0] > self.BREAK_Z).any() and (z[1] > self.BREAK_Z).any():
                r, med = np.array(self.recent), np.median(h[:, 0], axis=0)
                if len(r) < self.BREAK_K or (np.ptp(r, axis=0) > np.maximum(4 * floor, 0.25 * np.abs(np.median(r, axis=0) - med))).any():
                    self.break_left = self.BREAK_AFTER
                    return f"picture broken up (impulse z {z[0][0]:.0f}, band brightness z {z[0][1:].max():.0f})"
                self.cues = [np.array([v, np.zeros_like(v)]) for v in r]      # a lasting new level
        self.cues = self.cues[-(self.BREAK_HIST - 1):] + [np.array([x, dx])]
        if self.break_left > 0:
            self.break_left -= 1
            return "picture just after a broken-up one"
        return None

    def _check_inputs(self, t0, att, hgt):
        lo, hi = t0 + min(self.readout, 0.0), t0 + max(self.readout, 0.0)
        if att.last_t() < hi - 0.005:
            return "attitude late"
        if hgt.last_t() < lo - 0.5:
            return "height late"
        a = att.array()
        if a.shape[1] >= 7:           # gyro rates from ATTITUDE (rollspeed, pitchspeed, yawspeed)
            self.rate = float(max(abs(np.interp((lo + hi) / 2, a[:, 0], a[:, k])) for k in (4, 5, 6)))
        else:                         # attitude change across the frame: less noisy than single steps (E6)
            ends = [np.interp([lo - 0.03, hi + 0.03], a[:, 0], c) for c in (a[:, 1], a[:, 2], unwrapped_yaw(att, a))]
            self.rate = float(max(abs(e[1] - e[0]) for e in ends) / (hi - lo + 0.06))
        if self.rate > self.MAX_RATE:
            return "rotating too fast"
        return None

    # ---- keyframes ----
    def _corners(self, img):
        """Up to MAX_PTS corners, at most MAX_PTS / cells per grid cell, best first (E2, E3)."""
        n, (gx, gy) = self.MAX_PTS, self.GRID
        if self.keep is None:     # 4 positional arguments: clip_tool.MaskedHold replaces this call
            p = cv2.goodFeaturesToTrack(img, 4 * n, 0.005, 7)
        else:
            p = cv2.goodFeaturesToTrack(img, 4 * n, 0.005, 7, mask=self.keep)
        if p is None:
            return None
        p = p.reshape(-1, 2)
        H, W = img.shape
        cell = (p[:, 1] * gy // H).astype(int) * gx + (p[:, 0] * gx // W).astype(int)
        order = np.argsort(cell, kind="stable")
        c = cell[order]
        rank = np.arange(len(c)) - np.searchsorted(c, c)
        return p[np.sort(order[rank < -(-n // (gx * gy))])][:n].astype(np.float32)

    def _make_kf(self, img, t0, att, hgt, tref, pos, acc, chain, pending):
        """Corner points plus a grid of virtual points (virt) for the whole-picture path. A keyframe with too
        few corners starts in the whole-picture mode (dense); the picture's own checks then decide."""
        px = self._corners(img)
        px = np.zeros((0, 2), np.float32) if px is None else px
        (gx, gy), (H, W) = self.DENSE_GRID, img.shape
        u, v = np.meshgrid((0.1 + 0.8 * (np.arange(gx) + 0.5) / gx) * (W - 1), (0.1 + 0.8 * (np.arange(gy) + 0.5) / gy) * (H - 1))
        grid = np.c_[u.ravel(), v.ravel()].astype(np.float32)
        if self.keep is not None:
            grid = grid[self.keep[grid[:, 1].astype(int), grid[:, 0].astype(int)] > 0]
        virt = np.r_[np.zeros(len(px), bool), np.ones(len(grid), bool)]
        px = np.r_[px, grid]
        g, t, ok, h = self._rays(px, t0, img.shape[0], att, hgt)
        if (ok & virt).sum() < self.MIN_PTS:
            return None
        px, G, t, h, virt = px[ok], pos + g[ok], t[ok], h[ok], virt[ok]
        pred = self._project(np.c_[G - pos, h], attitude_R(att, t))
        a = att.array()
        return dict(img=img, px=px, G=G, pos=np.array(pos, float), alive=np.ones(len(px), bool), out=np.zeros(len(px), int),
                    pat=self._patches(img, px), n0=len(px), h=float(np.mean(h)), acc=acc, chain=chain,
                    pending=pending, n_first=None, var=2 * (acc + chain["carried"]), virt=virt,
                    dense=(~virt).sum() < self.MIN_PTS, ref=dict(t=tref, px=px.astype(float), pred=pred),
                    yaw=float(np.interp(tref, a[:, 0], unwrapped_yaw(att, a))))

    def _activate(self, kf, tref):
        old = self.kf
        keep = [k for k in self.kfs if k is not old and k is not kf]
        if old is not None and old is not kf and not old["pending"] and old is not self.home:
            keep.append(old)
        self.kfs = keep[-self.N_KF:] if self.N_KF > 0 else []
        self.kf, self.t_switch = kf, tref

    # ---- tracking ----
    def _orb_guess(self, kf, img):
        """Where kf's points are in img, from ORB matches and a RANSAC similarity. Used to re-find a
        keyframe after a gap when the attitude prediction is too far off for LK."""
        orb = cv2.ORB_create(500)
        if "orb" not in kf:
            kf["orb"] = orb.detectAndCompute(kf["img"], self.keep)
        (k0, d0), (k1, d1) = kf["orb"], orb.detectAndCompute(img, self.keep)
        if d0 is None or d1 is None:
            return None
        m = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(d0, d1)
        if len(m) < self.MIN_PTS:
            return None
        p0 = np.float32([k0[x.queryIdx].pt for x in m])
        p1 = np.float32([k1[x.trainIdx].pt for x in m])
        M, inl = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC, ransacReprojThreshold=3.0)
        if M is None or inl.sum() < self.MIN_PTS:
            return None
        return (kf["px"] @ M[:, :2].T + M[:, 2]).astype(np.float32)

    def _dmask(self, shape):
        """Ground mask (bool) at the whole-picture size, or None."""
        if self.mask is None:
            return None
        if shape not in self._dm:
            self._dm[shape] = cv2.resize(np.asarray(self.mask), shape[::-1], interpolation=cv2.INTER_NEAREST) > 0
        return self._dm[shape]

    def _small(self, img):
        s = self.DENSE_SCALE
        a = cv2.resize(img, (round(img.shape[1] * s), round(img.shape[0] * s)), interpolation=cv2.INTER_AREA).astype(np.float32)
        m = self._dmask(a.shape)
        if m is not None:
            a[~m] = np.nan
        return a

    def _dense(self, kf, img, idx, guess, J, p_lk):
        """Positions of kf points idx in img from one whole-picture match (low texture, a few strong marks).

        The current picture is warped onto the keyframe with a smooth (cubic) map fitted to the attitude
        prediction of the points (guess). What is left is the drone's move on the ground, delta (m, north and
        east). On a fisheye one ground move is not one pixel shift, so the map also carries how each point moves
        per metre of drone move (J, N x 2 x 2 px/m), and delta is solved in metres. Phase correlation finds a
        first delta (peak quality: DENSE_RESP); a Lucas-Kanade step on the whole picture refines it in the top
        and the bottom half separately (they must agree: DENSE_AGREE_PX); the aligned pictures must match
        (DENSE_NCC); corner points that LK tracked (p_lk, NaN where not) must agree (DENSE_PTS_PX).
        Returns (p2, variance of delta per axis in m^2) or a string saying why not.
        """
        if "dz" not in kf:
            a = self._small(kf["img"])
            v = np.isfinite(a)
            a[~v] = float(a[v].mean())
            ys, xs = np.mgrid[0:a.shape[0], 0:a.shape[1]].astype(np.float32)
            kf["dz"] = dict(a=a, v=v, gx=cv2.Sobel(a, cv2.CV_32F, 1, 0, ksize=3, scale=0.125),
                            gy=cv2.Sobel(a, cv2.CV_32F, 0, 1, ksize=3, scale=0.125),
                            win=cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F), xy=np.dstack([xs, ys]),
                            B=_cubic(np.c_[xs.ravel(), ys.ravel()] / self.DENSE_SCALE, kf["img"].shape))
        z, s = kf["dz"], self.DENSE_SCALE
        # A homography cannot follow the fisheye: the map from keyframe to current pixels, and its change per
        # metre of drone move, are cubics in x, y fitted to every point (rolling-shutter shear included).
        P = np.linalg.lstsq(_cubic(kf["px"][idx], img.shape), np.c_[guess - kf["px"][idx], J.reshape(-1, 4)], rcond=None)[0]
        M = (z["B"] @ P).astype(np.float32).reshape(z["a"].shape + (6,)) * s
        base, JN, JE = z["xy"] + M[..., :2], M[..., [2, 4]], M[..., [3, 5]]     # small px; small px per m
        Jc = P[0, 2:].reshape(2, 2)                      # full px per m at the picture centre (columns N, E)
        if abs(np.linalg.det(Jc)) < 1e-9:
            return "whole picture: no map"
        jN = z["gx"] * JN[..., 0] + z["gy"] * JN[..., 1]   # grey per m of drone move north (keyframe gradients)
        jE = z["gx"] * JE[..., 0] + z["gy"] * JE[..., 1]
        b = self._small(img)

        def warp(dl):     # warp(dl)(x) = current(map(x) + J(x) dl), NaN outside the picture and on the mask
            m = base + JN * np.float32(dl[0]) + JE * np.float32(dl[1])
            w = cv2.remap(b, m[..., 0], m[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=float("nan"))
            return w, np.isfinite(w) & z["v"]
        w, v = warp((0.0, 0.0))
        if v.mean() < 0.5:
            return "whole picture: keyframe out of view"
        w = np.where(v, w - float(w[v].mean()), 0.0).astype(np.float32)
        # Zero mean first: the window times the mean grey is the same in both pictures and makes a false peak at
        # zero shift (static noise peaked at 0.6 without it). cv2.phaseCorrelate windows its inputs in place.
        d, resp = cv2.phaseCorrelate(z["a"] - float(z["a"].mean()), w, z["win"])
        self.dq = [resp, None, None]                    # last whole-picture quality (peak, halves gap px, match), for logs
        if not resp >= self.DENSE_RESP:
            return f"whole picture: weak peak ({resp:.2f})"
        dl0 = np.linalg.solve(s * Jc, np.array(d))       # the shift at the centre, as a drone move
        half, ds, cov, P = z["a"].shape[0] // 2, [], [], 0.0
        for rows in (slice(0, half), slice(half, None)):
            dl = dl0.copy()
            for _ in range(2):           # Lucas-Kanade on the whole half, in metres of drone move
                w, v = warp(dl)
                v = v[rows]
                e, j1, j2 = (w[rows] - z["a"][rows])[v], jN[rows][v], jE[rows][v]
                A = np.array([[j1 @ j1, j1 @ j2], [j1 @ j2, j2 @ j2]], float)
                if len(e) < 100 or np.linalg.eigvalsh(A)[0] < 1e-6 * len(e):
                    return "whole picture: too flat"
                dl = dl - np.linalg.solve(A, [j1 @ e, j2 @ e])
            ds.append(dl)
            cov.append(float(e @ e) / len(e) * np.linalg.inv(A))
            if self.DENSE_LOCAL:
                P = P + np.linalg.inv(A) * float(np.sum(z["gx"][rows][v] ** 2 + z["gy"][rows][v] ** 2)) / 2   # m^2 per px^2
        if self.DENSE_LOCAL:
            # The gap in px of each half's own texture, not at the picture centre's scale (equal when J is the same
            # everywhere). A fisheye moves its edge less per metre, so a picture shift of 1-3 px (camera vibration the FC
            # does not see) in texture near the edge read as 4-8 px at the centre's scale (5 m, coarse ground; integ/lowh).
            dd = ds[0] - ds[1]
            gap = math.sqrt(2 * float(dd @ np.linalg.solve(P, dd))) / s
        else:
            gap = float(np.linalg.norm(Jc @ (ds[0] - ds[1])))
        self.dq[1] = gap
        if gap > self.DENSE_AGREE_PX:
            return f"whole picture: halves disagree ({gap:.1f} px)"
        dl = (ds[0] + ds[1]) / 2
        w, v = warp(dl)
        rho = float(np.corrcoef(w[v], z["a"][v])[0, 1]) if v.sum() > 100 else 0.0
        self.dq[2] = rho
        if not rho >= self.DENSE_NCC:
            return f"no ground (whole picture match quality {rho:.2f})"
        p2 = guess + J @ dl
        lk = np.isfinite(p_lk[:, 0])
        if lk.sum() >= 3:
            off = float(np.linalg.norm(_med(p_lk[lk] - p2[lk])))
            if off > self.DENSE_PTS_PX:
                return f"whole picture and points disagree ({off:.1f} px)"
        # Variance of delta: the fit's own (pixels are correlated, so it is optimistic), or the halves' gap if
        # larger, plus a floor of DENSE_SIG_PX at the centre's scale; m^2 per axis.
        var = max(float(np.trace(cov[0] + cov[1])) / 8, float((ds[0] - ds[1]) @ (ds[0] - ds[1])) / 8)
        return p2.astype(np.float32), var + self.DENSE_SIG_PX ** 2 / abs(np.linalg.det(Jc))

    def _track(self, kf, img, t0, att, hgt, tref, deep=False, start=None):
        """Corner points first; when too few of them are left, the whole-picture path on the same keyframe
        (it then stays in that mode: kf["dense"], and r["mode"] tells the caller to restart the reference)."""
        if not kf["dense"]:
            r = self._track1(kf, img, t0, att, hgt, tref, deep, start, False)
            if deep:              # re-finding a corner keyframe after a gap: the corners (or ORB) only. A whole-picture
                return r          # match there locked onto a moving object 27 m away (run_regression E3, seed 3)
            # too few trackable points, or no ground in the corner patches (noise corners on dark ground): the whole
            # picture may still hold a few strong marks, and it has its own ground tests. Not after "consistent
            # points" or "points left": enough points moved, but not together (a moving object, an overlay).
            if not (isinstance(r, str) and r.endswith(("used up", "tracked", "match", "in view"))
                    or isinstance(r, str) and r.startswith("no ground")):
                return r
        r = self._track1(kf, img, t0, att, hgt, tref, deep, None, True)
        if isinstance(r, dict) and not kf["dense"]:
            kf["dense"], r["mode"] = True, True
        return r

    def _track1(self, kf, img, t0, att, hgt, tref, deep, start, dense):
        """Track kf's live points into this frame. Returns a dict, or a string saying why not.

        LK starts at each point's attitude prediction (or at start, N x 2 for all kf points); deep=True
        (re-finding a keyframe after a gap) adds 2 pyramid levels. Points must pass the LK status, the backward check, the mask and the
        NCC test. dense=True: every point (virtual ones too) moves by the whole-picture shift (_dense) instead.
        Then the image motion is checked against the attitude: a frozen picture, a static
        overlay or a wrong mount moves differently from what the attitude predicts (E1, E2, H4).
        """
        idx = np.flatnonzero(kf["alive"] & (True if dense else ~kf["virt"]))
        if len(idx) < self.MIN_PTS:
            return "keyframe used up"
        rows, ref, p0 = img.shape[0], kf["ref"], kf["px"][idx]
        rp = ref["px"][idx]
        fin = np.isfinite(rp[:, 0])
        t = t0 + np.where(fin, rp[:, 1], p0[:, 1]) / rows * self.readout
        Rt, ht = attitude_R(att, t), hgt.at(t)
        pred = self._project(np.c_[kf["G"][idx] - kf["pos"], ht], Rt)
        off = rp - ref["pred"][idx]
        off[~fin] = _med(off[fin]) if fin.any() else 0.0
        guess = (pred + off).astype(np.float32) if start is None else start[idx]
        # shortcut: LK rebuilds both pyramids on every call (the Python binding takes no prebuilt pyramid).
        lk = dict(self.lk, maxLevel=self.LK_LEVELS + 2) if deep else self.lk
        a, b = kf["img"], img
        H, W = img.shape
        sel = ~kf["virt"][idx]                           # LK on the corner points only
        p2, ok = guess.copy(), np.zeros(len(idx), bool)
        if sel.any():
            q, s, _ = cv2.calcOpticalFlowPyrLK(a, b, p0[sel], guess[sel].copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk)
            q, s = q.reshape(-1, 2), s.ravel() == 1
            if deep or dense or self.frames % self.FB_EVERY == 0:
                pb, sb, _ = cv2.calcOpticalFlowPyrLK(b, a, q, p0[sel].copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk)
                s &= (sb.ravel() == 1) & (np.linalg.norm(pb.reshape(-1, 2) - p0[sel], axis=1) < 0.5)
            p2[sel], ok[sel] = q, s
        if dense:
            p_lk = np.where(ok[:, None], p2, np.nan)
            if ok.any():                                 # tracked corners must also match their keyframe patch
                k = np.flatnonzero(ok)
                p_lk[k[np.einsum("ij,ij->i", self._patches(img, p2[k]), kf["pat"][idx[k]]) < self.NCC_MIN]] = np.nan
            if kf["dense"]:                              # a corner LK lost is not tracked again (compute)
                kf["alive"][idx[sel & np.isnan(p_lk[:, 0])]] = False
            if self.DENSE_LOCAL:
                # px per m of drone move north and east, per point: taken where the drone is now (the last fix, self.pos,
                # in the keyframe's frame: without a KF_MAP ramp), with a central step of 1 % of the height, not at the
                # keyframe with a 1 m forward step (integ/lowh)
                at = self.pos if self.ramp is None else self.pos - self.ramp
                v, e = np.c_[kf["G"][idx] - at, ht], 0.01 * ht[:, None]
                J = np.stack([self._project(v - e * [1.0, 0, 0], Rt) - self._project(v + e * [1.0, 0, 0], Rt),
                              self._project(v - e * [0, 1.0, 0], Rt) - self._project(v + e * [0, 1.0, 0], Rt)], -1) / (2 * e[:, :, None])
            else:
                v = np.c_[kf["G"][idx] - kf["pos"], ht]       # px per m of drone move north and east, per point
                J = np.stack([self._project(v - [1.0, 0, 0], Rt) - pred, self._project(v - [0, 1.0, 0], Rt) - pred], -1)
            r = self._dense(kf, img, idx, guess, J, p_lk)
            if isinstance(r, str):
                return r
            p2, dvar = r
            ok = np.ones(len(idx), bool)
        ok &= (p2[:, 0] >= 0) & (p2[:, 0] <= W - 1) & (p2[:, 1] >= 0) & (p2[:, 1] <= H - 1)
        if self.keep is not None and ok.any():           # drop points that entered the mask
            q = np.round(p2[ok]).astype(int)
            ok[ok] = self.keep[q[:, 1], q[:, 0]] > 0
        if ok.sum() < self.MIN_PTS:
            return f"{ok.sum()} points tracked"
        if not dense:
            ncc = np.einsum("ij,ij->i", self._patches(img, p2), kf["pat"][idx])
            med_ncc = float(_med(ncc[ok]))
            if med_ncc < self.NCC_FRAME:
                return f"no ground (match quality {med_ncc:.2f})"
            ok &= ncc >= self.NCC_MIN
            if ok.sum() < self.MIN_PTS:
                return f"{ok.sum()} points match"
        meas = p_lk if dense else np.where(ok[:, None], p2, np.nan)   # positions LK measured (NaN: not measured)

        # Motion against attitude since the reference frame. dd: image motion the attitude and height
        # predict, dm: measured motion. lim: what drift and attitude noise can explain.
        f = float(self.K[0, 0])
        lim = (3.5 * math.sqrt(2 * max(self.SIG_TH ** 2, self.sig2)) * f + 0.3
               + self.V_MAX * (tref - ref["t"]) * f / float(hgt.at(tref)))
        com = ok & fin
        dd, dm = pred - ref["pred"][idx], p2 - rp
        dec = com & (np.linalg.norm(dd, axis=1) * (1 - self.MOVE_TOL) > lim)
        static = dec & (np.linalg.norm(dm, axis=1) < self.STILL_PX)
        if dec.sum() >= self.MIN_PTS // 2 and static.sum() > 0.5 * dec.sum():
            return "picture frozen"
        if not dense:                                    # one shift moves all points: no single static point
            kf["alive"][idx[static]] = False             # points on a static overlay (OSD, dirt)
        ok &= ~static
        com &= ~static
        live = False
        if com.sum() >= self.MIN_PTS // 2:
            th = np.linalg.norm(_med(p2[com] - guess[com]))
            if th > self.MOVE_TOL * np.linalg.norm(_med(dd[com])) + lim:
                return f"image motion does not match the attitude ({th:.1f} px)"
            # Proof that the picture is live: it moved MOVE_PX, or it moved as the attitude predicts ("move";
            # a frozen picture does not follow the attitude). A still picture under a still attitude counts too
            # ("still"), but keeps the reference frame, so the frozen test above still sees every attitude change
            # since the last real move; it ends when the attitude moves LIVE_STILL_PX.
            mm, md = np.linalg.norm(_med(dm[com])), _med(dd[com])
            if mm > self.MOVE_PX or (np.linalg.norm(md) > self.LIVE_STILL_PX and
                                     np.linalg.norm(_med(dm[com] - dd[com])) < self.MOVE_TOL * np.linalg.norm(md)):
                live = "move"
            elif mm < self.LIVE_STILL_PIC_PX and np.percentile(np.linalg.norm(dd[com], axis=1), 90) < self.LIVE_STILL_PX:
                live = "still"
            sn = ref["seen"][idx] if "seen" in ref else rp              # measured positions at the last "moved" proof
            cs = com & np.isfinite(meas[:, 0]) & np.isfinite(sn[:, 0])
            if not live and cs.sum() >= self.MIN_PTS // 2 and \
                    float(_med(np.linalg.norm(meas[cs] - sn[cs], axis=1))) > self.LIVE_MOVE_PX:
                live = "moved"
        if ok.sum() < self.MIN_PTS:
            return f"{ok.sum()} points left"

        g, _, okr, hh = self._rays(p2[ok], t0, rows, att, hgt)
        gi, g = idx[ok][okr], g[okr]
        if len(gi) < self.MIN_PTS:
            return f"{len(gi)} points in view"
        hm = float(np.mean(hh[okr]))
        C, sc, phi, inl, res = robust_fit(kf["G"][gi], g, self.FIT_FLOOR_PX * hm / f)
        n = int(inl.sum())
        kf["out"][gi] = np.where(inl, 0, kf["out"][gi] + 1)
        kf["alive"][gi[kf["out"][gi] >= self.OUT_STRIKES]] = False     # moving object, occluded point
        if n < self.MIN_PTS:
            return f"{n} consistent points"
        if abs(sc - 1) > 0.2 or abs(phi) > 0.35:
            return f"fit out of range (scale {sc:.2f}, yaw {math.degrees(phi):.0f} deg)"
        bi = g[inl]
        mb = bi.mean(0)
        full, predf, every = np.full((len(kf["px"]), 2), np.nan), np.full((len(kf["px"]), 2), np.nan), kf["px"].copy()
        full[idx[ok]], predf[idx], every[idx] = p2[ok], pred, p2
        measf = np.full((len(kf["px"]), 2), np.nan)
        measf[idx] = meas
        return dict(n=n, frac=n / len(gi), pos=C, s=sc, phi=phi, res_cov=np.cov(res[inl].T), h=hm, every=every,
                    lever=float(mb @ mb / max(np.sum((bi - mb) ** 2), 1e-9)), p2=full, pred=predf, live=live,
                    dvar=dvar if dense else None, meas=measf)

    # ---- main ----
    def _fail(self, tref, why, lost=False):
        if self.kf is not None and self.kf["pending"]:
            self.kf = None                                # a new anchor that failed is dropped
        if self.t_good is None:
            self.t_good = tref
        if (lost or tref - self.t_good > self.LOST_S) and not self.lost:
            self.lost = True
            if self.kf is not None:
                self.last = self.kf
            self.kf, self.t, self.t_live, self.t_moved = None, None, -math.inf, -math.inf   # E7: no stale time after a loss
        if why == "picture frozen" or why.startswith("image motion"):
            self.t_live = self.t_moved = -math.inf        # prove the picture is live before sending again
        return Est(tref, self.pos.copy(), None, 0, "lost" if self.lost else "degraded", False, why)

    def process(self, img, t0, att, hgt):
        """img: uint8 grey. t0: mid-exposure time of the first image row, same clock as att and hgt.

        att: Series of (t, roll, pitch, yaw[, rollspeed, pitchspeed, yawspeed]) as MAVLink ATTITUDE.
        hgt: Series of (t, height above the ground in m). Returns Est; send only when var is not None.
        """
        self.frames += 1
        self.note = None              # KF_MAP: this frame's relocalization note, for the run log
        tref = t0 + self.readout / 2
        why = self._picture(img, tref) or self._check_inputs(t0, att, hgt)
        if self.ROW_NORM:   # flicker is one gain per row (rolling shutter); the smoothing keeps the scene's row detail.
            # After the picture checks: on the normalized picture the break-up test let 5 of 90 broken frames through
            # (tests/bad_pictures.py with HOLD_CFG row_norm). Known limit: with strong moving bands, after a frozen
            # picture the break-up test can refuse live frames until the bands hold still (diag_flicker_people.txt)
            r = img.mean(axis=1, dtype=np.float32) if self.mask is None else \
                np.sum(img * (self.mask > 0), axis=1, dtype=np.float32) / np.maximum(np.sum(self.mask > 0, axis=1), 1)
            r = cv2.GaussianBlur(r.reshape(-1, 1), (1, 0), 5.0).ravel()
            img = np.clip(img * (float(r.mean()) / np.maximum(r, 1.0))[:, None] + 0.5, 0, 255).astype(np.uint8)
        if why:
            return self._fail(tref, why)
        if self.KF_BRIGHT > 0:
            self.bright = [x for x in self.bright if x[0] > tref - 2 * self.KF_BRIGHT_S] + [(tref, float(img[::8, ::8].mean()))]
        h = float(hgt.at(tref))
        if not h >= self.H_MIN:
            return self._fail(tref, f"height {h:.1f} m below {self.H_MIN:g} m", lost=True)
        a = att.array()
        yaw = float(np.interp(tref, a[:, 0], unwrapped_yaw(att, a)))
        kf, r, switched = self.kf, None, False

        if kf is None:            # start, or lost: find the last or the home keyframe again (E6, E7)
            for k in [k for k in (self.last, self.home) if k is not None][:1 if self.last is self.home else 2]:
                rr = self._track(k, img, t0, att, hgt, tref, deep=True)
                if not (isinstance(rr, dict) and rr["n"] >= 2 * self.MIN_PTS):
                    start = self._orb_guess(k, img)
                    rr = self._track(k, img, t0, att, hgt, tref, deep=True, start=start) if start is not None else rr
                if isinstance(rr, dict) and rr["n"] >= 2 * self.MIN_PTS:
                    kf, r, switched = k, rr, True
                    r["live"] = False     # the proof must come from motion after this frame
                    self._activate(k, tref)
                    break
            if r is None:         # a new anchor; nothing is sent until the picture proves it is live
                if self.lost and self.t_fix is not None:
                    self.carried = self.var_axis + (self.LOST_DRIFT * (tref - self.t_fix)) ** 2
                chain = dict(h=h, t=tref, yaw=yaw, carried=self.carried, w=self.rate)
                k = self._make_kf(img, t0, att, hgt, tref, self.pos, 0.0, chain, pending=True)
                if k is None:
                    return self._fail(tref, "not enough texture")
                self.kf, self.t_good = k, tref
                return Est(tref, self.pos.copy(), None, k["n0"], "init", False, None)
        elif self.home is not None and kf is not self.home and tref - self.t_home > self.HOME_EVERY:
            self.t_home = tref    # return to the home keyframe when it is in view again: no random walk
            rr = self._track(self.home, img, t0, att, hgt, tref, deep=True)
            if isinstance(rr, dict) and rr["n"] >= 2 * self.MIN_PTS:
                kf, r, switched = self.home, rr, True
                self._activate(kf, tref)
        if r is None:
            r = self._track(kf, img, t0, att, hgt, tref)
            if isinstance(r, str):
                return self._fail(tref, r)
        if switched or r["live"] == "move" or r.get("mode"):   # the motion test of the next frames starts from this frame
            kf["ref"] = dict(t=tref, px=r["p2"], pred=r["pred"], seen=r["meas"])
        if r["live"] == "moved":                                # the next "moved" proof needs new motion
            kf["ref"]["seen"], self.t_moved, kf["pending"] = r["meas"], tref, False
        elif r["live"]:
            self.t_live, kf["pending"] = tref, False
        self.t_good = tref
        self.prev_px = r["every"].reshape(-1, 1, 2).astype(np.float32)   # one per keyframe point, as in version 1
        pos, n = r["pos"], r["n"]
        if kf["pending"] or (tref - self.t_live > self.LIVE_S and tref - self.t_moved > self.LIVE_MOVED_S):
            return Est(tref, pos.copy(), None, n, "init" if kf["pending"] else "degraded", False,
                       "waiting for image motion that proves the picture is live")
        if self.home is None:
            self.home = kf
        # KF_MAP ramp after a relocalization: this fix is in the map's frame; the output adds the ramp offset, which
        # shrinks at RELOC_RATE (dt capped at LOST_S, so a gap does not give a step). pc: this fix as the last output saw it.
        rp0, out = self.ramp, pos
        pc = pos if rp0 is None else pos + rp0
        if self.t is not None and not switched and tref - self.t < self.LOST_S and \
                np.linalg.norm(pc - self.pos) > 1.0 + 0.05 * h:
            return self._fail(tref, "jump")
        if rp0 is not None:
            m = float(np.linalg.norm(rp0))
            m2 = m - self.RELOC_RATE * min(max(tref - self.t_ramp, 0.0), self.LOST_S)
            self.ramp, self.t_ramp = (rp0 * (m2 / m) if m2 > 0 else None), tref
            out = pos if self.ramp is None else pos + self.ramp

        # Honest covariance (E4, E10, G3, H13): per-axis terms common to both axes, plus the fit's own
        # scatter and its yaw/scale lever on the centroid of the points.
        if self.t is not None and not switched and tref - self.t < 0.25:
            d = pc - self.pos     # fix jitter: the per-frame attitude error the camera really sees
            self.sig2 = 0.98 * self.sig2 + 0.02 * float(d @ d) / (4 * h * h)
        sig2 = max(self.SIG_TH ** 2, self.sig2)
        ch = kf["chain"]
        dpsi = min(abs(yaw - ch["yaw"]), math.pi)
        iso = (sig2 * ch["h"] ** 2 + ch["carried"] + kf["acc"]            # anchor attitude error, hand-overs
               + sig2 * h * h                                              # this frame's attitude error
               + (self.TIME_ERR * (ch["h"] * ch["w"])) ** 2 + (self.TIME_ERR * h * self.rate) ** 2   # timing x rate
               + 2 * (self.SIG_BIAS * h) ** 2 * (1 - math.exp(-(tref - ch["t"]) / self.TAU_BIAS))
               + 0.5 * (2 * h * self.BETA_UNC * math.sin(dpsi / 2)) ** 2   # mount error x yaw change
               + 0.5 * (self.HGT_ERR / h * np.linalg.norm(pos - kf["pos"])) ** 2)   # baro offset x move
        if self.ramp is not None:     # KF_MAP: the output is off the map's frame by the ramp
            iso = iso + float(self.ramp @ self.ramp)
        loc = (self.N_EFF * (r["res_cov"] / n + np.trace(r["res_cov"]) / 2 * r["lever"] * np.eye(2)) if r["dvar"] is None
               else r["dvar"] * np.eye(2))           # whole-picture path: one shift, its own variance
        spread = math.sqrt(np.trace(r["res_cov"]))
        state = "ok" if spread < 0.02 * h + 0.2 and n >= 2 * self.MIN_PTS and r["frac"] >= 0.6 else "degraded"
        cov = iso * np.eye(2) + loc
        if state == "degraded":
            cov = cov * self.DEGRADED_K
        recovered, self.lost = self.lost, False
        self.pos, self.t, self.t_fix, self.var_axis = out.copy(), tref, tref, float(np.trace(cov)) / 2
        kf["n_first"] = kf["n_first"] or n
        est = Est(tref, out.copy(), float(np.trace(cov)), n, state, recovered, None, cov, r["phi"],
                  r["s"] * r["h"] / kf["h"])

        # Keyframe rule (E6, KfRuleHold): change when the points run low, or (at most once per KF_GAP,
        # while turning slowly) when they fall under half of what this keyframe first gave. Try an
        # older keyframe first; a new one inherits this fix's error plus the fit's scatter.
        # n < 0.8 n_first too: a keyframe that starts with few points (prairie at 100 m: 28) is not replaced on
        # every frame; that churn cost 2 deep tracks and a new keyframe per frame and doubled the frame time.
        # A keyframe asked by request_keyframe (the pilot's switch) is new, not an older one, and becomes home.
        if self.kf_wanted and self.rate < self.KF_RATE and self._bright_settled(tref):
            # KF_MAP: during a ramp the new home takes the sent position (out), so the hold point is where the pilot
            # switched; the older keyframes (the map's frame) are then dropped. Without a ramp out is pos.
            k = self._make_kf(img, t0, att, hgt, tref, out, kf["acc"] + np.trace(loc) / 2, ch, False)
            if k is not None:
                old_home = self.home
                self._activate(k, tref)
                if old_home is not None and old_home is not k and self.N_KF > 0:
                    self.kfs = (self.kfs + [old_home])[-self.N_KF:]      # the old home stays for revisits
                self.home, self.kf_wanted = k, False
                if self.KF_MAP:       # the pilot's switch sets the hold point: the old map is not the reference now
                    if self.ramp is not None:
                        self.ramp, self.kfs = None, []
                    self.map, self.map_home, self.in_map, self.reloc_run = [], None, True, []
        elif (n < self.MIN_PTS + 10 and n < self.KF_LOW * kf["n_first"]) or (n < 0.5 * kf["n_first"] and tref - self.t_switch > self.KF_GAP
                                     and self.rate < self.KF_RATE and self._bright_settled(tref)) \
                or (kf["dense"] and abs(yaw - kf["yaw"]) > self.DENSE_YAW):
            # shortcut: an older keyframe is not warped by the yaw change before LK, so revisits fail after
            # about 15 deg of yaw; warp its picture if the hold must survive yaw turns (E6).
            for k in [k for k in [self.home] + self.kfs[::-1] if k is not None and k is not kf][:2]:
                rr = self._track(k, img, t0, att, hgt, tref, deep=True)
                if isinstance(rr, dict) and rr["n"] >= max(2 * self.MIN_PTS, n):
                    self._activate(k, tref)
                    k["ref"] = dict(t=tref, px=rr["p2"], pred=rr["pred"])
                    break
            else:
                k = self._make_kf(img, t0, att, hgt, tref, pos, kf["acc"] + np.trace(loc) / 2, ch, False)
                if k is not None:
                    self._activate(k, tref)
        if self.KF_MAP and self.map and not self.in_map and state == "ok" and tref - self.t_reloc >= self.RELOC_EVERY:
            self._reloc(img, t0, att, hgt, tref, h, out)
        return est
