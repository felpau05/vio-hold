#!/usr/bin/env python3
"""Justice D lens (digital, 1280x720, 16:9): the three assumed profiles, load_lens at 640x360,
hold.same_aspect, calibration with no target (fx, fy, k1 from a hand-turn recording, from each
assumed start profile, through apb_main calib --rec, into settings lens_measured) and tools/lens_calib.py
on checkerboard views. One refusal for each tool: too little rotation, too little coverage. A refusal
writes no lens. G: a true lens outside the search range is refused (D3). C counts the residuals over all
matches (D4: the search uses a subsample). H: a lens measured with a target is not fitted again.
T6: two more hidden true lenses, a lens of another projection model (r = 2 f sin(theta / 2)), and a
scene at 60 m with the camera moving during the turn. A T6 case that misses its limit prints
KNOWN LIMIT with its numbers: that is not a PASS.

Independence: the test pictures come from this file's own numpy lens equations (equidistant fisheye,
r = f theta (1 + k1 theta^2 + k2 theta^4 + ...), Newton inverse; equisolid r = 2 f sin(theta / 2)) and its
own Euler rotations. They do not use cv2.fisheye.projectPoints, hold.load_lens, Hold._project,
calib.Rec._rays or px4sim/camera.py.
Synthetic pictures only: this shows that the method works, not that the real camera is known.
Run: ~/.venvs/vio/bin/python tests_apb/test_lens_d.py      (about 4 minutes)
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "tools"))
import apb_main            # noqa: E402
import calib               # noqa: E402
import hold                # noqa: E402
import settings as S       # noqa: E402

LENS = os.path.join(PROJ, "justice_lens_profiles.json")
# (H, V) field of view in deg from the analysis in the task: equidistant lens, 1920x1080 sensor.
ASSUMED = {"Justice D full sensor (assumed)": (152.0, 85.5), "Justice D crop (assumed)": (114.0, 64.1),
           "Justice D anamorphic (assumed)": (114.0, 85.5)}
W, H, FPS, T = 640, 360, 60.0, 10.0
# The true lens at 640x360 differs from all three profiles: fx != fy, k1 and k2 not zero, centre moved.
TRUE = dict(fx=300.0, fy=289.0, cx=324.0, cy=176.0, k=(-0.03, -0.004, 0.0, 0.0))
rng = np.random.default_rng(7)
fails = known = 0
CALLS = []                    # matches in each calib.Rec.residual call (D4)
_residual = calib.Rec.residual


def _counted(self, *a, **k):
    CALLS.append(len(self.ia))
    return _residual(self, *a, **k)


calib.Rec.residual = _counted


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


def check_t6(ok, text):
    """T6 cases: a miss is a KNOWN LIMIT with its numbers. It is not a PASS and not a FAIL of the code."""
    global known
    known += not ok
    print(f"  {'PASS' if ok else 'KNOWN LIMIT'} {text}", flush=True)


# ---------------------------------------------------------------- own geometry (not shared with the code)
def rot(axis, a):
    """Right-hand rotation about body axis 0, 1 or 2 by angle a (rad); a may be an array."""
    a = np.asarray(a, float)
    c, s = np.cos(a), np.sin(a)
    R = np.zeros(a.shape + (3, 3))
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    R[..., axis, axis] = 1.0
    R[..., i, i], R[..., i, j], R[..., j, i], R[..., j, j] = c, -s, s, c
    return R


def euler(roll, pitch, yaw):
    """Body FRD -> NED for ZYX angles: yaw about down, then pitch, then roll."""
    return rot(2, yaw) @ rot(1, pitch) @ rot(0, roll)


def theta_d(th, k):
    t2 = th * th
    return th * (1 + k[0] * t2 + k[1] * t2 ** 2 + k[2] * t2 ** 3 + k[3] * t2 ** 4)


def unproject(u, v, L):
    """Unit camera rays (x right, y down, z optical axis) of pixels u, v: invert theta_d by Newton steps.
    L["model"] "equisolid": r = 2 f sin(theta / 2) instead (no k terms)."""
    x, y = (u - L["cx"]) / L["fx"], (v - L["cy"]) / L["fy"]
    td = np.hypot(x, y)
    if L.get("model") == "equisolid":
        th = 2 * np.arcsin(np.clip(td / 2, 0, 1))
    else:
        th, k = td.copy(), L["k"]
        for _ in range(30):
            t2 = th * th
            th = th - (theta_d(th, k) - td) / (1 + 3 * k[0] * t2 + 5 * k[1] * t2 ** 2 + 7 * k[2] * t2 ** 3 + 9 * k[3] * t2 ** 4)
    s = np.sin(th) / np.maximum(td, 1e-12)
    return np.stack([x * s, y * s, np.cos(th)], -1)


def project(P, L):
    """Pixels of camera-frame points P (..., 3)."""
    rho = np.hypot(P[..., 0], P[..., 1])
    r = theta_d(np.arctan2(rho, P[..., 2]), L["k"]) / np.maximum(rho, 1e-12)
    return np.stack([L["fx"] * r * P[..., 0] + L["cx"], L["fy"] * r * P[..., 1] + L["cy"]], -1)


def start_K(p):
    """K and D of profile p for a 640x360 picture of its 1280x720 picture (pixel centres at integers)."""
    sx, sy = W / p["image_size"][0], H / p["image_size"][1]
    K = np.array([[p["fx"] * sx, 0, (p["cx"] + 0.5) * sx - 0.5], [0, p["fy"] * sy, (p["cy"] + 0.5) * sy - 0.5], [0, 0, 1.]])
    return K, np.array([p["k1"], p["k2"], p["k3"], p["k4"]], float)


# ---------------------------------------------------------------- A, B: profiles and loader
def profiles_part():
    print("A. assumed Justice D profiles (angles from fx, fy with r = f theta)")
    models = json.load(open(LENS))["models"]
    by = {m["name"]: m for m in models}
    for name, (wh, wv) in ASSUMED.items():
        p = by.get(name)
        if p is None:
            check(False, f"profile '{name}' is missing. Add it to justice_lens_profiles.json.")
            continue
        pw, ph = p["image_size"]
        hd, vd = math.degrees(pw / p["fx"]), math.degrees(ph / p["fy"])
        ok = p["projection"] == "equidistant_fisheye" and [pw, ph] == [1280, 720] and abs(hd - wh) < 0.3 and abs(vd - wv) < 0.3
        check(ok, f"'{name}': {pw}x{ph}, {hd:.1f} x {vd:.1f} deg (analysis {wh} x {wv})")
        K, D = hold.load_lens(LENS, name, W, H)
        Kw, Dw = start_K(p)
        check(np.allclose(K, Kw, atol=1e-9) and np.array_equal(D, Dw),
              f"load_lens 640x360: fx {K[0, 0]:.2f} fy {K[1, 1]:.2f} cx {K[0, 2]:.2f} cy {K[1, 2]:.2f} = half of "
              f"{p['fx']:.2f} {p['fy']:.2f} {p['cx']:.1f} {p['cy']:.1f}")
    print("B. same_aspect")
    sa = getattr(hold, "same_aspect", None)
    if sa is None:
        check(False, "hold.same_aspect is missing. Add it.")
        return
    cases = [((1280, 720), 640, 360, True), ((1280, 720), 1280, 726, True), ((1280, 720), 640, 480, False),
             ((960, 720), 360, 288, False), ((960, 720), 640, 480, True), ((1280, 720), 1280, 740, False)]
    got = [sa(ps, w, h) for ps, w, h, _ in cases]
    check(got == [c[3] for c in cases], f"profile vs picture: {[(c[0], f'{c[1]}x{c[2]}', g) for c, g in zip(cases, got)]}")


# ---------------------------------------------------------------- C, D: calibration with no target
def texture():
    """Multi-scale noise on an equirectangular map. Its pole points north, so the nadir view sits on its equator."""
    t = np.zeros((2048, 4096), np.float32)
    for k in range(3, 11):
        t += cv2.resize(rng.standard_normal((2 ** (k - 1), 2 ** k)).astype(np.float32), (4096, 2048),
                        interpolation=cv2.INTER_CUBIC)
    lo, hi = np.percentile(t, [1, 99])
    return cv2.GaussianBlur(np.clip((t - lo) / (hi - lo) * 255, 0, 255), (0, 0), 1.0)


def hand(t, k=1.0):
    return np.radians(k * np.c_[15 * np.sin(2 * np.pi * 0.5 * t), 12 * np.sin(2 * np.pi * 0.4 * t + 1),
                                20 * np.sin(2 * np.pi * 0.3 * t)])


def mount():
    """Camera axes in body FRD: picture top to the right wing, twist 0.8 deg, tilt roll 1.5, pitch -1.0 deg."""
    y, z = np.array([0., -1., 0.]), np.array([0., 0., 1.])
    return rot(1, math.radians(-1.0)) @ rot(0, math.radians(1.5)) @ np.stack([np.cross(y, z), y, z], 1) @ rot(2, math.radians(0.8))


def hand_recording(tex, k=1.0, latency=0.045, readout=0.012, lens=TRUE, dist=None, lever=0.0):
    """Rolling shutter: row r is exposed at t0 + r / H * readout. dist None: rotation only, scene at infinity.
    dist (m): the scene is a sphere of that radius about the turning point, and the camera sits lever (m) from the
    turning point along its optical axis (toward the scene), so it moves in phase with the turn."""
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    rays = unproject(u, v, lens) @ mount().T
    arm = mount() @ np.array([0.0, 0.0, lever])          # camera position in body axes from the turning point
    t0s = np.arange(0, T, 1 / FPS)
    imgs = []
    for t0 in t0s:
        tr = t0 + np.arange(H) / H * readout
        Rr = euler(*hand(tr, k).T)
        n = np.einsum("rij,rwj->rwi", Rr, rays)
        if dist:                                          # where the ray from the camera meets the sphere
            c = Rr @ arm - arm
            cn = np.einsum("rwi,ri->rw", n, c)
            n = (c[:, None, :] + (np.sqrt(cn ** 2 - (c ** 2).sum(-1)[:, None] + dist ** 2) - cn)[..., None] * n) / dist
        lat, lon = np.arcsin(np.clip(n[..., 0], -1, 1)), np.arctan2(n[..., 1], n[..., 2])
        img = cv2.remap(tex, ((lon / (2 * np.pi) + 0.5) * tex.shape[1]).astype(np.float32),
                        ((0.5 - lat / np.pi) * tex.shape[0]).astype(np.float32), cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_WRAP)
        img = cv2.GaussianBlur(img, (3, 3), 0.8) + rng.normal(0, 3, img.shape)
        imgs.append(np.clip(img, 0, 255).astype(np.uint8))
    ta = np.arange(-1, T + 1, 0.01)                      # FC attitude at 100 Hz, 0.05 deg noise
    att = np.c_[ta, hand(ta, k) + rng.normal(0, math.radians(0.05), (len(ta), 3))]
    return dict(imgs=np.array(imgs), t_grab=t0s + latency, att=att, mode=np.array("sim"))


def run_calib(d, rec, name, **kw):
    """apb_main calib --rec with a copy of the lens file; kw: more settings. Returns rc, output lines, settings,
    lens models, seconds."""
    lens = os.path.join(d, "lens.json")
    shutil.copy(LENS, lens)
    p = next(m for m in json.load(open(LENS))["models"] if m["name"] == name)
    K, D = start_K(p)
    path, sp = os.path.join(d, "rec.npz"), os.path.join(d, "settings.json")
    np.savez(path, **rec, K=K, D=D)
    S.save(dict(S.load("/nonexistent"), lens_file=lens, lens_name=name, field="frame", log_dir=d, **kw), sp)
    lines = []
    t = time.time()
    rc = apb_main.mode_calib(S.load(sp), SimpleNamespace(rec=path, seconds=None, settings=sp), say=lines.append)
    return rc, lines, json.load(open(sp)), json.load(open(lens))["models"], time.time() - t


def measured(w, truth):
    """(fx error %, fy error %, k1) of the settings' lens_measured against truth (fx, fy at 640x360), or None."""
    L = w.get("lens_measured")
    if not L:
        return None
    return ((L["fx"] * W / L["image_size"][0] / truth[0] - 1) * 100, (L["fy"] * H / L["image_size"][1] / truth[1] - 1) * 100,
            L["k1"])


def calib_part(tex):
    print(f"C. calibration with no target: true lens at {W}x{H} fx {TRUE['fx']} fy {TRUE['fy']} cx {TRUE['cx']} "
          f"cy {TRUE['cy']} k {TRUE['k']}. Delay 45 ms, readout +12 ms, picture top right.")
    t = time.time()
    rec = hand_recording(tex)
    print(f"  rendered {len(rec['imgs'])} frames in {time.time() - t:.0f} s", flush=True)
    old = json.load(open(LENS))["models"]
    found = []
    for name in ASSUMED:
        if name not in {m["name"] for m in old}:
            check(False, f"start profile '{name}' is missing. Add it first.")
            continue
        CALLS.clear()
        with tempfile.TemporaryDirectory() as d:
            ok, txt, w, dt = calib_case(d, rec, name, (TRUE["fx"], TRUE["fy"]), 1.0)
        check(ok, f"from '{name}': {txt}")
        if ok:
            found.append((w["lens_measured"]["fx"], w["lens_measured"]["fy"]))
        if name == "Justice D full sensor (assumed)":      # D4: the search runs on a subsample of the matches
            n = w["calib_info"].get("n_matches", 0) if w.get("calib_info") else 0
            full, rest = sum(c == n for c in CALLS), [c for c in CALLS if c != n]
            check(n > 0 and full <= 2 and len(rest) > 1000, f"D4: {len(CALLS)} residuals, {full} over all {n} matches, "
                  f"{len(rest)} over at most {max(rest, default=0)} matches. calib took {dt:.1f} s on this laptop")
    if len(found) == 3:
        f = np.array(found)
        spread = (f.max(0) / f.min(0) - 1) * 100
        check(spread.max() < 1, f"the three starts agree: fx spread {spread[0]:.2f} %, fy spread {spread[1]:.2f} %")

    print("D. too little rotation: refused, no lens written")
    rec_c, rec = rec, hand_recording(tex, k=0.2)
    with tempfile.TemporaryDirectory() as d:
        rc, lines, w, models, _ = run_calib(d, rec, "Justice D crop (assumed)")
    ok = (rc == 1 and any(x.startswith("FAIL") for x in lines) and models == old and w["lens_name"] == "Justice D crop (assumed)"
          and w["lens_measured"] is None)
    check(ok, f"rc {rc}, profiles {len(models)} (before {len(old)}), lens_name '{w['lens_name']}', lens_measured "
              f"{w['lens_measured']}. Last line: {lines[-1]}")
    return rec_c


def calib_case(d, rec, name, truth, limit, full=True):
    """mode_calib from start profile name. ok: rc 0, a hand-turn lens_measured with fx and fy within limit % of truth,
    lens file and lens_name unchanged; full: also delay, readout and picture top as recorded. Returns ok, text, settings, s."""
    old = json.load(open(LENS))["models"]
    rc, lines, w, models, dt = run_calib(d, rec, name)
    e = measured(w, truth)
    if rc != 0 or e is None:
        return False, f"rc {rc}, no lens_measured. Output: {' | '.join(lines)}", w, dt
    ex, ey, k1 = e
    ok = (abs(ex) < limit and abs(ey) < limit and w["focal_scale"] == 1.0 and w["lens_measured"]["by"] == "hand turn"
          and w["lens_name"] == name and models == old)
    if full:
        ok = ok and (w["top"] == "right" and not w["mirror"] and abs(w["video_delay_s"] - 0.045) < 0.005
                     and abs(w["readout_s"] - 0.012) < 0.0045)
    for x in lines:
        if x.startswith(("INFO", "FAIL")):
            print("    | " + x)
    return ok, (f"fx error {ex:+.2f} %, fy error {ey:+.2f} % (limit {limit} %), k1 {k1:+.4f}. Delay "
                f"{w['video_delay_s'] * 1e3:.0f} ms, readout {w['readout_s'] * 1e3:+.0f} ms, top {w['top']}. {dt:.0f} s"), w, dt


# ---------------------------------------------------------------- E: calibration with a checkerboard
BOARD = (9, 6)                       # inner corners; square = 1 unit; one white square of margin


def board_poses(n, max_off_deg, dist, seed):
    """Board -> camera (R, t) with the whole board and margin inside the picture."""
    rr = np.random.default_rng(seed)
    edge = np.r_[[[x, -2] for x in np.linspace(-2, 10, 25)], [[x, 7] for x in np.linspace(-2, 10, 25)],
                 [[-2, y] for y in np.linspace(-2, 7, 19)], [[10, y] for y in np.linspace(-2, 7, 19)]]
    edge = np.c_[edge, np.zeros(len(edge))]
    out = []
    while len(out) < n:
        a, phi = math.radians(rr.uniform(0, max_off_deg)), rr.uniform(0, 2 * np.pi)
        dvec = np.array([math.sin(a) * math.cos(phi), math.sin(a) * math.sin(phi), math.cos(a)])
        ax = np.cross([0, 0, 1.], dvec)
        R0 = cv2.Rodrigues(ax / max(np.linalg.norm(ax), 1e-12) * a)[0]
        R = R0 @ rot(2, rr.uniform(-0.5, 0.5)) @ rot(0, rr.uniform(-0.6, 0.6)) @ rot(1, rr.uniform(-0.6, 0.6))
        tvec = dist * rr.uniform(0.85, 1.2) * dvec - R @ np.array([4.0, 2.5, 0.0])
        P = edge @ R.T + tvec
        px = project(P, TRUE)
        if (P[:, 2] > 0).all() and (px[:, 0] > 8).all() and (px[:, 0] < W - 9).all() and (px[:, 1] > 8).all() and (px[:, 1] < H - 9).all():
            out.append((R, tvec))
    return out


def board_images(poses):
    """3 x 3 samples per pixel through the true lens onto the board plane; grey 110 outside the board."""
    u, v = np.meshgrid((np.arange(W * 3) - 1) / 3.0, (np.arange(H * 3) - 1) / 3.0)
    rays = unproject(u, v, TRUE)
    imgs = []
    np.seterr(divide="ignore", invalid="ignore")          # rays parallel to the board give inf: grey
    for R, tvec in poses:
        nrm = R[:, 2]
        s = (tvec @ nrm) / (rays @ nrm)
        b = (s[..., None] * rays - tvec) @ R[:, :2]
        x, y = b[..., 0], b[..., 1]
        sq = (np.floor(x) + np.floor(y)) % 2 == 0
        val = np.where((x >= -1) & (x < 9) & (y >= -1) & (y < 6), np.where(sq, 30.0, 220.0), 220.0)
        val = np.where((x >= -2) & (x < 10) & (y >= -2) & (y < 7) & (s > 0), val, 110.0)
        img = val.reshape(H, 3, W, 3).mean(axis=(1, 3))
        img = cv2.GaussianBlur(img, (0, 0), 0.5) + rng.normal(0, 2, img.shape)
        imgs.append(np.clip(img, 0, 255).astype(np.uint8))
    return np.array(imgs)


def lens_calib_main(lens_calib, argv):
    lines = []
    try:
        rc = lens_calib.main(argv, say=lines.append)
    except SystemExit as e:              # argparse refuses an unknown option
        rc = e.code
    return rc, lines


def target_part(rec_c):
    print("E. tools/lens_calib.py on checkerboard views (true lens as in C)")
    try:
        import lens_calib
    except ImportError as e:
        check(False, f"Python cannot import tools/lens_calib.py ({e}). Add the file.")
        return
    old = json.load(open(LENS))["models"]
    with tempfile.TemporaryDirectory() as d:
        lens = os.path.join(d, "lens.json")
        shutil.copy(LENS, lens)
        path = os.path.join(d, "board.npz")
        np.savez(path, imgs=board_images(board_poses(24, 50, 14.0, 1)))
        lines = []
        rc = lens_calib.main([path, "--name", "Justice D board test", "--lens-file", lens], say=lines.append)
        models = json.load(open(lens))["models"]
        p = next((m for m in models if m["name"] == "Justice D board test"), None)
        for x in lines:
            print("    | " + x)
        if p is None:
            check(False, f"lens_calib gave rc {rc} and wrote no profile. Read its output above.")
        else:
            ex, ey = (p["fx"] / TRUE["fx"] - 1) * 100, (p["fy"] / TRUE["fy"] - 1) * 100
            ok = (rc == 0 and abs(ex) < 0.5 and abs(ey) < 0.5 and p["image_size"] == [W, H] and models[:len(old)] == old
                  and p["projection"] == "equidistant_fisheye")
            check(ok, f"fx error {ex:+.3f} %, fy error {ey:+.3f} %, cx {p['cx']:.2f} (true {TRUE['cx']}), cy {p['cy']:.2f} "
                      f"(true {TRUE['cy']}), k1 {p['k1']:+.4f} k2 {p['k2']:+.4f} (true {TRUE['k'][0]} {TRUE['k'][1]})")

        print("F. checkerboard views near the centre only: refused, no profile written")
        folder = os.path.join(d, "near_centre")
        os.makedirs(folder)
        for i, img in enumerate(board_images(board_poses(14, 6, 30.0, 2))):
            cv2.imwrite(os.path.join(folder, f"v{i:02d}.png"), img)
        lines = []
        before = open(lens).read()
        rc = lens_calib.main([folder, "--name", "Justice D small coverage", "--lens-file", lens], say=lines.append)
        ok = rc == 1 and any(x.startswith("FAIL") and "deg" in x for x in lines) and open(lens).read() == before
        check(ok, f"rc {rc}. Last line: {lines[-1] if lines else 'no output'}")
        target_settings_part(lens_calib, d, path, rec_c)


def target_settings_part(lens_calib, d, path, rec_c):
    print("H. lens_calib.py --settings writes lens_measured (by target). calib then fits one focal scale, not the lens")
    name = "Justice D full sensor (assumed)"
    lens, sp = os.path.join(d, "lens_h.json"), os.path.join(d, "settings_h.json")
    shutil.copy(LENS, lens)
    S.save(dict(S.load("/nonexistent"), camera="digital", lens_file=lens, lens_name=name, field="frame", log_dir=d,
                focal_scale=0.97), sp)
    before = open(lens).read()
    rc, lines = lens_calib_main(lens_calib, [path, "--settings", sp, "--lens-file", lens])
    for x in lines:
        print("    | " + x)
    try:
        L = S.load(sp)["lens_measured"]
    except (ValueError, KeyError) as e:
        L = None
        print(f"    | {e!r}")
    e = measured(dict(lens_measured=L), (TRUE["fx"], TRUE["fy"]))
    ok = (rc == 0 and e is not None and L["by"] == "target" and L["image_size"] == [W, H] and abs(e[0]) < 0.5
          and abs(e[1]) < 0.5 and open(lens).read() == before and S.load(sp)["focal_scale"] == 1.0)
    check(ok, f"rc {rc}, lens_measured {'fx error %+.3f %%, fy error %+.3f %%' % e[:2] if e else None}, by "
              f"{L and L['by']}, lens file unchanged {open(lens).read() == before}")
    if not ok:
        return
    for scale, want_fail in ((1.0, False), (1.05, True)):
        tl = dict(L, fx=L["fx"] * scale, fy=L["fy"] * scale)
        with tempfile.TemporaryDirectory() as d2:
            rc, lines, w, models, dt = run_calib(d2, rec_c, name, camera="digital", lens_measured=tl)
        fs = w["focal_scale"]
        bad = [x for x in lines if x.startswith("FAIL") and "focal scale" in x]
        ok = (rc == 0 and w["lens_measured"] == tl and bool(bad) == want_fail and abs(fs * scale - 1) < 0.01
              and w["lens_name"] == name)
        check(ok, f"target lens x {scale}: rc {rc}, lens_measured unchanged {w['lens_measured'] == tl}, focal scale "
                  f"{fs:.4f} (want {1 / scale:.4f}), FAIL line {bad}. {dt:.0f} s")


# ---------------------------------------------------------------- G: D3, T6: cases the code was not built on
def edge_part(tex):
    print("G. true lens outside the search range (fx = 1.7 x w / 2, narrow): refused, no lens written")
    narrow = dict(fx=544.0, fy=544.0, cx=322.0, cy=178.0, k=(-0.01, 0.0, 0.0, 0.0))
    rec = hand_recording(tex, lens=narrow)
    old = json.load(open(LENS))["models"]
    with tempfile.TemporaryDirectory() as d:
        rc, lines, w, models, dt = run_calib(d, rec, "Justice D full sensor (assumed)")
    for x in lines:
        if x.startswith(("INFO", "FAIL")):
            print("    | " + x)
    edge = [x for x in lines if x.startswith("FAIL") and "search range" in x]
    ok = rc == 1 and bool(edge) and w["lens_measured"] is None and models == old and not w["calibrated"]
    check(ok, f"rc {rc}, edge FAIL line {bool(edge)}, lens_measured {w['lens_measured']}, calibrated {w['calibrated']}. "
              f"Last line: {lines[-1]}")


def near(name, seed):
    """A true lens near start profile 'name' at 640x360, drawn with a fixed seed. The code never sees it: fx and fy
    within 6 % of the profile's, centre within 6 px of the middle, k1 -0.05 to 0, k2 -0.01 to +0.01."""
    p = next(m for m in json.load(open(LENS))["models"] if m["name"] == name)
    rr = np.random.default_rng(seed)
    f = rr.uniform(0.94, 1.06, 2)
    return dict(fx=p["fx"] / 2 * f[0], fy=p["fy"] / 2 * f[1], cx=319.5 + rr.uniform(-6, 6), cy=179.5 + rr.uniform(-6, 6),
                k=(rr.uniform(-0.05, 0.0), rr.uniform(-0.01, 0.01), 0.0, 0.0))


def t6_part(tex):
    start = "Justice D full sensor (assumed)"           # tools/profiles/justice_d.json
    print(f"T6. cases that the code was not built on; calib from '{start}'")
    for name, seed in (("Justice D full sensor (assumed)", 11), ("Justice D anamorphic (assumed)", 12)):
        L = near(name, seed)
        with tempfile.TemporaryDirectory() as d:
            ok, txt, w, dt = calib_case(d, hand_recording(tex, lens=L), start, (L["fx"], L["fy"]), 1.0)
        check_t6(ok, f"hidden lens near '{name}' (fx {L['fx']:.1f} fy {L['fy']:.1f} cx {L['cx']:.1f} cy {L['cy']:.1f} "
                     f"k1 {L['k'][0]:+.4f} k2 {L['k'][1]:+.4f}): {txt}")
    # Equisolid lens with the field of view of TRUE: the same ray angle at the right and the bottom picture edge.
    th = [math.acos(unproject(np.array([u]), np.array([v]), TRUE)[0, 2]) for u, v in ((W - 0.5, TRUE["cy"]), (TRUE["cx"], H - 0.5))]
    eq = dict(model="equisolid", fx=(W - 0.5 - TRUE["cx"]) / (2 * math.sin(th[0] / 2)),
              fy=(H - 0.5 - TRUE["cy"]) / (2 * math.sin(th[1] / 2)), cx=TRUE["cx"], cy=TRUE["cy"])
    # Best-fit equidistant fx, fy: near the picture centre both models are r = f theta, so the equidistant focal
    # length is the true model's slope dr / dtheta at theta = 0, here r(delta) / delta with delta = 1e-4 rad.
    d0 = 1e-4
    best = (eq["fx"] * 2 * math.sin(d0 / 2) / d0, eq["fy"] * 2 * math.sin(d0 / 2) / d0)
    with tempfile.TemporaryDirectory() as d:
        ok, txt, w, dt = calib_case(d, hand_recording(tex, lens=eq), start, best, 2.0, full=False)
    check_t6(ok, f"equisolid r = 2 f sin(theta / 2), f {eq['fx']:.1f} x {eq['fy']:.1f} px, edges at {math.degrees(th[0]):.1f} "
                 f"and {math.degrees(th[1]):.1f} deg as TRUE; against best-fit equidistant fx {best[0]:.2f} fy {best[1]:.2f}: {txt}")
    # Scene at 60 m. The camera sits 0.4 m from the turning point along its optical axis (toward the scene).
    lever, dist = 0.4, 60.0
    ta = np.arange(0, T, 0.01)
    c = euler(*hand(ta).T) @ (mount() @ np.array([0, 0, lever]))
    span = c.max(0) - c.min(0)
    with tempfile.TemporaryDirectory() as d:
        ok, txt, w, dt = calib_case(d, hand_recording(tex, dist=dist, lever=lever), start, (TRUE["fx"], TRUE["fy"]), 1.0,
                                    full=False)
    check_t6(ok, f"true lens of C, scene at {dist:.0f} m, camera moves {span[0]:.2f} m north, {span[1]:.2f} m east, "
                 f"{span[2]:.2f} m down peak to peak, in phase with the turn: {txt}")


def main():
    t0 = time.time()
    profiles_part()
    tex = texture()
    rec_c = calib_part(tex)
    target_part(rec_c)
    del rec_c
    edge_part(tex)
    t6_part(tex)
    print(f"{'ALL PASS' if not fails and not known else f'{fails} FAIL, {known} KNOWN LIMIT (not a pass)'} in "
          f"{time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
