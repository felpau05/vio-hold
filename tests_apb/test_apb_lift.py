"""apb_lift.LiftScale on synthetic pictures: a textured floor seen through the event lens (settings.json), at
960 x 540 as Hold gets it. Cases: a 0.5 m lift with yaw, side moves and tilt jitter; a start height that is not
the assumed 1.0 m; a still hold; a covered lens breaks the chain. No camera needed."""
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import apb_est as E          # noqa: E402
import apb_lift as L         # noqa: E402
import apb_main              # noqa: E402
import settings as S         # noqa: E402

FAILS = []


def check(ok, msg):
    print(("PASS " if ok else "FAIL ") + msg)
    if not ok:
        FAILS.append(msg)


SHAPE = (540, 960)
s = E.bench_settings(S.load())
K, D = apb_main.lens(s, SHAPE)
v, u = np.mgrid[0:SHAPE[0], 0:SHAPE[1]]
RAYS = cv2.fisheye.undistortPoints(np.c_[u.ravel(), v.ravel()].astype(np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)

# floor texture: noise at several scales (like concrete or carpet), 6 m wide, no regular lines. Note: the repo's
# sim_ground.png is a chart with grid lines; on a regular grid the tracker can alias (see the tile-floor case).
rng = np.random.default_rng(1)
tex = np.zeros((3000, 3000), np.float32)
for sigma, amp in ((1.5, 25.0), (5.0, 40.0), (18.0, 60.0), (60.0, 50.0)):
    n = cv2.GaussianBlur(rng.normal(0, 1, tex.shape).astype(np.float32), (0, 0), sigma)
    tex += amp * n / n.std()
tex = cv2.equalizeHist(np.clip(tex + 128, 0, 255).astype(np.uint8))   # full contrast: capture._Judge refuses flat pictures
M_PER_PX = 6.0 / tex.shape[1]


def render(h, x=0.0, y=0.0, yaw=0.0, roll=0.0, pitch=0.0, noise=2.0):
    """Camera looking down from height h at floor position (x, y) m; angles in degrees."""
    r, p, w = (math.radians(a) for a in (roll, pitch, yaw))
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(w), -math.sin(w), 0], [math.sin(w), math.cos(w), 0], [0, 0, 1]])
    d = np.c_[RAYS, np.ones(len(RAYS))] @ (Rz @ Ry @ Rx).T           # z = down to the floor
    gx, gy = x + h * d[:, 0] / d[:, 2], y + h * d[:, 1] / d[:, 2]
    mx = (gx / M_PER_PX + tex.shape[1] / 2).reshape(SHAPE).astype(np.float32)
    my = (gy / M_PER_PX + tex.shape[0] / 2).reshape(SHAPE).astype(np.float32)
    img = cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    return np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8), Rz @ Ry @ Rx


def lift(h0, h1, still_s=1.0, lift_s=2.0, fps=15.0, yaw_deg=10.0, shift=0.1, tilt=3.0):
    """Frames: still at h0, a smooth lift to h1 with yaw and side shift, still at h1. Tilt jitter all along."""
    n0, n1 = int(still_s * fps), int(lift_s * fps)
    out = []
    for k in range(2 * n0 + n1):
        a = 0.0 if k < n0 else min(1.0, (k - n0) / n1)
        a = 0.5 - 0.5 * math.cos(math.pi * a)
        out.append(render(h0 + (h1 - h0) * a, x=shift * a, y=-0.5 * shift * a, yaw=yaw_deg * a,
                          roll=tilt * math.sin(0.9 * k), pitch=tilt * math.cos(0.7 * k)))
    return out


def walk(points, speed=0.4, fps=15.0, h=1.0, tilt=3.0, wobble=0.03, still_s=0.7):
    """Frames along straight legs between floor points (m), at speed m/s, hand tilt jitter and a height wobble.
    Returns (frames, true end position)."""
    pos = [np.array(points[0], float)]
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(np.linalg.norm(b - a) / speed * fps))
        pos += [a + (b - a) * (0.5 - 0.5 * math.cos(math.pi * k / n)) for k in range(1, n + 1)]
    pos = [pos[0]] * int(still_s * fps) + pos + [pos[-1]] * int(still_s * fps)
    out = [render(h + wobble * math.sin(1.3 * k), x=p[0], y=p[1], yaw=2.0 * math.sin(0.2 * k),
                  roll=tilt * math.sin(0.9 * k), pitch=tilt * math.cos(0.7 * k)) for k, p in enumerate(pos)]
    return out, pos[-1] - pos[0]


def odo(frames):
    ls = L.LiftScale(K, D, SHAPE)
    for img, R in frames:
        ls.update(img, R)
    return ls


def run(frames, h_zero=1.0, tilt_fix=True):
    ls = L.LiftScale(K, D, SHAPE)
    t0 = time.perf_counter()
    r = None
    for img, R in frames:
        r = ls.update(img, R if tilt_fix else None)
    ms = (time.perf_counter() - t0) / len(frames) * 1e3
    return (None if r is None else h_zero * (r - 1.0)), ls, ms


def main():
    t_start = time.perf_counter()
    dh, ls, ms = run(lift(1.0, 1.5))
    check(dh is not None and abs(dh - 0.5) < 0.03, f"lift 1.0 -> 1.5 m with 10 deg yaw, 0.1 m shift, 3 deg tilt jitter: "
          f"{dh if dh is None else round(dh, 3)} m (want 0.50 +- 0.03), {ls.n} inliers")
    check(ms < 40, f"{ms:.1f} ms per frame on this laptop (the A53 is about 10x slower; Hold takes about 60 ms there)")

    dh, ls, _ = run(lift(0.8, 1.3))
    check(dh is not None and abs(dh - 0.5) < 0.15, f"start at 0.8 m, h_zero assumed 1.0 m: {dh if dh is None else round(dh, 3)} m "
          "(B3 window 0.35-0.65; exact would be 0.625)")

    dh, ls, _ = run(lift(1.0, 1.5, tilt=0.0, yaw_deg=0.0), tilt_fix=False)
    check(dh is not None and abs(dh - 0.5) < 0.03, f"lift with no tilt, no tilt correction input: "
          f"{dh if dh is None else round(dh, 3)} m (want 0.50 +- 0.03)")

    dh, ls, _ = run(lift(1.0, 1.0, still_s=1.5, lift_s=1.0))
    check(dh is not None and abs(dh) < 0.02, f"still at 1.0 m with tilt jitter: {dh if dh is None else round(dh, 3)} m (want 0 +- 0.02)")

    fr, true = walk([(0, 0), (0, 1.5)], speed=1.0, fps=10.0)
    ls = odo(fr)
    err = None if ls.disp is None else float(np.linalg.norm(ls.disp - true))
    check(err is not None and err < 0.08, f"B1 right 1.5 m at 1.0 m/s and 10 fps: odometer "
          f"{None if ls.disp is None else np.round(ls.disp, 3)} m, error {None if err is None else round(err, 3)} m (want < 0.08)")

    for name, pts, lim in (("B1 forward 1.0 m", [(0, 0), (1.0, 0)], 0.05),
                           ("B1 diagonal 1.5 m", [(0, 0), (1.06, -1.06)], 0.08),
                           ("B2 loop of 3 m back to the start", [(0, 0), (0.75, 0), (0.75, 0.75), (0, 0.75), (0, 0)], 0.10)):
        fr, true = walk(pts)
        ls = odo(fr)
        err = None if ls.disp is None else float(np.linalg.norm(ls.disp - true))
        check(err is not None and err < lim, f"{name}: odometer {None if ls.disp is None else np.round(ls.disp, 3)} m, "
              f"true {np.round(true, 3)} m, error {None if err is None else round(err, 3)} m (want < {lim})")

    # a covered lens: a gap. The values are unknown while covered, then go on from the last values.
    fr, true = walk([(0, 0), (1.0, 0)])
    ls = L.LiftScale(K, D, SHAPE)
    during, moved = [], None
    for k, (img, R) in enumerate(fr):
        if 25 <= k < 30:
            if k == 25:
                moved = ls.disp.copy()
            ls.bad_picture()
            during.append(ls.disp)
        else:
            ls.update(img, R)
    lost = float(np.linalg.norm(true)) - float(ls.disp[0]) if ls.disp is not None else None
    check(all(d is None for d in during) and ls.gaps == 1 and ls.disp is not None and lost is not None and 0 < lost < 0.3,
          f"lens covered 5 frames during a 1.0 m move: unknown while covered, then goes on; reads "
          f"{None if ls.disp is None else round(float(ls.disp[0]), 3)} m (the move while covered is lost), {ls.gaps} gap")
    ls2 = L.LiftScale(K, D, SHAPE)
    for img, R in fr[:15]:
        ls2.update(img, R)
    try:
        for _ in range(5):
            ls2.update(np.full(SHAPE, 128, np.uint8), fr[15][1])
        ok_u = ls2.disp is None and ls2.gaps >= 1
    except Exception as ex:     # noqa: BLE001
        ok_u = False
        print("   ", type(ex).__name__, ex)
    check(ok_u, "uniform frames after tracking (LK refuses every point): a gap, no error (board: 'NoneType' .reshape)")

    ls.zero()
    r0 = ls.update(*fr[-1])
    r = ls.update(*fr[-1])
    check(r0 is None and r is not None and abs(r - 1.0) < 1e-3 and ls.gaps == 0 and np.allclose(ls.disp, 0, atol=1e-3),
          "zero starts again at 0 (no value on the first picture: new points need one good fit)")

    # a dark, noisy cover (a hand or cloth with sensor noise): corners in the noise never track -> no value on any frame
    ls = L.LiftScale(K, D, SHAPE)
    for img, R in fr[:20]:
        ls.update(img, R)
    vals = []
    for k in range(30):
        cover = np.clip(20 + rng.normal(0, 3.0, SHAPE), 0, 255).astype(np.uint8)
        ls.update(cover, fr[19][1])
        vals.append(ls.disp)
    check(all(v is None for v in vals), f"dark noisy cover, 30 frames: no value on any frame "
          f"({sum(v is not None for v in vals)} frames had one)")
    for img, R in fr[20:]:
        ls.update(img, R)
    check(ls.disp is not None, "after the cover the track gives values again")

    print(("ALL PASS" if not FAILS else f"{len(FAILS)} FAIL") + f" in {time.perf_counter() - t_start:.0f} s")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
