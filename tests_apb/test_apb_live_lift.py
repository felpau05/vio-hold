"""apb_live.run end to end with fake camera, IMU and barometer: B3 on the live page.
Synthetic frames (test_apb_lift.render, the event lens) at 15 fps: 3 s still at 1.0 m (gyro bias), "Zero here" through
the web page, a 2 s lift to 1.5 m, 1.5 s still. The page must show a height change of 0.5 +- 0.05 m from the camera,
while the fake barometer wanders 0.4 m (as indoors at the event). No hardware needed."""
import json
import math
import os
import sys
import tempfile
import threading
import time
import urllib.request

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.argv = [sys.argv[0]]
import test_apb_lift as T    # noqa: E402  (frame renderer and lens)
import apb_live              # noqa: E402
import apb_sensors as A      # noqa: E402
import settings as S         # noqa: E402

PORT, FPS = int(os.environ.get("TEST_PORT", "18089")), 15.0   # TEST_PORT: two runs at once (two sessions)
FAILS = []


def check(ok, msg):
    print(("PASS " if ok else "FAIL ") + msg)
    if not ok:
        FAILS.append(msg)


still0, still1 = int(3.0 * FPS), int(1.5 * FPS)
frames = [T.render(1.0)[0] for _ in range(still0)]
frames += [img for img, _ in T.lift(1.0, 1.5, still_s=0.0, lift_s=2.0, fps=FPS, yaw_deg=5.0, shift=0.05, tilt=0.0)]
frames += [T.render(1.5, x=0.05, y=-0.025, yaw=5.0)[0] for _ in range(still1)]
T0 = time.monotonic()


class FakeGrabber:
    def __init__(self, cam, fps, rec):
        self.k = 0

    def get(self, timeout=0.5):
        if self.k >= len(frames):
            time.sleep(0.05)
            return None
        t_due = T0 + self.k / FPS
        time.sleep(max(0.0, t_due - time.monotonic()))
        self.k += 1
        return frames[self.k - 1], time.monotonic(), self.k

    def close(self):
        pass


class FakeStream:
    """Still IMU at 200 Hz: gyro 0, accel -g on z (body FRD, z down). rows and count as apb_sensors.ImuStream."""
    def __init__(self, row):
        self.row, self.count, self.rows, self.t_last = row, 0, [], None

    def since(self, t):
        now = time.monotonic()
        t_from = max(t, T0 - 1.0) if self.t_last is None else min(max(t, T0 - 1.0), self.t_last)
        ts = np.arange(round((t_from - (T0 - 1.0)) / 0.005) * 0.005 + (T0 - 1.0) + 0.005, now, 0.005)
        new = [(float(x), *self.row) for x in ts if self.t_last is None or x > self.t_last + 1e-9]
        if new:
            self.count += len(new)
            self.rows, self.t_last = self.rows[-500:] + new, new[-1][0]
        return [(float(x), *self.row) for x in ts if x > t]

    def close(self):
        pass


class FakeBaro:
    def __init__(self):
        self.count, self.rejected = 0, 0

    def since(self, t):
        now = time.monotonic()
        ts = np.arange(max(t, T0 - 1.0) + 0.125, now, 0.125)
        self.count += len(ts)
        return [(float(x), 101325.0 + 4.8 * math.sin(0.7 * (x - T0))) for x in ts]   # about +-0.4 m of wander

    def close(self):
        pass


class FakeCamera:
    def close(self):
        pass


apb_live.Grabber, apb_live.Camera = FakeGrabber, FakeCamera
apb_live.A.open_imu = lambda hz=200: (FakeStream((0.0, 0.0, 0.0)), FakeStream((0.0, 0.0, -9.80665)))
apb_live.A.Baro = FakeBaro

states, jpgs = [], []


def poll():
    zeroed = False
    while time.monotonic() - T0 < len(frames) / FPS + 1.0:
        try:
            st = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{PORT}/state", timeout=1).read())
            states.append((time.monotonic() - T0, st))
            if len(states) % 5 == 0:
                tj = time.perf_counter()
                jpgs.append((time.monotonic() - T0, urllib.request.urlopen(f"http://127.0.0.1:{PORT}/frame.jpg", timeout=1).read(),
                             time.perf_counter() - tj))
            if not zeroed and time.monotonic() - T0 > (still0 - 9) / FPS:
                urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{PORT}/zero", method="POST"), timeout=1)
                zeroed = True
        except OSError:
            pass
        time.sleep(0.1)


th = threading.Thread(target=poll, daemon=True)
th.start()
with tempfile.TemporaryDirectory() as d:
    apb_live.run(S.load(), port=PORT, fps=FPS, seconds=len(frames) / FPS + 0.5, log_dir=d, say=lambda *a: None,
                 fixed_height=1.0)
th.join(timeout=3)

for t, st in states[::5]:
    print(f"  t {t:5.1f}  dh {st.get('dh')}  src {st.get('dh_src')}  why {st.get('why')}")
end = [st for t, st in states if t > (len(frames) - still1 + 5) / FPS]
dh = [st.get("dh") for st in end if st.get("dh") is not None]
src = {st.get("dh_src", "")[:6] for st in end}
check(len(dh) > 3 and all(abs(x - 0.5) < 0.05 for x in dh),
      f"page height change after the 0.5 m lift: {[round(x, 3) for x in dh[-5:]]} (want 0.50 +- 0.05)")
check(src == {"camera"}, f"source shown on the page: {sorted(src)}")
base = [st.get("dh") for t, st in states if (still0 - 6) / FPS < t < still0 / FPS and st.get("dh") is not None]
check(bool(base) and all(abs(x) < 0.05 for x in base), f"just after Zero here: {[round(x, 3) for x in base]} (want 0 +- 0.05)")
# an error in the camera height code must not stop the live loop (B1/B2 go on)
import apb_lift   # noqa: E402


def boom(self, img, R=None):
    raise RuntimeError("test error")


apb_lift.LiftScale.update = boom
T0 = time.monotonic()
states.clear()
th = threading.Thread(target=poll, daemon=True)
th.start()
with tempfile.TemporaryDirectory() as d:
    apb_live.run(S.load(), port=PORT + 1 if False else PORT, fps=FPS, seconds=4.0, log_dir=d, say=lambda *a: None,
                 fixed_height=1.0)
th.join(timeout=3)
late = [st for t, st in states if t > 3.0]
check(bool(late) and all("camera height off: RuntimeError: test error" in (st.get("dh_src") or "") for st in late)
      and late[-1].get("up", 0) > 3.5, f"errors in apb_lift: after 20 it turns off, and the loop runs on: "
      f"{(late[-1].get('dh_src'), round(late[-1].get('up', 0), 1)) if late else None}")
# B1 with --pos floor: Zero here, then a 1.0 m walk at 0.4 m/s with hand tilt; the page shows the floor-track move
import apb_lift as LF2   # noqa: E402
import importlib         # noqa: E402
importlib.reload(LF2)    # the real update again (the error test above replaced it)
walk_frames, true = T.walk([(0, 0), (1.0, 0)], fps=FPS, still_s=0.0, tilt=0.0)   # the fake IMU is still: no tilt
frames = [T.render(1.0)[0] for _ in range(still0)] + [img for img, _ in walk_frames] + \
         [T.render(1.0, x=1.0)[0] for _ in range(still1)]
T0 = time.monotonic()
states.clear()
th = threading.Thread(target=poll, daemon=True)
th.start()
with tempfile.TemporaryDirectory() as d:
    apb_live.run(S.load(), port=PORT, fps=FPS, seconds=len(frames) / FPS + 0.5, log_dir=d, say=lambda *a: None,
                 fixed_height=1.0, pos_src="floor")
th.join(timeout=3)
end = [st for t, st in states if t > (len(frames) - still1 + 5) / FPS]
dist = [st.get("dist") for st in end if st.get("dist") is not None]
check(len(dist) > 3 and all(abs(x - 1.0) < 0.08 for x in dist),
      f"--pos floor, 1.0 m walk: page distance {[round(x, 3) for x in dist[-3:]]} (want 1.00 +- 0.08; B1 allows 0.25); "
      f"info: {end[-1].get('pos_info') if end else None}")
import cv2   # noqa: E402
dec = [(tj, cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_GRAYSCALE), dt) for tj, b, dt in jpgs]
shapes = {d.shape for _, d, _ in dec if d is not None}
moved = len(dec) > 2 and float(np.abs(dec[0][1].astype(int) - dec[-1][1].astype(int)).mean()) > 5
check(len(dec) > 3 and shapes == {(270, 480)} and moved and max(dt for *_, dt in dec) < 0.2,
      f"/frame.jpg: {len(dec)} pictures, shapes {shapes}, the picture changes over the walk: {moved}, "
      f"slowest request {max(dt for *_, dt in dec) * 1e3 if dec else 0:.0f} ms, {np.mean([len(b) for _, b, _ in jpgs]) / 1e3 if jpgs else 0:.0f} kB each")
same = [abs(st["fwd_body"] - st["fwd"]) < 1e-6 and abs(st["right_body"] - st["right"]) < 1e-6
        for st in end if st.get("fwd_body") is not None and st.get("fwd") is not None]
check(len(same) > 3 and all(same), "fwd_body/right_body (body axes now) equal fwd/right when the heading does not change")
late = [st for t, st in states if t > 4.0]
ok_a = [st for st in late if st.get("gyro_hz") and abs(st["gyro_hz"] - 200) < 20 and abs(st["acc_hz"] - 200) < 20
        and st.get("imu_age_ms") is not None and st["imu_age_ms"] < 50 and abs(st["roll_deg"]) < 1 and abs(st["pitch_deg"]) < 1]
check(len(late) > 5 and len(ok_a) / len(late) > 0.9, f"area A line: IMU rates, age and attitude on the page ({len(ok_a)} of "
      f"{len(late)}); last: gyro {late[-1].get('gyro_hz') if late else None} Hz, accel {late[-1].get('acc_hz') if late else None} Hz, "
      f"age {late[-1].get('imu_age_ms') if late else None} ms")
walking = [st for t, st in states if (still0 + 5) / FPS < t < (len(frames) - still1) / FPS]
rdy = [bool(st.get("ready")) and bool(st.get("floor_ok")) for st in walking]
check(len(rdy) > 5 and sum(rdy) / len(rdy) > 0.9, f"--pos floor: READY follows the floor track while walking ({sum(rdy)} of {len(rdy)})")
if end:
    print(f"info: floor F {end[-1].get('fwd'):+.3f} R {end[-1].get('right'):+.3f}")
# one error in the floor track during a walk: a gap that keeps the last values, never a restart at 0
real_update, calls = LF2.LiftScale.update, [0]


def once(self, img, R=None):
    calls[0] += 1
    if calls[0] == 40:
        raise ValueError("test error once")
    return real_update(self, img, R)


LF2.LiftScale.update = once
frames = [T.render(1.0)[0] for _ in range(still0)] + [img for img, _ in walk_frames] + \
         [T.render(1.0, x=1.0)[0] for _ in range(still1)]
T0 = time.monotonic()
states.clear()
th = threading.Thread(target=poll, daemon=True)
th.start()
with tempfile.TemporaryDirectory() as d:
    apb_live.run(S.load(), port=PORT, fps=FPS, seconds=len(frames) / FPS + 0.5, log_dir=d, say=lambda *a: None,
                 fixed_height=1.0, pos_src="floor")
th.join(timeout=3)
LF2.LiftScale.update = real_update
dists = [(t, st.get("dist")) for t, st in states if st.get("dist") is not None]
reset = any(dists[i][1] > 0.3 and dists[j][1] < 0.05 for i in range(len(dists)) for j in range(i + 1, len(dists)))
end_d = [d for t, d in dists if t > (len(frames) - still1 + 5) / FPS]
check(calls[0] > 40 and not reset and len(end_d) > 2 and 0.9 < end_d[-1] < 1.05,
      f"one floor-track error mid-walk: no restart at 0 ({not reset}), end distance "
      f"{round(end_d[-1], 3) if end_d else None} (want 0.9-1.05; the move during the gap is lost)")

# floor mode, Hold skipped: a walk with motion blur (detail under COVER_FINE, as on the board) stays READY and reads the
# move; then a 3 s hand cover (bright, blurred: passes judge) is NOT READY within 1 s and to its end (E1), READY after.
import cv2   # noqa: E402,F811


def mblur(im, n=60):
    k = np.full((1, n), 1.0 / n, np.float32)
    return cv2.filter2D(im, -1, k)


rng_c = np.random.default_rng(9)
hand = lambda: np.clip(cv2.GaussianBlur(T.render(1.0, x=1.0)[0].astype(np.float32), (0, 0), 25) * 0.15 + 170
                       + rng_c.normal(0, 2.0, T.SHAPE), 0, 255).astype(np.uint8)   # noqa: E731
n_cov = int(3.0 * FPS)
far_a, far_b = T.render(1.0, x=-2.0, y=1.5)[0], T.render(1.0, x=2.2, y=-1.8)[0]
far_k = [0]


def far():
    # sharp pictures of two other places in turn: no fit on any frame (a still far picture would track again)
    far_k[0] += 1
    return far_a if far_k[0] % 2 else far_b

n_short, n_long = 6, 15                          # 0.4 s (inside GAP_GRACE_S 0.5) and 1.0 s (longer)
hold_s = lambda s: [T.render(1.0, x=1.0)[0] for _ in range(int(s * FPS))]   # noqa: E731
frames = [T.render(1.0)[0] for _ in range(still0)] + [mblur(img) for img, _ in walk_frames] + hold_s(1.0) + \
         [far() for _ in range(n_short)] + hold_s(1.0) + [far() for _ in range(n_long)] + hold_s(1.0) + \
         [hand() for _ in range(n_cov)] + hold_s(1.5)
t_walk0, t_walk1 = still0 / FPS, (still0 + len(walk_frames)) / FPS
t_sh0 = t_walk1 + 1.0
t_sh1 = t_sh0 + n_short / FPS
t_lg0 = t_sh1 + 1.0
t_lg1 = t_lg0 + n_long / FPS
t_cov0 = t_lg1 + 1.0
t_cov1 = t_cov0 + n_cov / FPS
T0 = time.monotonic()
states.clear()
th = threading.Thread(target=poll, daemon=True)
th.start()
with tempfile.TemporaryDirectory() as d:
    apb_live.run(S.load(), port=PORT, fps=FPS, seconds=len(frames) / FPS + 0.5, log_dir=d, say=lambda *a: None,
                 fixed_height=1.0, pos_src="floor")
th.join(timeout=3)
walk_st = [st for t, st in states if t_walk0 + 0.4 < t < t_walk1]
det_low = sum("lens covered or blurred" in (st.get("why") or "") or (st.get("ready") and True) for st in walk_st)
rdy_w = [bool(st.get("ready")) for st in walk_st]
before_cov = [st.get("dist") for t, st in states if t_walk1 + 0.3 < t < t_cov0 and st.get("dist") is not None]
check(len(rdy_w) > 5 and sum(rdy_w) / len(rdy_w) > 0.9 and before_cov and abs(before_cov[-1] - 1.0) < 0.08,
      f"floor mode, blurred walk (detail under {apb_live.COVER_FINE}): READY {sum(rdy_w)} of {len(rdy_w)}, "
      f"distance {round(before_cov[-1], 3) if before_cov else None} (want 1.00 +- 0.08)")
sh = [st for t, st in states if t_sh0 + 0.12 < t < t_sh1]
check(len(sh) >= 2 and all(st.get("ready") for st in sh) and any(st.get("floor_grace") for st in sh)
      and all(abs((st.get("dist") or 0) - before_cov[-1]) < 0.05 for st in sh),
      f"short gap 0.4 s with a sharp picture: READY kept with the last value ({sum(bool(st.get('ready')) for st in sh)} of {len(sh)}),"
      f" why: {sh[-1].get('why') if sh else None}")
lg = [st for t, st in states if t_lg0 + 0.75 < t < t_lg1]
check(len(lg) >= 2 and not any(st.get("ready") for st in lg),
      f"long gap 1.0 s: NOT READY after the 0.5 s grace ({len(lg)} samples), why: {lg[-1].get('why') if lg else None}")
cov_st = [(t, st) for t, st in states if t_cov0 + 0.25 < t < t_cov1]
check(len(cov_st) > 5 and not any(st.get("ready") for _, st in cov_st) and all(st.get("floor_ok") is False for _, st in cov_st)
      and not any(st.get("floor_grace") for _, st in cov_st),
      f"floor mode, 3 s hand cover: NOT READY from 0.25 s after its start to its end, no grace ({len(cov_st)} samples), "
      f"why: {cov_st[-1][1].get('why') if cov_st else None}")
after = [bool(st.get("ready")) for t, st in states if t_cov1 + 0.5 < t < t_cov1 + 1.4]   # frames end at +1.5 s
check(len(after) > 2 and all(after), f"floor mode: READY again after the cover ({sum(after)} of {len(after)})")
check(all(st.get("state") is None for _, st in states) and "Hold off" in (states[-1][1].get("pos_info") or ""),
      f"floor mode: Hold skipped (no Hold state), info says: {(states[-1][1].get('pos_info') or '')[:60]}")

# zero with the lens low (0.3 m), then a lift to 1.0 m: h_now / h_zero 3.3 is outside LIFT_RATIO_OK -> NOT READY at once
lowlift = [T.render(0.3 + 0.7 * (0.5 - 0.5 * math.cos(math.pi * k / 15)))[0] for k in range(1, 16)]
frames = [T.render(0.3)[0] for _ in range(still0)] + lowlift + [T.render(1.0)[0] for _ in range(int(2.0 * FPS))]
T0 = time.monotonic()
states.clear()
th = threading.Thread(target=poll, daemon=True)
th.start()
with tempfile.TemporaryDirectory() as d:
    apb_live.run(S.load(), port=PORT, fps=FPS, seconds=len(frames) / FPS + 0.3, log_dir=d, say=lambda *a: None,
                 fixed_height=1.0, pos_src="floor")
th.join(timeout=3)
hi = [st for t, st in states if (still0 + 15) / FPS + 0.3 < t < len(frames) / FPS]
check(len(hi) > 5 and not any(st.get("ready") for st in hi) and "push Zero here" in (hi[-1].get("why") or ""),
      f"zero at 0.3 m, lift to 1.0 m: NOT READY ({len(hi)} samples), why: {hi[-1].get('why') if hi else None}")

# camera watchdog (E2): the pictures stop -> run() ends with an error after CAM_STALL_S (1 s here), so the boot unit
# restarts the program. The page says why before the stop.
frames = [T.render(1.0)[0] for _ in range(int(2.0 * FPS))]
T0 = time.monotonic()
states.clear()
apb_live.CAM_STALL_S = 1.0
th = threading.Thread(target=poll, daemon=True)
th.start()
err, t_end = None, None
with tempfile.TemporaryDirectory() as d:
    try:
        apb_live.run(S.load(), port=PORT, fps=FPS, seconds=20.0, log_dir=d, say=lambda *a: None, fixed_height=1.0,
                     pos_src="floor")
    except RuntimeError as ex:
        err = str(ex)
    t_end = time.monotonic() - T0
th.join(timeout=3)
said = [st.get("why") or "" for _, st in states]
check(err is not None and "no picture from the camera for" in err and 2.0 + 0.8 < t_end < 2.0 + 3.0,
      f"camera watchdog: run() stopped with {err!r} at {t_end:.1f} s (pictures stopped at 2.0 s, limit 1 s)")
apb_live.CAM_STALL_S = 5.0
fps = [st.get("fps") for t, st in states if st.get("fps")]
print(f"info: page fps {fps[-1] if fps else None}")
print(("ALL PASS" if not FAILS else f"{len(FAILS)} FAIL"))
sys.exit(1 if FAILS else 0)
