#!/usr/bin/env python3
"""Camera and run code faults found by the hardware review (D1-D8), with fake cameras and a fake flight controller:
D1 a failed flight controller connection leaves no camera open; D2 a camera that stops in flight is opened again;
D3 close() while the reader is blocked in read(); D4 the stamp quality line of check mode; D5 a full disk; D6 a wrong
buffer shape; D7 a covered lens gives no fix (picture check and Hold, as run_live); D8 the Hold timing frame stamps.
Run: ~/.venvs/vio/bin/python tests_apb/test_robust.py      (about 40 s; one part: add d1 ... d8)
"""
import contextlib
import io
import math
import os
import re
import shutil
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import apb_main            # noqa: E402
import capture             # noqa: E402
import settings as S       # noqa: E402
import test_digital as T   # noqa: E402  (fake capture, settings, scenes and child commands of the digital tests)
from hold import Series    # noqa: E402

W, H = T.W, T.H
LENS_D = "Justice D full sensor (assumed)"     # a 16:9 profile for the 640x360 picture
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


@contextlib.contextmanager
def printed(out):
    """Collects the lines that the code prints in out."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        yield
    out.extend(x for x in buf.getvalue().splitlines() if x.strip())


@contextlib.contextmanager
def patched(obj, **kw):
    real = {k: getattr(obj, k, patched) for k in kw}       # patched: the name was not there
    for k, v in kw.items():
        setattr(obj, k, v)
    try:
        yield
    finally:
        for k, v in real.items():
            if v is patched:
                delattr(obj, k)
            else:
                setattr(obj, k, v)


def frames(n=30, img=None):
    img = np.zeros((36, 64), np.uint8) if img is None else img
    return [capture.Frame(img, t - 0.04, t, i + 1, None) for i, t in enumerate(T.stamps60(n))]


class StubPipe:
    """Stands in for Pipe and Hold: each frame gives a fix, sent when send is True."""
    note, good, send = None, 0, False

    def __init__(self, s, fc_pos):
        pass

    def step(self, fr, att, hgt, gate=None):
        e = SimpleNamespace(t=fr.t0, pos=np.zeros(2), var=0.01, state="good", n=50)
        return e, self.send, None if self.send else "stub"


def fake_fc(s, sent=None):
    fc = T.fake_fc(s)
    fc.close = lambda: None
    fc.send_fix = lambda e, att, hgt: sent.append(e.t) if sent is not None else None
    return fc


class StopSource(T.ListSource):
    """Gives its frames, then None, as a camera whose command stopped. After kill_s, newest() raises
    KeyboardInterrupt, so that a loop that never ends cannot stop the test."""

    def __init__(self, frs, kill_s, pace_s=0.0):
        super().__init__(frs)
        self.kill, self.pace, self.closed = time.monotonic() + kill_s, pace_s, False

    def newest(self, timeout=0.5):
        if time.monotonic() > self.kill:
            raise KeyboardInterrupt
        if self.frames:
            time.sleep(self.pace)
        return super().newest(timeout)

    def close(self):
        self.closed = True


def started(pidfile, wait_s=5.0):
    """The process id that a child wrote to pidfile, or None if it did not start in wait_s."""
    end = time.monotonic() + wait_s
    while time.monotonic() < end:
        try:
            return int(open(pidfile).read())
        except (OSError, ValueError):
            time.sleep(0.01)
    return None


# ---------------------------------------------------------------- D1
def d1_fc_fails(d):
    print("D1: the flight controller connection fails 3 times; the camera is a video_command child")
    s = T.digital_settings(calibrated=True, log_dir=d, pixel_format="YUYV", video_device="/nonexistent")
    srcs, pids, n_fc, lines = [], [], [0], []

    def open_video(s):
        cmd, pid = T.child_command(d, b"", "sleep")          # a camera that waits for its first frame
        srcs.append(capture.DigitalSource(dict(s, video_command=cmd)))
        pids.append(pid)
        started(pid)
        return srcs[-1]

    def open_fc(s, say=print):
        n_fc[0] += 1
        if n_fc[0] > 3:
            raise KeyboardInterrupt                           # the test stops mode_run here
        raise RuntimeError("no heartbeat from the flight controller (test)")

    with patched(apb_main, open_video=open_video, open_fc=open_fc):
        rc = apb_main.mode_run(s, SimpleNamespace(raw=False), say=lines.append)
    readers = sum(src._t.is_alive() for src in srcs)
    children = sum(not T.gone(p, 2.0) for p in pids)
    check(rc == 0 and len(srcs) == 4 and readers == 0 and children == 0,
          f"{len(srcs)} camera sources opened by mode_run; {readers} reader threads and {children} camera commands "
          "still alive after the failed connections")
    for src in srcs:                                          # clean up after a failure
        if src._t.is_alive():
            src.close()


# ---------------------------------------------------------------- D2
def d2_camera_stops(d):
    print("D2: the camera stops in flight (newest() gives None for ever)")
    s = T.digital_settings(log_dir=d)
    err, t1 = None, time.monotonic()
    with patched(apb_main, Pipe=StubPipe, start_problems=lambda *a: []):
        try:
            apb_main.run_live(s, StopSource(frames(), kill_s=10.0), fake_fc(s), seconds=6.0,
                              log_path=os.path.join(d, "d2.csv"))
        except (RuntimeError, KeyboardInterrupt) as e:
            err = e
    dt = time.monotonic() - t1
    check(isinstance(err, RuntimeError) and 2.0 <= dt < 3.5 and "camera" in str(err),
          f"run_live stops {dt:.1f} s after the last frame with: {err!r}")
    opened, lines = [], []

    def open_video(s):
        opened.append(StopSource(frames(), kill_s=8.0))
        if len(opened) > 1:
            raise KeyboardInterrupt                           # the camera is opened again: the test stops here
        return opened[0]

    with patched(apb_main, Pipe=StubPipe, start_problems=lambda *a: [], open_video=open_video,
                 open_fc=lambda s, say=print: fake_fc(s)):
        apb_main.mode_run(dict(s, calibrated=True), SimpleNamespace(raw=False), say=lines.append)
    fail = [x for x in lines if x.startswith("FAIL")]
    check(len(opened) == 2 and opened[0].closed and len(fail) == 1 and "camera" in fail[0],
          f"mode_run closes the camera and opens it again: {len(opened)} opens, first closed {opened[0].closed}; {fail}")


# ---------------------------------------------------------------- D3
class BlockCap(T.FakeCap):
    """A capture whose read() blocks for block_s (None: until unblock is set), as a V4L2 driver that waits for a
    buffer. release() notes whether the reader was inside read() at that time."""

    def __init__(self, block_s):
        super().__init__("YUYV")
        self.block_s, self.unblock, self.in_read, self.released_in_read = block_s, threading.Event(), False, None

    def read(self):
        self.in_read = True
        self.unblock.wait(self.block_s)
        self.in_read = False
        return False, None

    def release(self):
        self.released, self.released_in_read = True, self.in_read


def wait_for(cond, wait_s=2.0):
    end = time.monotonic() + wait_s
    while not cond() and time.monotonic() < end:
        time.sleep(0.005)
    return cond()


def d3_close_blocked():
    print("D3: close() while the reader thread is blocked in read()")
    for name in ("DigitalSource", "V4L2Source (analog)"):
        cap = BlockCap(2.0)
        if name == "DigitalSource":
            src = capture.DigitalSource(T.digital_settings(), cap=cap)
        else:
            with patched(cv2, VideoCapture=lambda *a: cap):
                src = capture.V4L2Source(dict(S.load("/nonexistent"), video_device="/nonexistent"))
        wait_for(lambda: cap.in_read)
        t1 = time.monotonic()
        src.close()
        dt = time.monotonic() - t1
        check(cap.released and cap.released_in_read is False and not src._t.is_alive(),
              f"{name}, read() blocks 2 s: capture released {cap.released}, with the reader inside read() "
              f"{cap.released_in_read}; close() took {dt:.1f} s")
    wait = getattr(capture, "CLOSE_WAIT_S", None)
    cap, out = BlockCap(None), []
    src = capture.DigitalSource(T.digital_settings(), cap=cap)
    wait_for(lambda: cap.in_read)
    with patched(capture, CLOSE_WAIT_S=0.5), printed(out):   # the test waits 0.5 s, not the real wait
        src.close()
    cap.unblock.set()
    src._t.join(2.0)
    check(wait == 12.0 and not cap.released and len(out) == 1,
          f"a reader that does not stop: capture not released ({not cap.released}), one line printed {out}; "
          f"real wait {wait} s")


# ---------------------------------------------------------------- D4
class TickSource:
    """newest() gives a frame at each 1/fps s tick of the clock, stamped at the tick plus a random error of up to
    jitter_ms; stamp: 'read' or 'driver'. A frame that the caller misses still counts in seq."""

    def __init__(self, fps, jitter_ms, stamp, analog=False):
        self.fps, self.j, self.stamp, self.t_start = fps, jitter_ms, stamp, time.monotonic()
        self.rng, self.seq = np.random.default_rng(3), 0
        self.info = ({"width": 720, "height": 576, "field": "interlaced", "standard": "PAL"} if analog else
                     {"width": W, "height": H, "fourcc": "YUYV"})
        self.img = np.zeros((288, 360) if analog else (360, 640), np.uint8)

    def newest(self, timeout=0.5):
        k = max(self.seq + 1, math.ceil((time.monotonic() - self.t_start) * self.fps))
        t = self.t_start + k / self.fps
        time.sleep(max(0.0, t - time.monotonic()))
        self.seq = k
        stamp = t + self.rng.uniform(-self.j, self.j) / 1e3
        return capture.Frame(self.img, stamp - 0.04, stamp, k, None)

    def close(self):
        pass


def d4_stamp_line(d):
    print("D4: check mode measures the stamp quality of each source")
    cases = (("digital, read stamps, +-2 ms", T.digital_settings(log_dir=d), (60, 2.0, "read"), "PASS", (2.0, 4.5)),
             ("digital, read stamps, +-10 ms", T.digital_settings(log_dir=d), (60, 10.0, "read"), "FAIL", (12.5, 19.0)),
             ("digital, driver stamps", T.digital_settings(log_dir=d), (60, 0.2, "driver"), "PASS", (0.0, 1.0)),
             ("analog, driver stamps", dict(S.load("/nonexistent"), log_dir=d), (25, 0.2, "driver", True), "PASS", (0.0, 1.0)))
    for name, s, src, want, (lo, hi) in cases:
        lines = T.run_check(s, video=lambda s, a=src: TickSource(*a))
        hit = [x for x in lines if "frame stamps" in x]
        val = [float(v) for v in re.findall(r"(\d+\.\d+) ms", hit[0])][:1] if hit else []
        check(len(hit) == 1 and hit[0].startswith(want) and val and lo <= val[0] <= hi
              and not any("no usable frame time" in x for x in lines), f"{name}: {hit}")


# ---------------------------------------------------------------- D5
def join_savers():
    """Waits for the save threads of Raw (not daemon threads)."""
    for th in threading.enumerate():
        if th is not threading.current_thread() and not th.daemon:
            th.join()


def d5_full_disk(d):
    print("D5: a full disk")
    s = T.digital_settings(log_dir=d, send_hz=50)
    sent, out, err = [], [], None
    with patched(apb_main, Pipe=StubPipe, start_problems=lambda *a: []), patched(StubPipe, send=True), printed(out):
        try:
            apb_main.run_live(s, StopSource(frames(80), kill_s=10.0, pace_s=1 / 60), fake_fc(s, sent), seconds=1.0,
                              log_path="/dev/full")            # each write to /dev/full fails: no space left
        except Exception as e:
            err = e
    check(err is None and len(sent) >= 30 and len(out) == 1 and "log" in out[0],
          f"(a) run log on a full disk: {len(sent)} fixes sent in 1 s, error {err!r}; printed {out}")
    out, err = [], None
    with printed(out):
        log = apb_main.Log("/dev/full")
        try:
            for _ in range(5):
                log.row(None, None, False, "no frame", 0.0, None, None, None)
            log.close()
        except OSError as e:
            err = e
    check(err is None and getattr(log, "errors", 0) >= 5 and len(out) == 1,
          f"(a) Log counts each failed write ({getattr(log, 'errors', None)}, error {err!r}) and prints one line {out}")
    s = T.digital_settings(lens_name=LENS_D, min_free_gb=2.0, record_hz=0)
    fc = fake_fc(s)
    for free_gb in (1.5, 50.0):
        out, dd = [], os.path.join(d, f"raw{free_gb:g}")
        usage = lambda p: SimpleNamespace(total=100e9, used=100e9 - free_gb * 1e9, free=free_gb * 1e9)  # noqa: E731
        with patched(shutil, disk_usage=usage), printed(out):
            raw = apb_main.Raw(dict(s, log_dir=dd), fc, (36, 64))
            for fr in frames(60 * 35):                    # 35 s at 60 fps: 3 chunks and the last part
                raw.add(fr)
                join_savers()                             # a run gives 10 s to save each chunk
            raw.flush()
        join_savers()
        files = [f for f in os.listdir(dd) if f.endswith(".npz")]
        if free_gb < 2.0:
            check(not files and len(out) == 1 and "min_free_gb" in out[0],
                  f"(b) {free_gb} GB free, min_free_gb 2: {len(files)} raw chunks written; printed {out}")
        else:
            check(len(files) == 4 and not out, f"(b) {free_gb} GB free: {len(files)} raw chunks written; printed {out}")
    gate, calls = threading.Event(), []

    def slow_savez(path, **data):
        calls.append(path)
        gate.wait(10.0)

    out = []
    with patched(np, savez=slow_savez), printed(out):
        raw = apb_main.Raw(dict(s, log_dir=os.path.join(d, "slow")), fc, (36, 64))
        tg = T.stamps60(60 * 25)
        for fr in frames(60 * 25):                         # chunks end at 10 and 20 s
            raw.add(fr)
        raw.flush()                                        # a third chunk while the first is still saving
        n_saving = sum(t.is_alive() and not t.daemon for t in threading.enumerate()) - 1
        gate.set()
        join_savers()
        raw.add(capture.Frame(np.zeros((36, 64), np.uint8), tg[-1] + 0.96, tg[-1] + 1.0, 0, None))
        raw.flush()                                        # the last save has ended: this chunk is saved
        join_savers()
    check(len(calls) == 2 and n_saving == 1 and getattr(raw, "dropped", None) == 2 and len(out) == 1,
          f"(c) slow disk: {len(calls)} chunks saved, {n_saving} save threads at once, {getattr(raw, 'dropped', None)} "
          f"chunks dropped and counted; printed {out}")


# ---------------------------------------------------------------- D6
def d6_buffer_shape():
    print("D6: a buffer that does not fit the driver size and pixel format")
    y = T.noisy(T.scene(), np.random.default_rng(1))
    for name, fourcc, buf, word in (("YUYV as one channel (720, 2560)", "YUYV", T.pack(y, "YUYV").reshape(H, 2 * W), "pixel_format"),
                                     ("NV12 set, GREY buffer (720, 1280)", "NV12", y.copy(), "pixel_format"),
                                     ("YUYV of half the height (360, 1280, 2)", "YUYV", T.pack(y, "YUYV")[:H // 2].copy(), "video_size")):
        cap = T.FakeCap(fourcc)
        src = capture.DigitalSource(T.digital_settings(), cap=cap)
        try:
            fr, err = T.take(src, cap, buf), None
        except RuntimeError as e:
            fr, err = None, e
        finally:
            src.close()
        check(fr is None and err is not None and word in str(err),
              f"{name}: {err if err else f'no error, picture {fr.img.shape if fr else None} given to Hold'}")


# ---------------------------------------------------------------- D7
def osd_text():
    """OSD-like text as the camera writes it (1280x720, 235 on the strokes, 0 elsewhere)."""
    osd = np.zeros((H, W), np.uint8)
    for i, txt in enumerate(("12.6V", "JUSTICE", "03:21")):
        cv2.putText(osd, txt, (60 + 420 * i, 680), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 235, 3)
    for i, txt in enumerate(("ALT 40", "SAT 0", "1.2A")):
        cv2.putText(osd, txt, (60 + 420 * i, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 235, 3)
    return osd


def moving_attitude(h):
    """Roll and pitch +-8 deg at 0.5 and 0.4 Hz with their rates (about 25 deg/s at most), height h."""
    att, hgt = Series(1e9), Series(1e9)
    a = math.radians(8)
    for t in np.arange(-1, 30, 0.01):
        w1, w2 = 2 * math.pi * 0.5, 2 * math.pi * 0.4
        att.add(t, a * math.sin(w1 * t), a * math.cos(w2 * t), 0.3, a * w1 * math.cos(w1 * t), -a * w2 * math.sin(w2 * t), 0.0)
    for t in np.arange(-1, 30, 0.1):
        hgt.add(t, h)
    return att, hgt


def night_scene(rng):
    """Dark ground at night (median about 25 grey) with ground texture of several sizes (std 6 grey), 3 lights,
    OSD text and noise 4 grey, at 1280x720."""
    t = sum(cv2.resize(rng.standard_normal((2 ** k, 2 ** k * 2)).astype(np.float32), (W, H), interpolation=cv2.INTER_CUBIC)
            for k in range(3, 8))
    g = 25 + 6 * t / t.std() + rng.normal(0, 4, (H, W))
    for x, y in ((300, 200), (700, 420), (1000, 300)):
        cv2.circle(g, (x, y), 6, 255, -1)
    return np.maximum(np.clip(np.round(g), 0, 255).astype(np.uint8), osd_text())


def d7_covered_lens(d):
    print("D7: covered lens, uniform 0-60 grey: picture check and Hold with a moving attitude, as run_live")
    rng = np.random.default_rng(11)
    osd = osd_text()
    osd_small = capture.shrink(osd, 2)
    near = cv2.dilate((osd > 0).astype(np.uint8), np.ones((15, 15), np.uint8))
    mask = np.where(cv2.resize(near, (W // 2, H // 2), interpolation=cv2.INTER_AREA) > 0, 0, 255).astype(np.uint8)
    mask_path = os.path.join(d, "osd_mask.png")
    cv2.imwrite(mask_path, mask)
    att, hgt = moving_attitude(40.0)
    n = 600                    # 10 s at 60 fps. With BLACK_GREY 24, Hold sent 9 and 7 fixes (noise 4, text, no mask, 56 and 60 grey)
    sent, accepted = {}, {}
    t1 = time.time()
    for sd in (0.3, 4.0):
        for level in range(0, 61, 4):
            pool = [capture.shrink(np.clip(np.round(level + rng.normal(0, sd, (H, W))), 0, 255).astype(np.uint8), 2)
                    for _ in range(8)]
            for masked in (False, True):
                s = T.digital_settings(lens_name=LENS_D, mask_file=mask_path if masked else "")
                for text in (False, True):
                    judge, pipe = capture.judge(s), apb_main.Pipe(s, lambda t: None)
                    acc = snt = 0
                    for i in range(n):
                        img = np.roll(pool[i % 8], (int(rng.integers(H // 2)), int(rng.integers(W // 2))), (0, 1))
                        if text:
                            img = np.maximum(img, osd_small)   # the camera writes its OSD on the picture
                        t = 1.0 + i / 60
                        fr = capture.Frame(img, t - s["video_delay_s"], t, i + 1, judge(img))
                        acc += fr.why is None
                        snt += pipe.step(fr, att, hgt)[1]
                    key = f"noise {sd}, {'OSD text' if text else 'no text'}, {'mask' if masked else 'no mask'}"
                    if acc:
                        accepted.setdefault(key, []).append(level)
                    if snt:
                        sent.setdefault(key, []).append((level, snt))
    for k, v in accepted.items():
        print(f"    | picture check accepted {k}: levels {v}")
    for k, v in sent.items():
        print(f"    | fixes sent {k}: (level, fixes) {v}")
    check(not sent, f"no fix sent in 128 cases of {n} frames ({time.time() - t1:.0f} s): {sent or 'none sent'}")
    check(not accepted, f"the picture check refuses each covered-lens picture: {accepted or 'all refused'}")
    s = T.digital_settings(lens_name=LENS_D)
    img = capture.shrink(night_scene(np.random.default_rng(12)), 2)
    small = cv2.resize(img, (img.shape[1] // 8, img.shape[0] // 8), interpolation=cv2.INTER_AREA).astype(np.float32)
    why = capture.judge(s)(img)
    check(why is None, f"night scene with ground texture (median {np.median(small):.0f} grey, MAD "
                       f"{np.median(np.abs(small - np.median(small))):.1f} at 1/8 size), lights and OSD text: accepted ({why})")


# ---------------------------------------------------------------- D8
def d8_timing_stamps():
    print("D8: hold_timing stamps its test frames at the camera's frame interval")

    class FakeHold:
        def __init__(self):
            self.t, self.covered = [], True

        def process(self, img, t, att, hgt):
            self.t.append(t)
            self.covered &= att.last_t() > t and hgt.last_t() > t

    for name, s, want in (("digital 60 fps", T.digital_settings(video_fps=60), 1 / 60),
                          ("digital 10 fps", T.digital_settings(video_fps=10), 1 / 10),
                          ("analog", S.load("/nonexistent"), 1 / 25)):
        h = FakeHold()
        with patched(apb_main, make_hold=lambda s, shape, h=h: h):
            apb_main.hold_timing(s, (36, 64), n=60)
        dt = np.diff(h.t)
        check(len(h.t) == 60 and np.allclose(dt, want) and h.covered,
              f"{name}: frames {dt.mean() * 1e3:.1f} ms apart (want {want * 1e3:.1f}), attitude and height cover each "
              f"frame {h.covered}")


def main():
    t0 = time.time()
    only = sys.argv[1:]
    with tempfile.TemporaryDirectory() as d:
        for name, fn in (("d1", lambda: d1_fc_fails(d)), ("d2", lambda: d2_camera_stops(d)), ("d3", d3_close_blocked),
                         ("d4", lambda: d4_stamp_line(d)), ("d5", lambda: d5_full_disk(d)), ("d6", d6_buffer_shape),
                         ("d7", lambda: d7_covered_lens(d)), ("d8", d8_timing_stamps)):
            if not only or name in only:
                try:
                    fn()
                except Exception as e:      # an old fault can raise: count it and go on with the next part
                    check(False, f"{name} raised {e!r}")
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
