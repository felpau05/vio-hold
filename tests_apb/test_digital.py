#!/usr/bin/env python3
"""The digital camera path (Orqa Justice D, MIPI CSI, 1280x720 at 60 fps) with injected buffers: no camera.

Buffer layouts are OpenCV's with CAP_PROP_CONVERT_RGB off: YUYV and UYVY (h, w, 2), NV12 (h * 3/2, w), GREY (h, w).
Parts: grey picture from each layout, picture checks (still scene, frozen stream, covered lens), stamps,
settings (old files, new keys), capture mode, recorder rate (record_hz), raw frames from a child process
(video_command, a Python stand-in for gst-launch), video files, check mode, and the justice_d profile.
Run: ~/.venvs/vio/bin/python tests_apb/test_digital.py      (about 25 s)
"""
import glob
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
import apb_main            # noqa: E402
import capture             # noqa: E402
import settings as S       # noqa: E402
from hold import Series    # noqa: E402

NEW_KEYS = ("camera", "video_size", "video_fps", "pixel_format", "video_pipeline", "video_reduce", "video_command",
            "record_hz")
W, H = 1280, 720
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


class FakeCap:
    """Stands in for cv2.VideoCapture: read() gives the buffers the test puts in q, in order. The driver
    stamp (CAP_PROP_POS_MSEC) is the read time minus lag_s, or 0 like a GStreamer stream position."""

    def __init__(self, fourcc, lag_s=0.02):
        self.q, self.fourcc, self.lag, self.t, self.released = queue.Queue(), fourcc, lag_s, None, False

    def read(self):
        try:
            buf = self.q.get(timeout=0.05)
        except queue.Empty:
            return False, None
        self.t = time.monotonic() - self.lag if self.lag is not None else 0.0
        return True, buf

    def get(self, prop):
        return {cv2.CAP_PROP_POS_MSEC: (self.t or 0.0) * 1e3, cv2.CAP_PROP_FRAME_WIDTH: W, cv2.CAP_PROP_FRAME_HEIGHT: H,
                cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*self.fourcc)}.get(prop, 0.0)

    def set(self, prop, v):
        return True

    def isOpened(self):
        return True

    def release(self):
        self.released = True


def digital_settings(**kw):
    return {**S.load("/nonexistent"), **dict(camera="digital", video_device="/dev/video2", video_size=[W, H], video_fps=60,
                                             video_reduce=2, video_standard="none"), **kw}


def scene(seed=0):
    rng = np.random.default_rng(seed)
    g = cv2.GaussianBlur(rng.normal(128, 50, (H, W)).astype(np.float32), (0, 0), 3) * 3 - 256
    return np.clip(g, 20, 235).astype(np.float32)


def noisy(base, rng, sd=0.3):
    return np.clip(np.round(base + rng.normal(0, sd, base.shape)), 0, 255).astype(np.uint8)


def pack(y, fourcc):
    """y (H, W) uint8 in a raw buffer of this layout. Chroma has values Y never has here, so a wrong byte shows."""
    if fourcc in ("YUYV", "UYVY"):
        b = np.empty((H, W, 2), np.uint8)
        b[:, :, 0 if fourcc == "YUYV" else 1] = y
        b[:, :, 1 if fourcc == "YUYV" else 0] = np.tile(np.array([3, 252], np.uint8), W // 2)
        return b
    if fourcc == "NV12":
        return np.vstack([y, np.tile(np.array([5, 250], np.uint8), (H // 2, W // 2))])
    if fourcc == "BGR3":
        return cv2.cvtColor(y, cv2.COLOR_GRAY2BGR)
    return y.copy()


def take(src, cap, buf):
    cap.q.put(buf)
    return src.newest(timeout=1.0)


def layouts_part():
    print("grey picture from each buffer layout, reduced 2x (1280x720 -> 640x360)")
    y = noisy(scene(), np.random.default_rng(1))
    want = cv2.resize(y, (W // 2, H // 2), interpolation=cv2.INTER_AREA)
    for fourcc in ("YUYV", "UYVY", "NV12", "GREY", "BGR3"):
        cap = FakeCap(fourcc)
        src = capture.DigitalSource(digital_settings(), cap=cap)
        try:
            fr = take(src, cap, pack(y, fourcc))
        finally:
            src.close()
        ok = fr is not None and fr.img.shape == (360, 640) and fr.img.dtype == np.uint8 and np.array_equal(fr.img, want)
        check(ok, f"{fourcc}: {None if fr is None else fr.img.shape} equals the INTER_AREA reduction of Y"
                  f"{'' if fr is None else f', why={fr.why}'}")
    check(cap.released, "close() releases the capture")
    cap = FakeCap("YUYV")
    src = capture.DigitalSource(digital_settings(video_reduce=1), cap=cap)
    try:
        fr = take(src, cap, pack(y, "YUYV"))
    finally:
        src.close()
    check(fr is not None and np.array_equal(fr.img, y), "video_reduce 1 gives the full 1280x720 Y picture")


def judge_part():
    print("picture checks")
    s = digital_settings()
    rng = np.random.default_rng(2)
    base = scene(3)
    cap = FakeCap("YUYV")
    src = capture.DigitalSource(s, cap=cap)
    try:
        whys = [take(src, cap, pack(noisy(base, rng), "YUYV")) for _ in range(120)]
        bad = [f.why for f in whys if f is None or f.why]
        mad = min(cv2.absdiff(whys[i].img, whys[i + 1].img).mean() for i in range(119))
        check(not bad and mad < s["repeat_mad"], f"120 fresh frames of a still scene, noise 0.3 grey: {len(bad)} refused "
                                                 f"(smallest mean difference {mad:.3f} grey, analog repeat_mad {s['repeat_mad']})")
        check(src.newest(timeout=0.1) is None, "no new buffer inside the timeout: newest() gives None")
        frozen = pack(noisy(base, rng), "YUYV")
        res = [take(src, cap, frozen.copy()) for _ in range(6)]
        check(res[0].why is None and all(f.why and "repeated" in f.why for f in res[1:]),
              f"frozen stream: first accepted, the 5 repeats refused ({res[1].why})")
        black = pack(noisy(np.full((H, W), 16.0), rng), "YUYV")
        fr = take(src, cap, black)
        check(fr.why is not None, f"black picture (covered lens) refused: {fr.why}")
        osd = noisy(np.full((H, W), 16.0), rng)
        for i, txt in enumerate(("12.6V", "JUSTICE", "03:21")):
            cv2.putText(osd, txt, (60 + 420 * i, 680), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 235, 3)
        fr2 = take(src, cap, pack(osd, "YUYV"))
        check(fr2.why is not None, f"black picture with OSD text (covered lens) refused: {fr2.why}")
        fr3 = take(src, cap, pack(noisy(base, rng), "YUYV"))
        check(fr3.why is None, f"the next good picture is accepted: {fr3.why}")
        snow = np.full((H, W), 200.0)
        for x in (200, 600, 1000):
            snow[300:340, x:x + 60] = 60.0                 # three dark tarps on snow
        fr4 = take(src, cap, pack(noisy(snow, rng), "YUYV"))
        check(fr4.why is None, f"bright flat snow with 3 tarps and almost no noise is accepted: {fr4.why}")
        flat = take(src, cap, pack(noisy(np.full((H, W), 128.0), rng), "YUYV"))
        check(flat.why is not None, f"flat grey picture refused: {flat.why}")
    finally:
        src.close()


def stamp_part():
    print("stamps")
    s = digital_settings(video_delay_s=0.040)
    y = noisy(scene(), np.random.default_rng(4))
    cap = FakeCap("NV12", lag_s=0.02)
    src = capture.DigitalSource(s, cap=cap)
    try:
        fr = take(src, cap, pack(y, "NV12"))
    finally:
        src.close()
    check(fr is not None and abs(fr.t_grab - cap.t) < 1e-6 and abs(fr.t0 - (fr.t_grab - 0.040)) < 1e-9
          and src.stamp == "driver", f"driver stamp used; t0 = stamp - video_delay_s (stamp from {src.stamp})")
    cap = FakeCap("NV12", lag_s=None)                          # stamp 0: a stream position, not the clock
    src = capture.DigitalSource(s, cap=cap)
    try:
        t1 = time.monotonic()
        fr = take(src, cap, pack(y, "NV12"))
    finally:
        src.close()
    check(fr is not None and src.stamp == "read" and t1 <= fr.t_grab <= time.monotonic()
          and abs(fr.t0 - (fr.t_grab - 0.040)) < 1e-9, f"implausible driver stamp: read time used (stamp from {src.stamp})")


def settings_part(d):
    print("settings")
    p = os.path.join(d, "old.json")
    old = {k: v for k, v in S.load("/nonexistent").items() if k not in NEW_KEYS}
    json.dump(old, open(p, "w"))
    s = S.load(p)
    picked = []
    real = capture.V4L2Source, capture.DigitalSource
    capture.V4L2Source = lambda s: picked.append("analog") or "analog"
    capture.DigitalSource = lambda s, cap=None: picked.append("digital") or "digital"
    try:
        apb_main.open_video(s)
        apb_main.open_video(dict(s, camera="digital"))
    finally:
        capture.V4L2Source, capture.DigitalSource = real
    check(s["camera"] == "analog" and picked == ["analog", "digital"],
          f"old settings file (no new keys) loads; camera {s['camera']}; open_video picks {picked}")
    seen = []
    real_ll = apb_main.load_lens
    apb_main.load_lens = lambda path, name, w, h, field=None: seen.append(field) or (np.eye(3), np.zeros(4))
    try:
        apb_main.lens(s, (288, 360))
        apb_main.lens(dict(s, camera="digital"), (360, 640))
    finally:
        apb_main.load_lens = real_ll
    check(seen == [0, None], f"lens(): field 0 for analog top field, None for the digital camera -> {seen}")
    for bad, word in (({"camera": "usb"}, "must be one of"), ({"video_reduce": 1.5}, "whole number"),
                      ({"video_reduce": 0}, "whole number"), ({"pixel_format": "YUV"}, "4 characters"),
                      ({"video_size": [1280]}, "list of 2"), ({"video_command": 5}, "must be a str"),
                      ({"record_hz": "30"}, "must be a number"), ({"record_rate": 30}, "unknown key")):
        json.dump(bad, open(p, "w"))
        try:
            S.load(p)
            check(False, f"{bad} accepted")
        except ValueError as e:
            check(word in str(e), f"{bad} refused: {e}")
    json.dump({"video_standard": "none", "pixel_format": "NV12", "video_pipeline": "v4l2src ! appsink"}, open(p, "w"))
    check(S.load(p)["video_standard"] == "none", "video_standard 'none', a FOURCC and a pipeline are accepted")
    json.dump({"video_command": "gst-launch-1.0 -q v4l2src ! fdsink fd=1", "record_hz": 30}, open(p, "w"))
    try:
        s = S.load(p)
    except ValueError as e:
        s = {"error": e}
    check(s.get("record_hz") == 30 and str(s.get("video_command")).startswith("gst"),
          f"video_command and record_hz are accepted {s.get('error', '')}")
    check(S.DEFAULTS.get("record_hz") == 0 and S.DEFAULTS.get("video_command") == "",
          "defaults: record_hz 0 (keep each frame), video_command empty (not used)")


def mode_part():
    print("capture mode")
    s = digital_settings()
    info = {"width": W, "height": H, "fourcc": "YUYV"}
    a = capture.capture_mode(S.load("/nonexistent"), {"standard": "PAL"}, (288, 360))
    d2 = capture.capture_mode(s, info, (360, 640))
    d1 = capture.capture_mode(dict(s, video_reduce=1), info, (720, 1280))
    check(len({a, d1, d2}) == 3 and "digital" in d2 and "2" in d2.replace("1280x720", ""),
          f"analog '{a}', digital '{d2}', digital reduce 1 '{d1}' all differ")
    check(a == "PAL field=top 360x288", "the analog mode string is unchanged, so old analog calibrations stay valid")
    c = capture.capture_mode(dict(s, video_command="x", video_pipeline="y"), info, (360, 640))
    check("command" in c and "gstreamer" not in c and c != d2,
          f"video_command (read-time stamps) has its own mode '{c}', so run refuses a calibration made without it")


def stamps60(n):
    """n frame stamps at 60 fps from t = 100 s, each up to 3 ms early or late (read-time jitter)."""
    return 100.0 + np.arange(n) / 60 + np.random.default_rng(9).uniform(-0.003, 0.003, n)


class ListSource:
    """newest() gives these frames in order, then None after a short wait."""

    def __init__(self, frames):
        self.frames, self.info, self.stamp = list(frames), {"width": W, "height": H}, "driver"

    def newest(self, timeout=0.5):
        if self.frames:
            return self.frames.pop(0)
        time.sleep(min(timeout, 0.05))
        return None

    def close(self):
        pass


def fake_fc(s):
    """A flight controller that is always ready: level attitude and 50 m height at each stamp of the test."""
    att, hgt = Series(1e9), Series(1e9)
    for t in (0.0, 1e4):
        att.add(t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        hgt.add(t, 50.0)
    return SimpleNamespace(s=s, att=att, hgt=hgt, lpos=Series(1e9), firmware="px4", armed=False, local_pos=lambda: None,
                           clock=SimpleNamespace(ready=lambda: True), rate=lambda *a: 100.0, age=lambda *a: 0.0)


def per_slot(kept, hz):
    """True when kept has at most one stamp in each 1/hz slot and at most hz stamps in each whole second."""
    kept = np.asarray(kept)
    slots = np.floor(kept * hz)
    return len(kept) > 0 and len(set(slots)) == len(slots) and np.unique(np.floor(kept), return_counts=True)[1].max() <= hz


def recorder_part(d):
    print("recorder rate: record_hz on a 60 fps stream with 3 ms stamp jitter")
    tg = stamps60(120)
    small = np.zeros((36, 64), np.uint8)
    frames = lambda: [capture.Frame(small, t - 0.04, t, i + 1, "repeated buffer" if i in (4, 5) else None)  # noqa: E731
                      for i, t in enumerate(tg)]
    good = np.delete(tg, [4, 5])
    rec = {hz: apb_main.record(ListSource(frames()), fake_fc(digital_settings(record_hz=hz)), 0.3, say=lambda x: None)
           for hz in (0, 30)}
    check(np.array_equal(rec[0]["t_grab"], good), f"record(), record_hz 0: each usable frame kept, as before "
                                                   f"({len(rec[0]['t_grab'])} of 120; 2 refused)")
    k = rec[30]["t_grab"]
    check(len(k) >= 57 and np.isin(k, good).all() and per_slot(k, 30) and len(rec[30]["imgs"]) == len(k),
          f"record(), record_hz 30: {len(k)} of {len(good)} usable frames kept with their exact stamps, at most 30 "
          "each second (a smallest gap of 1/30 s would keep about 46)")
    real = apb_main.Pipe, apb_main.start_problems
    hold, raw = {}, {}
    for hz in (0, 30):
        out = os.path.join(d, f"raw{hz}")
        s = digital_settings(record_hz=hz, raw_record=True, log_dir=out)
        hold[hz] = got = []

        class CountPipe:          # stands in for Pipe and Hold: notes each frame it gets
            note, good = None, 0

            def __init__(self, s, fc_pos):
                pass

            def step(self, fr, att, hgt, gate=None):
                got.append(fr.t_grab)
                return None, False, "counted"

        apb_main.Pipe, apb_main.start_problems = CountPipe, lambda *a: []
        try:
            apb_main.run_live(s, ListSource(frames()), fake_fc(s), seconds=0.5, log_path=os.path.join(out, "run.csv"))
        finally:
            apb_main.Pipe, apb_main.start_problems = real
        for th in threading.enumerate():              # Raw saves each chunk in a thread of its own
            if th is not threading.current_thread() and not th.daemon:
                th.join()
        files = sorted(glob.glob(os.path.join(out, "raw_*.npz")))
        raw[hz] = np.concatenate([np.load(p)["t_grab"] for p in files]) if files else np.zeros(0)
    after = tg[1:]                                    # run_live takes the first usable frame for its start checks
    check(np.array_equal(hold[0], after) and np.array_equal(hold[30], after),
          f"run: Hold gets each frame with record_hz 0 and 30 ({len(hold[0])} and {len(hold[30])} of {len(after)})")
    check(np.array_equal(raw[0], after), f"run --raw, record_hz 0: each frame saved, as before ({len(raw[0])})")
    check(len(raw[30]) >= 57 and np.isin(raw[30], after).all() and per_slot(raw[30], 30),
          f"run --raw, record_hz 30: {len(raw[30])} of {len(after)} frames saved with their exact stamps, at most 30 each second")


# Stands in for "gst-launch-1.0 ... ! fdsink fd=1" (no GStreamer in a test): writes its process id to a file, then the
# bytes of a file to standard output in pieces of 100003 bytes (a frame arrives in parts), then sleeps (gives no more
# data) or stops with exit code 3.
CHILD = r"""
import os, sys, time
path, then, pidfile = sys.argv[1:4]
with open(pidfile, "w") as f:
    f.write(str(os.getpid()))
data, out = open(path, "rb").read(), sys.stdout.buffer
for i in range(0, len(data), 100003):
    out.write(data[i:i + 100003])
    out.flush()
    time.sleep(0.002)
if then == "sleep":
    time.sleep(60)
sys.exit(3)
"""
n_cmd = 0


def child_command(d, data, then):
    """(video_command, pid file) for a child that writes data, then sleeps or stops. '; exit $?' keeps the shell
    alive as the parent of the child, as a command with a pipe or a list does: close() must stop both."""
    global n_cmd
    n_cmd += 1
    base = os.path.join(d, f"cmd{n_cmd}")
    with open(base + ".py", "w") as f:
        f.write(CHILD)
    with open(base + ".raw", "wb") as f:
        f.write(data)
    return f"{sys.executable} {base}.py {base}.raw {then} {base}.pid; exit $?", base + ".pid"


def gone(pidfile, wait_s=3.0):
    """True when the child named in pidfile has started and then ended (no /proc entry, or a zombie) within wait_s."""
    end, pid = time.monotonic() + wait_s, None
    while time.monotonic() < end:
        try:
            pid = pid or int(open(pidfile).read())
            with open(f"/proc/{pid}/stat") as f:
                if f.read().rsplit(")", 1)[1].split()[0] == "Z":
                    return True
        except ValueError:          # the child has not written its pid yet
            pass
        except OSError:
            if pid:
                return True
        time.sleep(0.05)
    return False


def command_source(s):
    """DigitalSource(s), with each OpenCV capture refused: the frames must come from video_command. None if it fails."""
    real = cv2.VideoCapture

    def refuse(*a):
        raise RuntimeError(f"the test refuses an OpenCV capture {a}")

    cv2.VideoCapture = refuse
    try:
        return capture.DigitalSource(s)
    except Exception as e:
        check(False, f"DigitalSource with video_command: {e}")
        return None
    finally:
        cv2.VideoCapture = real


def command_part(d):
    print("video_command: raw frames from a child process")
    rng = np.random.default_rng(7)
    ys = [noisy(scene(10 + i), rng) for i in range(3)]
    small = [cv2.resize(y, (W // 2, H // 2), interpolation=cv2.INTER_AREA) for y in ys]
    cmd, pid = child_command(d, b"".join(pack(y, "YUYV").tobytes() for y in ys), "sleep")
    s = digital_settings(pixel_format="YUYV", video_command=cmd, video_pipeline="v4l2src ! appsink")
    t1 = time.monotonic()
    src = command_source(s)
    if src is not None:
        try:
            end = time.monotonic() + 5.0
            while src.seq < 3 and time.monotonic() < end:
                time.sleep(0.01)
            fr = src.newest(timeout=1.0)
            check(fr is not None and fr.seq == 3 and np.array_equal(fr.img, small[2]),
                  "3 YUYV frames in pieces: the third is the INTER_AREA reduction of its Y. video_command has priority "
                  "over video_pipeline, and no OpenCV capture is opened")
            check(fr is not None and src.stamp == "read" and t1 < fr.t_grab < time.monotonic()
                  and abs(fr.t0 - (fr.t_grab - s["video_delay_s"])) < 1e-9, f"stamp: the read time (stamp from {src.stamp})")
            check(src.newest(timeout=0.3) is None, "the command gives no more data: newest() gives None after the timeout")
        finally:
            src.close()
        check(gone(pid), "close() stops the command and the child process of its shell")
    cmd, pid = child_command(d, pack(ys[0], "YUYV").tobytes() + bytes(1000), "stop")
    src = command_source(digital_settings(pixel_format="YUYV", video_command=cmd))
    if src is not None:
        try:
            fr, nxt = src.newest(timeout=3.0), src.newest(timeout=0.5)
            code = src.cap.proc.poll()
        finally:
            src.close()
        check(fr is not None and np.array_equal(fr.img, small[0]) and nxt is None and code == 3,
              f"the command stops after 1 frame and part of a frame: the frame, then None (exit code {code})")
    try:                                              # pixel_format "" gives no frame size
        capture.DigitalSource(digital_settings(video_command="true", video_device="/nonexistent")).close()
        check(False, "video_command with no pixel_format accepted")
    except RuntimeError as e:
        check("pixel_format" in str(e) and "video_command" in str(e), f"video_command with no pixel_format refused: {e}")
    cmd, pid = child_command(d, pack(ys[0], "YUYV").tobytes(), "sleep")
    prog = (f"import sys\nsys.path.insert(0, {PROJ!r})\nimport capture, settings as S\n"
            f"s = dict(S.load('/nonexistent'), camera='digital', video_device='/nonexistent', pixel_format='YUYV', "
            f"video_command={cmd!r})\nsrc = capture.DigitalSource(s)\nprint('frame', src.newest(timeout=5.0) is not None)\n"
            "raise RuntimeError('a test error with the camera open')\n")
    with open(os.path.join(d, "prog.txt"), "w+") as f:    # a file, not a pipe: a child that stays alive keeps a pipe open
        r = subprocess.run([sys.executable, "-c", prog], stdout=f, stderr=subprocess.STDOUT, timeout=60)
        f.seek(0)
        txt = f.read()
    check(r.returncode == 1 and "frame True" in txt and "test error" in txt and gone(pid),
          "the program stops with an error before close(): the command does not stay alive"
          + ("" if r.returncode == 1 else f" | {txt.strip()[-300:]}"))
    for what, data, then, word in (("stops", bytes(1000), "stop", "exit code 3"), ("gives no data", b"", "sleep", "no frame")):
        cmd, pid = child_command(d, data, then)
        lines = run_check(digital_settings(log_dir=d, pixel_format="YUYV", video_command=cmd, video_device="/nonexistent"))
        hit = [x for x in lines if x.startswith("FAIL video_command")]
        check(len(hit) == 1 and word in hit[0] and "check video_command" in hit[0] and gone(pid),
              f"check mode, the command {what}: {hit}")


def run_check(s, video=None):
    """mode_check lines with no flight controller and a fixed Hold time. video: stands in for open_video."""
    real = apb_main.open_fc, apb_main.open_video, apb_main.hold_timing

    def no_fc(s, say=print):
        raise RuntimeError("no flight controller in this test")

    apb_main.open_fc, apb_main.hold_timing = no_fc, lambda s, shape: [20.0] * 10
    apb_main.open_video = video or apb_main.open_video
    lines = []
    try:
        apb_main.mode_check(s, SimpleNamespace(set_params=False), say=lines.append)
    finally:
        apb_main.open_fc, apb_main.open_video, apb_main.hold_timing = real
    return lines


def file_part(d):
    print("video file: a digital frame is reduced by video_reduce; an analog frame is split into fields as before")
    p = os.path.join(d, "clip.avi")
    rng = np.random.default_rng(8)
    wr = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*"MJPG"), 60, (W, H))
    for i in range(3):
        wr.write(cv2.cvtColor(noisy(scene(20 + i), rng), cv2.COLOR_GRAY2BGR))
    wr.release()
    cap = cv2.VideoCapture(p)
    greys = [cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY) for _ in range(3)]
    cap.release()

    def frames(s):
        src = capture.FileSource(p, s)
        try:
            return [src.newest() for _ in range(3)]
        finally:
            src.close()

    dig = frames(digital_settings(video_reduce=4))
    want = [cv2.resize(g, (W // 4, H // 4), interpolation=cv2.INTER_AREA) for g in greys]
    check(all(f is not None and np.array_equal(f.img, w) for f, w in zip(dig, want)),
          f"digital, video_reduce 4: {dig[0].img.shape} is the INTER_AREA reduction of each frame, no field split")
    ana = frames(S.load("/nonexistent"))
    want = [capture.split_field(g, "top") for g in greys]
    check(all(f is not None and np.array_equal(f.img, w) for f, w in zip(ana, want)),
          f"analog: {ana[0].img.shape} is the top field, width halved, as before")


def check_mode_part(d):
    print("check mode with a digital camera at 60 and at 45 fps (no flight controller)")
    rng = np.random.default_rng(5)
    base = scene(6)
    bufs = [pack(noisy(base, rng), "YUYV") for _ in range(8)]
    real = apb_main.open_fc, apb_main.open_video, apb_main.hold_timing

    def no_fc(s, say=print):
        raise RuntimeError("no flight controller in this test")

    out = {}
    for hz in (60, 45):
        cap, stop = FakeCap("YUYV"), threading.Event()

        def feed():
            i, t = 0, time.monotonic()
            while not stop.is_set():
                cap.q.put(bufs[i % len(bufs)])
                i, t = i + 1, t + 1.0 / hz
                time.sleep(max(0.0, t - time.monotonic()))

        th = threading.Thread(target=feed, daemon=True)
        th.start()
        apb_main.open_fc = no_fc
        apb_main.open_video = lambda s: capture.DigitalSource(s, cap=cap)
        apb_main.hold_timing = lambda s, shape: [20.0] * 10
        lines = []
        kw = {} if hz == 60 else dict(lens_name="Justice D full sensor (assumed)", record_hz=30)
        try:
            apb_main.mode_check(digital_settings(log_dir=d, **kw), SimpleNamespace(set_params=False), say=lines.append)
        finally:
            stop.set()
            th.join()
            apb_main.open_fc, apb_main.open_video, apb_main.hold_timing = real
        out[hz] = lines
    for x in out[60]:
        print("    | " + x)
    vid = [x for x in out[60] if x.startswith("PASS video:")]
    check(bool(vid) and "1280x720" in vid[0] and "YUYV" in vid[0] and "stamps from driver" in vid[0],
          f"video line: driver size, pixel format, stamp source: {vid}")
    check(any(x.startswith("PASS video rate") for x in out[60]), "60 fps measured against video_fps 60: PASS")
    check(any(x.startswith("FAIL video rate") for x in out[45]), "45 fps measured against video_fps 60: FAIL")
    check(not any("standard" in x for x in out[60]), "no video standard line for the digital camera")
    disk = [x for x in out[60] if "free disk" in x]
    check(bool(disk) and "640x360" in disk[0] and "MB/s" in disk[0], f"free disk uses the real picture and rate: {disk}")
    disk = [x for x in out[45] if "free disk" in x]
    check(bool(disk) and ", 30 Hz)" in disk[0], f"record_hz 30: free disk counts 30 pictures each second: {disk}")
    lens = [x for x in out[60] if "lens profile" in x]
    check(len(lens) == 1 and lens[0].startswith("FAIL") and "16:9" in lens[0] and "justice_d" in lens[0],
          f"the 4:3 lens profile on the 16:9 picture: {lens}")
    lens = [x for x in out[45] if "lens profile" in x]
    check(len(lens) == 1 and lens[0].startswith("PASS"), f"a 16:9 lens profile: {lens}")

    def no_video(s):
        raise RuntimeError("no camera in this test")

    lines = run_check(dict(S.load("/nonexistent"), log_dir=d), video=no_video)
    check(not any("lens profile" in x for x in lines), "analog: no lens shape line (a field is not the profile's shape)")


def profile_part(d):
    print("tools/use_profile.py apply justice_d")
    p = os.path.join(d, "settings_profile.json")
    tool = os.path.join(PROJ, "tools", "use_profile.py")
    run = lambda *names: subprocess.run([sys.executable, tool, "apply", *names, "--settings", p],     # noqa: E731
                                        capture_output=True, text=True)
    r = run("justice_d", "outdoor_day", "goal_a", "ardupilot")
    try:
        s = S.load(p)
    except ValueError as e:
        s = {}
        print(f"    | {e}")
    want = dict(camera="digital", video_device="/dev/video2", video_size=[W, H], video_fps=60, video_reduce=2,
                lens_name="Justice D full sensor (assumed)", readout_s=0.016, video_delay_s=0.040, record_hz=30,
                video_command="")
    got = {k: s.get(k) for k in want}
    check(r.returncode == 0 and got == want, f"apply justice_d outdoor_day goal_a ardupilot -> loader accepts: {got}"
                                             + ("" if r.returncode == 0 else f" | {r.stdout.strip()} {r.stderr.strip()}"))
    with open(os.path.join(PROJ, "tools", "profiles", "justice_d.json")) as f:
        why = json.load(f)["_why"]
    unmeasured = ("lens_name", "readout_s", "video_delay_s", "video_reduce", "record_hz")
    check(all(str(why.get(k, "")).startswith("assumed:") for k in unmeasured),
          "each unmeasured value has a _why note that starts with 'assumed:'")
    example = ("gst-launch-1.0 -q v4l2src device=/dev/video2 ! video/x-raw,width=1280,height=720,format=YUY2 ! "
               "fdsink fd=1")
    note = str(why.get("video_command", ""))
    check(example in note and "assumed:" in note and "YUY2" in note.split("assumed:", 1)[-1],
          "the video_command note gives the gst-launch example and says that its format name is assumed")
    s.update(calibrated=True, video_delay_s=0.052, readout_s=0.012)
    S.save(s, p)
    r = run("justice_d", "outdoor_day", "goal_b", "ardupilot")
    s2 = S.load(p)
    check(r.returncode == 0 and s2["video_delay_s"] == 0.052 and s2["readout_s"] == 0.012,
          "apply again after calib keeps the measured video_delay_s and readout_s")
    r = run("outdoor_day", "goal_a")
    s3 = S.load(p)
    check(r.returncode == 0 and s3["camera"] == "analog" and s3["video_delay_s"] == S.DEFAULTS["video_delay_s"]
          and s3["lens_name"] == S.DEFAULTS["lens_name"] and s3.get("record_hz") == 0,
          "apply without justice_d gives the analog camera and its values back (record_hz 0)")


def main():
    t0 = time.time()
    with tempfile.TemporaryDirectory() as d:
        layouts_part()
        judge_part()
        stamp_part()
        settings_part(d)
        mode_part()
        recorder_part(d)
        command_part(d)
        file_part(d)
        check_mode_part(d)
        profile_part(d)
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
