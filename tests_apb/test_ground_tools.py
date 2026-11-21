#!/usr/bin/env python3
"""The ground-tool corrections of the 2026-11-20 rehearsal (integ/rehearsal_2026-11-20/README.txt, section 3) and of
its review (integ/integration_2026-11-20/review/REVIEW.txt), without a simulator.

A. timing (D1, B1, B2, S2): the frames of the 15 s before the first switch with 15 s in the air in no hold mode
   before it, only when they hold a wiggle and are the run log's own frames. The fixture log writes t = t_grab -
   video_delay_s, as capture and Log do. calib.Rec is replaced by a spy: this tests the frame choice and what timing
   writes, not calib's search (tests_apb/test_calib.py does that).
B. D2 check on ArduPilot: the height stream is GLOBAL_POSITION_INT, and fc_link asks ArduPilot for it.
C. D3 bench, mask, calib: one FAIL line for a RuntimeError or a ValueError; other errors keep their traceback (S4: a
   damaged lens file is named).
D. D4 free disk (Justice D, no_sd): check needs at least min_free_gb; when check passes, Raw records; the line gives
   the minutes of raw recording, and WARN under 10 min (S3).
The expected frames are written by hand (109.0 to 123.9 s), not with the code's own expression.
Run: ~/.venvs/vio/bin/python tests_apb/test_ground_tools.py      (about 20 s)
"""
import contextlib
import csv
import io
import os
import shutil
import sys
import tempfile
from collections import namedtuple
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
import apb_main            # noqa: E402
import calib               # noqa: E402
import fc_link             # noqa: E402
import settings as S       # noqa: E402
import use_profile as U    # noqa: E402
from hold import Series    # noqa: E402

fails = 0
D = tempfile.mkdtemp()
DLY = 0.07                 # video_delay_s of the fixture flight
STAMP = "20261120_100000"
SET = os.path.join(D, "settings.json")
S.save(dict(S.load("/nonexistent"), video_delay_s=DLY, min_height_m=3.0), SET)


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


def run(fn, *a, **k):
    """(result, printed text). An exception is a result too: the old code raised where the new one prints FAIL."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        try:
            r = fn(*a, **k)
        except Exception as e:      # noqa: BLE001
            r = e
    return r, out.getvalue()


# ---------------------------------------------------------------- A. timing
print("A. timing: the wiggle window of the right switch, from the run's own frames")


class RecSpy:
    """calib.Rec stand-in: keeps the t_grab it gets. Its answers are class settings (default: 0 deg/s, so FAIL)."""
    got, rate, n, result = None, 0.0, 0, (0.041, 0.017, 0.5)

    def __init__(self, rec, mask=None):
        RecSpy.got = np.array(rec["t_grab"])
        self.rate_dps, self.ia = RecSpy.rate, range(RecSpy.n)

    def timing(self, R):
        return RecSpy.result

    def coarse(self):
        return [(RecSpy.result[2], "front")]


def roll(t, wig, gust):
    """An 8 s roll wiggle from wig (s), 0.15 rad at 0.67 Hz (25 deg/s RMS), plus turning all the time with amplitude
    gust rad at 0.43 Hz (a hold in gusts; 0.05 rad: 5.5 deg/s RMS)."""
    r = np.where((t >= wig) & (t <= wig + 8.0), 0.15 * np.sin(2 * np.pi * (t - wig) / 1.5), 0.0) if wig else 0 * t
    return r + gust * np.sin(2 * np.pi * t / 2.3)


def flight(notes, wig=112.0, gust=0.0, raw_stamp=STAMP, shift=0.0, drop=(), held=(), blank=None):
    """[run log, raw files] of a fixture run: a frame every 0.05 s from t_grab 100 to 160 s, log t = t_grab - DLY,
    every second frame recorded in 10 s chunks. On the ground (0.5 m) before 104 s, then 50 m. notes: {t_grab: note}.
    held: (t1, t2) spans in LOITER (else ALT_HOLD). blank: the t_grab of a row after which comes a row with no frame
    (no t) with the 'engaged' note. shift: added to the raw t_grab (frames of another run). drop: chunks left out."""
    d = tempfile.mkdtemp(dir=D)
    tl = 100.0 + 0.05 * np.arange(1200)
    log = os.path.join(d, f"run_{STAMP}.csv")
    with open(log, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(apb_main.Log.COLS)
        for x in tl:
            k = round(float(x), 2)
            mode = "LOITER" if any(a <= k < b for a, b in held) else "ALT_HOLD"
            w.writerow([f"{x - DLY:.4f}", "", "", "", "", "", 0, "", "0.0", "", "0.5" if k < 104 else "50.0",
                        notes.get(k, ""), mode])
            if blank == k:
                w.writerow(["", "", "", "", "", "", 0, "no frame", "0.0", "", "", "engaged: FC mode LOITER", "LOITER"])
    raws, tr = [], tl[::2]
    for c in range(6):
        t = tr[(tr >= 100 + 10 * c - 0.01) & (tr < 110 + 10 * c - 0.01)] + shift
        ta = np.arange(t[0] - 1.0, t[-1] + 1.0, 0.02)
        if c in drop:
            continue
        path = os.path.join(d, f"raw_{raw_stamp}_{c:03d}.npz")
        np.savez(path, imgs=np.zeros((len(t), 4, 4), np.uint8), t_grab=t,
                 att=np.c_[ta, roll(ta, wig, gust), np.zeros((len(ta), 5))], hgt=np.c_[ta, ta * 0 + 50],
                 lpos=np.c_[ta, ta * 0, ta * 0], K=np.eye(3), D=np.zeros(4))
        raws.append(path)
    return [log] + raws


def timing(files, rate=0.0, n=0):
    """(exit code, lines, frames calib got, settings unchanged, settings after) on a fresh copy of SET."""
    RecSpy.got, RecSpy.rate, RecSpy.n, lines = None, rate, n, []
    s_path = os.path.join(tempfile.mkdtemp(dir=D), "settings.json")
    shutil.copy(SET, s_path)
    r, _ = run(apb_main.mode_timing, S.load(s_path), SimpleNamespace(files=files, settings=s_path), say=lines.append)
    after = S.load(s_path)
    return r, lines, RecSpy.got, open(s_path).read() == open(SET).read(), after


def first(lines):
    return lines[0][:150] if lines else ""


def window_ok(got, lo=109.0, hi=123.9, n=150):
    return got is not None and len(got) == n and np.isclose(got.min(), lo) and np.isclose(got.max(), hi)


calib_Rec, calib.Rec = calib.Rec, RecSpy
ENG, REL = "engaged: FC mode LOITER", "released: FC mode ALT_HOLD"

r, lines, got, same, _ = timing(flight({125.05: ENG}))
check(window_ok(got), "switch on the frame of t_grab 125.05 s (log t 124.98): calib gets the 150 recorded frames "
      f"of t_grab 109.0 to 123.9 s (got {'none' if got is None else f'{len(got)}, {got.min():.2f} to {got.max():.2f}'})")

r, lines, got, same, _ = timing(flight({}, blank=125.05))
check(window_ok(got), "the 'engaged' note on a row with no frame (no t): the last frame's t is the switch")

r, lines, got, same, _ = timing(flight({100.5: ENG, 102.0: REL, 125.05: ENG}, held=[(100.5, 102.0)]))
check(window_ok(got) and any("switch 2 of 2; not used: switch at t 100.4 s: the run log covers only part" in x
                             for x in lines),
      f"B1: a switch test 0.5 s after the start: the flight's switch is used; INFO names the other ({lines[:1]})")

r, lines, got, same, _ = timing(flight({115.0: ENG, 116.0: REL, 135.05: ENG}, wig=122.0, held=[(115.0, 116.0)]))
check(window_ok(got, 119.0, 133.9) and any("under min_height_m 3 m" in x for x in lines),
      f"B1: a switch test 11 s after takeoff (its 15 s reach the ground): the next switch is used ({lines[:1]})")

r, lines, got, same, _ = timing(flight({118.0: ENG, 119.0: REL, 125.05: ENG}, held=[(118.0, 119.0)]))
check(r == 1 and got is None and same and lines and lines[0].startswith("FAIL no switch to position hold after 15 s")
      and "position hold in the 15 s before it" in lines[0],
      f"B1: a switch test in the air 7 s before the switch: FAIL with both reasons ({first(lines)})")

r, lines, got, same, _ = timing(flight({125.05: ENG, 127.0: REL, 150.05: ENG}, held=[(125.05, 127.0), (150.05, 161)]))
check(window_ok(got), "B1: two good switches: the first one (the later one has no wiggle)")

r, lines, got, same, _ = timing(flight({125.05: ENG})[1:])
check(r == 1 and got is None and same and first(lines).startswith("FAIL give one run log"),
      f"raw files and no run log: FAIL, settings not changed ({first(lines)})")

r, lines, got, same, _ = timing(flight({}))
check(r == 1 and got is None and same and first(lines).startswith("FAIL no 'engaged' note"),
      f"a run log with no 'engaged' note: FAIL ({first(lines)})")

r, lines, got, same, _ = timing(flight({125.05: ENG}, shift=400.0))
check(r == 1 and got is None and same and "raw frames have no row in" in first(lines),
      f"S2: raw frames of another run of this boot (t + 400 s): FAIL ({first(lines)})")

r, lines, got, same, _ = timing(flight({125.05: ENG}, shift=0.013))
check(r == 1 and got is None and same and "raw frames have no row in" in first(lines),
      f"S2: raw frames of a run before a reboot, same times + 13 ms, same name stamp: FAIL ({first(lines)})")

r, lines, got, same, _ = timing(flight({125.05: ENG}, raw_stamp="20261120_100002"))
check(r == 1 and got is None and same and first(lines).startswith("FAIL the raw files are not from the run"),
      f"raw file names 2 s after the log's: FAIL ({first(lines)})")
r, lines, got, same, _ = timing(flight({125.05: ENG}, raw_stamp="20261120_100001"))
check(window_ok(got), "raw file names 1 s after the log's (a second boundary between Log and Raw): accepted")

r, lines, got, same, _ = timing(flight({125.05: ENG}, drop=(1,)))
check(r == 1 and got is None and same and first(lines).startswith("FAIL the raw files miss"),
      f"chunk 001 (110 to 120 s) not given: FAIL ({first(lines)})")

r, lines, got, same, _ = timing(flight({125.05: ENG}, wig=None, gust=0.05))
check(r == 1 and got is None and same and any(x.startswith("FAIL no wiggle") for x in lines),
      f"B2: turning all the time (a hold in gusts), no wiggle: FAIL ({lines[-1][:110] if lines else r})")
r, lines, got, same, _ = timing(flight({135.05: ENG}, gust=0.05))
check(r == 1 and got is None and same and any(x.startswith("FAIL no wiggle") for x in lines),
      f"B2: the switch 15 s after the wiggle, turning in gusts: FAIL ({lines[-1][:110] if lines else r})")
r, lines, got, same, _ = timing(flight({125.05: ENG}, gust=0.03))
check(window_ok(got), "B2: the wiggle 5 s before the switch, turning in gusts (3.3 deg/s RMS): accepted")
r, lines, got, same, _ = timing(flight({125.05: ENG}, wig=None))
check(r == 1 and got is None and same and any(x.startswith("FAIL no wiggle") for x in lines),
      f"B2: no turning at all: FAIL ({lines[-1][:110] if lines else r})")

r, lines, got, same, after = timing(flight({125.05: ENG}), rate=30.0, n=2000)
check(r == 0 and after["video_delay_s"] == 0.041 and after["readout_s"] == 0.017,
      f"PASS path: calib finds 41 ms / +17 ms: video_delay_s {after['video_delay_s']}, readout_s {after['readout_s']}")
r, lines, got, same, _ = timing(flight({125.05: ENG}), rate=12.0, n=5000)
check(r == 1 and same and any(x.startswith("FAIL the recording turned 12 deg/s") for x in lines),
      "the wiggle window turned 12 deg/s RMS (under calib_min_rate_dps 20): FAIL, settings not changed")
calib.Rec = calib_Rec


# ---------------------------------------------------------------- B. D2 check: the height stream
print("B. D2 check: the height stream of each firmware")


class FakeFC:
    def __init__(self, fw, rates):
        self.firmware, self.sysid, self.rates, self.rtts = fw, 1, rates, []
        self.clock, self.hgt, self.armed = SimpleNamespace(ready=lambda: True), Series(), False
        self.hgt.add(0.0, 0.0)

    def rate(self, kind, window=2.0):
        return self.rates.get(kind, 0.0)

    def ping_timesync(self):
        pass

    def close(self):
        pass


def no_video(s):
    raise RuntimeError("no camera in this test")


real = apb_main.open_fc, apb_main.open_video, fc_link.check_params
apb_main.open_video, fc_link.check_params = no_video, lambda fc, s, write=False: []
s = S.load(SET)
for fw, rates, name in (("ardupilot", {"ATTITUDE": 50, "GLOBAL_POSITION_INT": 10}, "GLOBAL_POSITION_INT"),
                        ("px4", {"ATTITUDE": 50, "LOCAL_POSITION_NED": 10}, "LOCAL_POSITION_NED")):
    apb_main.open_fc = lambda s, fc=FakeFC(fw, rates): fc
    lines = []
    r, _ = run(apb_main.mode_check, s, SimpleNamespace(set_params=False), say=lines.append)
    h = [x for x in lines if "POSITION" in x]
    check(h == [f"PASS {name} 10 Hz (asked {s['hgt_hz']})"],
          f"{fw}, on the ground with GPS off ({fw} sends {name} only): {h or r}")

asked = {}
stub = SimpleNamespace(s=s, firmware="ardupilot", acks={}, sysid=1, compid=1,
                       _send=lambda fn: fn(SimpleNamespace(command_long_send=lambda *a: asked.update({a[4]: a[5]}))))
fc_link.FC.setup_streams(stub, wait=0.0)
gid = fc_link.M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
check(asked.get(gid) == 1e6 / s["hgt_hz"], f"fc_link asks ArduPilot for GLOBAL_POSITION_INT at hgt_hz "
      f"({asked.get(gid)} us, want {1e6 / s['hgt_hz']:.0f})")


# ---------------------------------------------------------------- C. D3 one FAIL line, not a traceback
print("C. D3 bench, mask, calib: one FAIL line")


def fc_silent(s, say=print):
    raise RuntimeError("no MAVLink heartbeat on udpin:127.0.0.1:14599 in 5 s (test)")


def last_line(out):
    return out.strip().splitlines()[-1] if out.strip() else ""


apb_main.open_video = lambda s: SimpleNamespace(close=lambda: None)
apb_main.open_fc = fc_silent
for mode in ("bench", "mask", "calib"):
    r, out = run(apb_main.main, [mode, "--settings", SET])
    check(r == 1 and last_line(out).startswith("FAIL no MAVLink heartbeat"), f"{mode}, no FC: exit 1, "
          f"'{last_line(out) or r!r}'")


def open_video_bug(s):
    raise KeyError("video_size")


apb_main.open_video = open_video_bug
r, out = run(apb_main.main, ["bench", "--settings", SET])
check(isinstance(r, KeyError), f"bench, a program error (KeyError): the error goes on with its traceback ({r!r})")

t = np.arange(0, 2, 0.1)
rec = os.path.join(D, "rec.npz")
np.savez(rec, imgs=np.zeros((len(t), 288, 360), np.uint8), t_grab=t, att=np.c_[t, np.zeros((len(t), 6))],
         hgt=np.c_[t, t * 0], lpos=np.c_[t, t * 0, t * 0], K=np.eye(3), D=np.zeros(4))
mask = os.path.join(D, "mask_small.png")
cv2.imwrite(mask, np.full((10, 10), 255, np.uint8))
S.save(dict(S.load(SET), mask_file=mask), os.path.join(D, "s_mask.json"))
r, out = run(apb_main.main, ["calib", "--rec", rec, "--settings", os.path.join(D, "s_mask.json")])
check(r == 1 and last_line(out).startswith("FAIL mask is 10x10"), f"calib, a mask of another size (ValueError): "
      f"'{last_line(out) or r!r}'")

lens = os.path.join(D, "lens_cut.json")
open(lens, "w").write(open(S.here(S.load(SET)["lens_file"])).read()[:300])
S.save(dict(S.load(SET), lens_file=lens), os.path.join(D, "s_lens.json"))
r, out = run(apb_main.main, ["calib", "--rec", rec, "--settings", os.path.join(D, "s_lens.json")])
check(r == 1 and last_line(out).startswith(f"FAIL the lens file {lens} is damaged"),
      f"calib, a damaged lens file: the FAIL line names it ('{last_line(out)[:120] or r!r}')")


# ---------------------------------------------------------------- D. D4 free disk
print("D. D4 free disk (Justice D, no_sd): check, the recorder, the minutes")
apb_main.open_fc, apb_main.open_video = fc_silent, no_video
disk_usage = shutil.disk_usage
s = S.load("/nonexistent")
for name in ("justice_d", "no_sd"):
    U.merge(s, U.read(U.overlay_path(name), overlay=True))
s["log_dir"] = D
shape = (s["video_size"][1] // s["video_reduce"], s["video_size"][0] // s["video_reduce"])
Frame = namedtuple("Frame", "img t_grab")


def disk_check(free):
    shutil.disk_usage = lambda p: SimpleNamespace(free=free * 1e9)
    try:
        lines = []
        run(apb_main.mode_check, s, SimpleNamespace(set_params=False), say=lines.append)
        raw = apb_main.Raw(s, SimpleNamespace(att=Series(), hgt=Series(), lpos=Series()), shape)
        run(lambda: (raw.add(Frame(np.zeros(shape, np.uint8), 0.0)), raw.flush()))
        if raw.saver is not None:
            raw.saver.join()
    finally:
        shutil.disk_usage = disk_usage
    disk = [x for x in lines if "free disk" in x]
    return (disk[0] if disk else ""), [x for x in disk if x.startswith("WARN")], raw.off


# no_sd records 640x360 at 10 Hz (2.3 MB/s): 10 min is 1.38 GB above min_free_gb 0.5 GB.
for free, want_pass, want_warn in ((0.3, False, False), (0.47, False, False), (0.6, True, True), (3.0, True, False)):
    line, warn, off = disk_check(free)
    check(line.startswith("PASS" if want_pass else "FAIL") and (line.startswith("FAIL") or not off)
          and "min of raw recording before the recorder stops" in line and bool(warn) == want_warn,
          f"{free} GB free (min_free_gb {s['min_free_gb']:g}): '{line[:110]}', recorder {'off' if off else 'records'}"
          f"{', ' + warn[0][:60] if warn else ''}")
apb_main.open_fc, apb_main.open_video, fc_link.check_params = real

shutil.rmtree(D, ignore_errors=True)
print(f"{'ALL PASSED' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
