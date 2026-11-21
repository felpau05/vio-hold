#!/usr/bin/env python3
"""apb_main run mode end to end: simulator frames (capture.CallableSource) and the fake flight
controller (tests_apb/fake_fc.py, PX4, 80 ms clock offset, 0-20 ms jitter, hover at 50 m).

Phases: normal (fixes arrive and match the truth) -> picture frozen 2 s (fixes stop within 0.5 s,
H4) -> normal -> attitude stops 2 s (fixes stop within 0.2 s, H14) -> normal -> program restart
(starts clean, re-anchors to the FC position: no jump). Then the raw recording goes through replay.
Run: ~/.venvs/vio/bin/python tests_apb/test_run.py      (about 1.5 minutes)
"""
import csv
import glob
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import apb_main            # noqa: E402
import capture             # noqa: E402
import fc_link             # noqa: E402
import settings as S       # noqa: E402
import sim_test            # noqa: E402  (renders the frames)
from fake_fc import scenario   # noqa: E402

PORT, LAT = 14660, 0.060          # true capture delay of the sim frames
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


class SimCamera:
    """Newest 30 Hz frame of the hover, delivered LAT after its first row was exposed."""

    def __init__(self, t_start, name="hover50", span=400.0):
        self.t_start, self.k, self.frozen, self.last, self.span = t_start, -1, False, None, span
        self.pos, self.att, self.h = scenario(name)

    def __call__(self):
        while True:
            k = math.floor((time.monotonic() - LAT - self.t_start) * sim_test.FPS)
            if k > self.k:
                break
            time.sleep(0.002)
        self.k = k
        t0 = self.t_start + k / sim_test.FPS
        if not (self.frozen and self.last is not None):      # frozen: the grabber repeats the last buffer
            self.last = sim_test.render(t0 - self.t_start, self.pos, self.att, self.h, self.span)
        return self.last, t0 + LAT


def main():
    t_begin = time.time()
    d = tempfile.mkdtemp()
    log, pause = os.path.join(d, "fake.jsonl"), os.path.join(d, "pause")
    t_start = time.monotonic()
    fake = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_fc.py"), "--port", str(PORT), "--t-start",
                             repr(t_start), "--log", log, "--scenario", "hover50", "--pause", pause, "--seconds", "120"])
    cv2.imwrite(os.path.join(d, "mask.png"), np.full((sim_test.H, sim_test.W), 255, np.uint8))
    s = dict(S.load("/nonexistent"), fc_device=f"udpin:127.0.0.1:{PORT}", msp_device="/nonexistent",
             calibrated=True, video_delay_s=LAT, field="frame", readout_s=sim_test.READOUT, log_dir=d, raw_record=True,
             mask_file=os.path.join(d, "mask.png"))
    s["calib_capture"] = capture.capture_mode(s, {}, (sim_test.H, sim_test.W))
    cam = SimCamera(t_start)
    src = capture.CallableSource(cam, s)
    fc = apb_main.open_fc(s)
    print(f"connected: {fc.firmware}, ATTITUDE {fc.rate('ATTITUDE', 1.0):.0f} Hz, clock fitted {fc.clock.ready()}")
    try:
        apb_main.run_live(s, src, fc, seconds=1.0, log_path=os.path.join(d, "refused.csv"))
        check(False, "run started with EKF2_GPS_CTRL 7 (the EKF fuses GPS)")
    except RuntimeError as e:
        check("GNSS rule: EKF2_GPS_CTRL 7" in str(e), f"run refuses EKF2_GPS_CTRL 7: {e}")
    check(fc.set_param("EKF2_GPS_CTRL", 0) == 0, "EKF2_GPS_CTRL set to 0, as 'check --set-params' does")
    stop = threading.Event()
    out = {}

    def runner(key):
        out[key] = apb_main.run_live(s, src, fc, stop=stop, log_path=os.path.join(d, f"{key}.csv"))

    th = threading.Thread(target=runner, args=("run1",))
    th.start()
    marks = {}
    for name, secs in (("normal", 8.0), ("frozen", 2.0), ("normal2", 4.0), ("att_stopped", 2.0), ("normal3", 4.0)):
        marks[name] = time.monotonic()
        cam.frozen = name == "frozen"
        if name == "att_stopped":
            open(pause, "w").close()
        elif os.path.exists(pause):
            os.remove(pause)
        time.sleep(secs)
    marks["end1"] = time.monotonic()
    stop.set()
    th.join()
    # Restart: a new pipeline starts clean and must re-anchor to the FC position.
    stop.clear()
    marks["restart"] = time.monotonic()
    th = threading.Thread(target=runner, args=("run2",))
    th.start()
    time.sleep(4.0)
    stop.set()
    th.join()
    marks["end2"] = time.monotonic()
    fc.close()
    fake.terminate()
    fake.wait()

    v = [r for r in map(json.loads, open(log)) if r["kind"] == "vpe"]
    ta = np.array([r["t_arr"] for r in v])
    pos_fn, _, h_fn = scenario("hover50")

    def between(a, b):
        return [r for r, t in zip(v, ta) if a <= t < b]

    def err(rows):
        tt = np.array([r["usec"] * 1e-6 - t_start for r in rows])
        return np.linalg.norm(np.c_[[r["x"] for r in rows], [r["y"] for r in rows]] - pos_fn(tt), axis=1)

    n1 = between(marks["normal"], marks["frozen"])
    e1 = err(n1) if n1 else np.array([np.nan])
    age = np.array([r["t_arr"] - r["usec"] * 1e-6 for r in n1]) * 1e3 if n1 else np.array([np.nan])
    check(len(n1) > 40, f"normal: {len(n1)} fixes in 8 s ({len(n1) / 8:.1f}/s), position error vs truth rms "
                        f"{np.sqrt(np.mean(e1 ** 2)):.2f} m, max {e1.max():.2f} m; fix age {np.median(age):.0f} ms")
    check(np.sqrt(np.mean(e1 ** 2)) < 1.0, "normal: error under 1 m RMS at 50 m (hold anchored to the FC position)")
    r0 = n1[0] if n1 else None
    check(r0 is not None and all(r0["angles_nan"]) and abs(r0["z"] + h_fn(r0["usec"] * 1e-6 - t_start)) < 0.5
          and r0["covariance"][0] > 0 and r0["covariance"][6] > 0 and r0["covariance"][11] == 0.0,
          f"fix fields: z {r0 and r0['z']:.2f}, angles NaN, covariance[0] {r0 and r0['covariance'][0]:.4f} "
          f"[6] {r0 and r0['covariance'][6]:.4f} [11] {r0 and r0['covariance'][11]}")
    last_f = max([t for t in ta if marks["frozen"] <= t < marks["normal2"]], default=None)
    late_f = between(marks["frozen"] + 0.5, marks["normal2"])
    check(not late_f, f"frozen picture: last fix {(last_f - marks['frozen']) if last_f else 0:.2f} s after the freeze; "
                      f"{len(late_f)} fixes later than 0.5 s")
    n2 = between(marks["normal2"], marks["att_stopped"])
    check(len(n2) > 10, f"after the freeze: {len(n2)} fixes in 4 s; first {(min(t for t in ta if t >= marks['normal2']) - marks['normal2']) if n2 else math.nan:.2f} s after the picture moved again")
    last_a = max([t for t in ta if marks["att_stopped"] <= t < marks["normal3"]], default=None)
    late_a = between(marks["att_stopped"] + 0.2, marks["normal3"])
    check(not late_a, f"attitude stopped: last fix {(last_a - marks['att_stopped']) if last_a else 0:.2f} s after; "
                      f"{len(late_a)} fixes later than 0.2 s")
    n3 = between(marks["normal3"], marks["end1"])
    check(len(n3) > 10, f"after attitude returned: {len(n3)} fixes in 4 s")
    r2 = between(marks["restart"], marks["end2"])
    e2 = err(r2) if r2 else np.array([np.nan])
    check(len(r2) > 5 and e2[0] < 1.0, f"restart: {len(r2)} fixes; first fix {e2[0]:.2f} m from the truth (no jump)")
    rows = list(csv.DictReader(open(out["run1"])))
    whys = {}
    for r in rows:
        k = r["why"].split(" ")[0] + (" " + r["why"].split(" ")[1] if len(r["why"].split(" ")) > 1 else "")
        whys[k] = whys.get(k, 0) + 1
    notes = [r["note"] for r in rows if r["note"]]
    check(any("repeated" in r["why"] for r in rows) and any(r["why"].startswith("attitude") for r in rows)
          and notes and "FC position" in notes[0],
          f"log {len(rows)} rows; reasons {dict(sorted(whys.items(), key=lambda x: -x[1])[:8])}; first note '{notes[0] if notes else ''}'")

    raws = sorted(glob.glob(os.path.join(d, "raw_*.npz")))
    t1 = time.time()
    path, counts = apb_main.replay(s, raws[:1], log_path=os.path.join(d, "replay.csv"))
    check(len(raws) >= 2 and counts["sent"] > 50, f"replay of the first raw chunk ({len(raws)} chunks): {counts} "
                                                   f"in {time.time() - t1:.0f} s")
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t_begin:.0f} s (files in {d})")
    return fails


if __name__ == "__main__":
    sys.exit(main())
