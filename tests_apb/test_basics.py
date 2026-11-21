#!/usr/bin/env python3
"""Small pieces: settings, capture (picture judge, field split, V4L2 queries), the height gate,
the mask mode on the real analog video, and check mode against the fake flight controller.
Run: ~/.venvs/vio/bin/python tests_apb/test_basics.py      (about 40 s)
"""
import json
import os
import subprocess
import sys
import tempfile
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

_REF = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".clip-1")
VIDEO = open(_REF).read().strip() if os.path.exists(_REF) else ""     # local only: the mask part skips without it
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


def settings_part(d):
    print("settings")
    p = os.path.join(d, "s.json")
    s = S.load("/nonexistent")
    S.save(s, p)
    check(S.load(p) == s, f"defaults round-trip ({len(s)} keys)")
    for bad, word in (({"fc_baudrate": 1}, "unknown key"), ({"top": "Front"}, "must be one of"),
                      ({"mirror": 1}, "true or false"), ({"tilt_deg": [1]}, "list of 2"),
                      ({"readout_s": "16ms"}, "a number"), ({"origin_lat_lon_alt": [45.4]}, "lat, lon, alt"),
                      ({"params_ardupilot": {"GPS1_TYPE": 0}}, "GNSS rule"),
                      ({"params_ardupilot": {"EK3_SRC1_POSXY": 3}}, "GNSS rule"),
                      ({"params_ardupilot": {"EK3_SRC2_YAW": 8}}, "GNSS rule"),
                      ({"params_ardupilot": {"AHRS_GPS_USE": 1}}, "GNSS rule"),
                      ({"params_px4": {"EKF2_GPS_CTRL": 7}}, "GNSS rule"),
                      ({"params_ardupilot": {"VISO_TYPE": "1"}}, "must be a number")):
        json.dump(bad, open(p, "w"))
        try:
            S.load(p)
            check(False, f"{bad} accepted")
        except ValueError as e:
            check(word in str(e), f"{bad} refused: {e}")
    json.dump({"readout_s": -0.016, "origin_lat_lon_alt": [45.4, -75.7, 60]}, open(p, "w"))
    check(S.load(p)["readout_s"] == -0.016, "negative readout and an origin are accepted")


def capture_part():
    print("capture")
    s = S.load("/nonexistent")
    rng = np.random.default_rng(0)
    ground = np.clip(cv2.GaussianBlur(rng.normal(128, 60, (288, 360)), (0, 0), 3) * 3 - 256, 0, 255).astype(np.uint8)
    j = capture._Judge(s)
    blue = np.full((288, 360), 29, np.uint8)
    cv2.putText(blue, "NO SIGNAL", (90, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 255, 3)
    noisy = np.clip(ground + rng.normal(0, 3.7, ground.shape), 0, 255).astype(np.uint8)
    res = [j(ground), j(noisy), j(noisy.copy()), j(blue), j(ground, no_signal=True)]
    check(res == [None, None, "repeated frame", "flat picture (no signal?)", "no signal (driver)"],
          f"judge: fresh, fresh+noise, repeat, NO SIGNAL screen, driver flag -> {res}")
    frame = np.zeros((576, 720), np.uint8)
    frame[0::2], frame[1::2] = 10, 200
    top, bot = capture.split_field(frame, "top"), capture.split_field(frame, "bottom")
    check(top.shape == (288, 360) and top.max() == 10 and bot.min() == 200 and
          capture.split_field(frame, "frame").shape == (576, 720) and capture.split_field(frame[:288], "top").shape == (288, 720),
          "field split: PAL 720x576 -> 360x288 per field; 'frame' and single-field buffers pass through")
    check(capture.standard({}, 25.0) == "PAL" and capture.standard({}, 29.97) == "NTSC" and
          capture.standard({"field": "alternate"}, 50.0) == "PAL" and capture.standard({"standard": "NTSC"}, 25) == "NTSC",
          "standard from the driver, else from the frame rate")
    if os.path.exists("/dev/video0"):
        info = capture.v4l2_info("/dev/video0")      # queries only; does not start the camera
        check("width" in info and "field" in info, f"V4L2 queries on this laptop's /dev/video0: {info}")
    check("error" in capture.v4l2_info("/dev/nonexistent"), "missing device reported, no crash")


def gate_part():
    print("pipeline gates")
    s = dict(S.load("/nonexistent"), min_height_m=15.0)
    att, hgt = Series(1e9), Series(1e9)
    for t in np.arange(0, 3, 0.01):
        att.add(t, 0.0, 0.0, 0.0)
    for t in np.arange(0, 3, 0.1):
        hgt.add(t, 4.0)
    pipe = apb_main.Pipe(s, lambda t: None)
    fr = capture.Frame(np.zeros((240, 360), np.uint8), 1.0, 1.06, 1, None)
    e, ok, why = pipe.step(fr, att, hgt)
    check(e is None and not ok and why.startswith("height 4.0 m under 15"), f"low height: {why}")
    e, ok, why = pipe.step(fr._replace(t0=2.99), att, hgt)
    check(not ok and why == "attitude late", f"attitude does not cover the last row: {why}")
    e, ok, why = pipe.step(fr._replace(why="repeated frame"), att, hgt)
    check(not ok and why == "repeated frame", f"bad picture: {why}")


def mask_part(d):
    print("mask mode on the real analog video (DVR, PAL, OSD burned in)")
    if not os.path.exists(VIDEO):
        print("  skipped: no video")
        return
    s = dict(S.load("/nonexistent"), field="top")
    src = capture.FileSource(VIDEO, s)
    imgs = []
    while len(imgs) < 250 and (fr := src.newest()) is not None:
        if fr.why is None:
            imgs.append(fr.img)
    rec = os.path.join(d, "rec.npz")
    np.savez(rec, imgs=np.array(imgs), t_grab=np.arange(len(imgs)) / 25.0)
    sp = os.path.join(d, "s.json")
    S.save(dict(s, mask_file=os.path.join(d, "mask.png")), sp)
    lines = []
    rc = apb_main.mode_mask(S.load(sp), SimpleNamespace(rec=rec, seconds=None, settings=sp), say=lines.append)
    m = cv2.imread(os.path.join(d, "mask.png"), cv2.IMREAD_GRAYSCALE)
    ref = cv2.imread(os.path.join(PROJ, "real", "mask_clip-1.png"), cv2.IMREAD_GRAYSCALE)
    if m is not None and ref is not None:
        r = cv2.resize(ref, (m.shape[1], m.shape[0]), interpolation=cv2.INTER_NEAREST) > 0   # True = not ground
        a = m == 0
        iou = (a & r).sum() / max((a | r).sum(), 1)
        check(rc == 0 and 0.08 < a.mean() < 0.5, f"{lines[-1]}; {m.shape[1]}x{m.shape[0]} field; overlap with the "
                                                 f"colour reference real/mask_clip-1.png IoU {iou:.2f} "
                                                 f"(reference covers {r.mean():.0%})")
    else:
        check(False, f"mask not written: {lines}")
    still = np.repeat(np.array(imgs[:1]), 60, axis=0)
    np.savez(rec, imgs=still, t_grab=np.arange(60) / 25.0)
    lines = []
    rc = apb_main.mode_mask(S.load(sp), SimpleNamespace(rec=rec, seconds=None, settings=sp), say=lines.append)
    check(rc == 1 and "did not move" in lines[-1], f"a still picture is refused: {lines[-1]}")


def check_mode_part(d):
    print("check mode against the fake PX4 (no video device here)")
    log = os.path.join(d, "fake.jsonl")
    t_start = time.monotonic()
    fake = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_fc.py"), "--port", "14670", "--t-start",
                             repr(t_start), "--log", log, "--seconds", "60"])
    s = dict(S.load("/nonexistent"), fc_device="udpin:127.0.0.1:14670", msp_device="/nonexistent",
             video_device="/dev/nonexistent", log_dir=d)
    lines = []
    try:
        n_fail = apb_main.mode_check(s, SimpleNamespace(set_params=False), say=lines.append)
        lines2 = []
        apb_main.mode_check(s, SimpleNamespace(set_params=True), say=lines2.append)
        lines3 = []
        apb_main.mode_check(s, SimpleNamespace(set_params=False), say=lines3.append)
    finally:
        fake.terminate()
        fake.wait()
    for x in lines:
        print("    | " + x)
    has = lambda ls, a: any(x.startswith(a) for x in ls)       # noqa: E731
    check(has(lines, "PASS firmware: px4") and has(lines, "PASS ATTITUDE") and has(lines, "FAIL param EKF2_EV_CTRL")
          and has(lines, "FAIL video") and has(lines, "FAIL no mask") and has(lines, "PASS TIMESYNC")
          and any("Hold.process" in x for x in lines) and n_fail >= 6,
          f"first check: {n_fail} FAIL lines, one line per check")
    check(not any(x.startswith("FAIL param") for x in lines3) and any("wrote" in x for x in lines2),
          "after --set-params every parameter passes")


def main():
    t0 = time.time()
    with tempfile.TemporaryDirectory() as d:
        settings_part(d)
        capture_part()
        gate_part()
        mask_part(d)
        check_mode_part(d)
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
