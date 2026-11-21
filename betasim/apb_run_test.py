#!/usr/bin/env python3
"""apb_main with msp_link against the Betaflight SITL, on a patched copy of the project (APB_DIR).

1. apb_main.py check: fc_link.connect finds no MAVLink, finds Betaflight on MSP and returns an MspLink.
2. A short run_live (12 s): simulator frames at 50 m (the hover50 scenario of tests_apb), bench height 50 m
   (the SITL sits on the ground, disarmed). Counts the fixes, the flow messages and the FC acknowledges.
The FDM feed of the firmware's harness (BF_SRC) keeps the SITL's IMU and barometer alive; the model stays on
the ground. Out: betasim/results/apb_run.txt.
Run: APB_DIR=<patched copy> BF_SITL=<binary> BF_SRC=<clone> ~/.venvs/vio/bin/python betasim/apb_run_test.py
"""
import argparse
import importlib.util
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
APB = os.environ["APB_DIR"]
sys.path.insert(0, APB)
sys.path.insert(0, os.path.join(APB, "tests_apb"))
sys.path.insert(0, HERE)
import sitl                 # noqa: E402
import apb_main             # noqa: E402
import capture              # noqa: E402
import settings as S        # noqa: E402
import sim_test             # noqa: E402
from fake_fc import scenario   # noqa: E402

spec = importlib.util.spec_from_file_location("bf_harness", os.path.join(os.environ["BF_SRC"], "src/test/sitl/sitl_harness.py"))
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)
H.TELEMETRY_PORT = 0
LAT = 0.06
lines = []


def say(t):
    print(t, flush=True)
    lines.append(t)


class Cam:
    def __init__(self, t0):
        self.t0, self.k = t0, -1
        self.pos, self.att, self.h = scenario("hover50")

    def __call__(self):
        while (k := int((time.monotonic() - LAT - self.t0) * sim_test.FPS)) <= self.k:
            time.sleep(0.002)
        self.k = k
        t = self.t0 + k / sim_test.FPS
        return sim_test.render(t - self.t0, self.pos, self.att, self.h, 400.0), t + LAT


wd = os.path.join(os.environ.get("BF_WORK", "/tmp/betasim_work"), "apbrun")
sitl.provision(wd, ["feature -GPS", "feature RANGEFINDER", "feature OPTICALFLOW", "set rangefinder_hardware = MTF02P",
                    "set opticalflow_hardware = MT", "set debug_mode = BARO"])
proc = sitl.start(wd)
motors = H.MotorFeed()
fdm = H.FdmFeed(motors)
motors.start()
fdm.start()
try:
    prof = {k: v for k, v in json.load(open(os.path.join(APB, "betasim/profiles/betaflight.json"))).items() if k != "_why"}
    s = dict(S.load("/nonexistent"), **prof)
    cv2.imwrite(os.path.join(wd, "mask.png"), np.full((sim_test.H, sim_test.W), 255, np.uint8))
    s.update(fc_device="udpin:127.0.0.1:14999", msp_device=f"tcp:127.0.0.1:{sitl.MSP_PORT}", calibrated=True,
             video_delay_s=LAT, field="frame", readout_s=sim_test.READOUT, log_dir=wd, mask_file=os.path.join(wd, "mask.png"))
    s["calib_capture"] = capture.capture_mode(s, {}, (sim_test.H, sim_test.W))
    say("== apb_main check (FC part; video and CPU lines need the APB)")
    t = time.monotonic()
    apb_main.mode_check(s, argparse.Namespace(set_params=False), say=say)
    say(f"(check took {time.monotonic() - t:.0f} s)")
    say("== run_live 12 s, bench height 50 m")
    fc = apb_main.open_fc(s, say=say)
    calls = {"send_fix": 0, "send_speed": 0}
    for name in calls:
        f = getattr(fc, name)

        def wrap(*a, _f=f, _n=name, **k):
            calls[_n] += 1
            return _f(*a, **k)
        setattr(fc, name, wrap)
    src = capture.CallableSource(Cam(time.monotonic()), s)
    log = apb_main.run_live(s, src, fc, seconds=12, log_path=os.path.join(wd, "run.csv"), fixed_height=50.0)
    rows = list(__import__("csv").DictReader(open(log)))
    sent = sum(r["sent"] == "1" for r in rows)
    reasons = {}
    for r in rows:
        if r["sent"] != "1":
            reasons[r["why"]] = reasons.get(r["why"], 0) + 1
    say(f"frames {len(rows)}, fixes sent {sent}, send_fix calls {calls['send_fix']}, send_speed calls {calls['send_speed']}, "
        f"flow messages {len(fc.sent)}, FC acknowledges {fc.acks}, error replies {fc.errors}")
    say(f"not sent, by reason: {reasons}")
    say(f"modes logged: {sorted(set(r['mode'] for r in rows))}; attitude {fc.rate('ATTITUDE'):.0f} Hz, "
        f"height {fc.rate('ALTITUDE'):.0f} Hz")
    fc.close()
finally:
    fdm.shutdown()
    motors.shutdown()
    sitl.stop(proc)
    with open(os.path.join(HERE, "results", "apb_run.txt"), "w") as f:
        f.write(f"betasim/apb_run_test.py, {time.strftime('%Y-%m-%d %H:%M')}, copy {APB}\n" + "\n".join(lines) + "\n")
