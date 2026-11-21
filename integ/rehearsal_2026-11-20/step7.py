#!/usr/bin/env python3
"""Step 7 of the rehearsal: fly, record, replay, tune with Justice D recordings (tools/sim_rec.py --video JUSTICE_D).

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/step7.py HEIGHT [HEIGHT ...]     (10 30 60 100)

For each height: runs/jd_<h>m/ holds the recording (make_recs.sh) and settings.json with the true values.
1. settings_true.json = that file plus the outdoor_day values (min_height_m 3, hold_cfg h_min 2), as on the day.
2. settings_wrong.json = settings_true.json with video_delay_s 30 ms too long (the wrong value, set on purpose).
3. replay_tune.py on all chunks with settings_wrong.json (as EIGHT_HOURS hour 5), output in jd_<h>m/tune/.
4. apb_main.py timing on all chunks, with a copy of settings_wrong.json (timing writes settings when it passes).
5. Accuracy against the truth (truth_ne in each chunk): apb_main.replay with the true settings, the wrong settings
   and the wrong settings plus the tune patch. Error = fix minus truth, after the median offset is taken off
   (the re-anchor to the FC position sets an arbitrary offset). RMS and p95 of the horizontal error, over sent fixes.
Writes step7_<h>m.txt beside this script. The load average is printed before each part.
"""
import copy
import glob
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, PROJ)
import settings as S        # noqa: E402

PY = sys.executable
WRONG_S = 0.030


def load1():
    return os.getloadavg()[0]


def wait_load(say):
    while load1() > 4.0:
        say(f"  load {load1():.2f} > 4: wait 30 s")
        time.sleep(30)
    say(f"  load average {' '.join(f'{x:.2f}' for x in os.getloadavg())}")


def accuracy(s, files, log):
    import apb_main
    apb_main.replay(s, files, log_path=log)
    import csv
    rows = [r for r in csv.DictReader(open(log)) if r["sent"] == "1"]
    tr = np.concatenate([np.load(f)["truth_ne"] for f in files])
    t = np.array([float(r["t"]) for r in rows])
    est = np.array([[float(r["north"]), float(r["east"])] for r in rows])
    if len(rows) < 10:
        return f"{len(rows)} sent fixes: too few"
    true = np.c_[np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])]
    e = est - true
    e -= np.median(e, axis=0)
    d = np.hypot(e[:, 0], e[:, 1])
    return (f"{len(rows)} sent fixes of {sum(1 for _ in open(log)) - 1} frames, error RMS {np.sqrt(np.mean(d ** 2)):.3f} m, "
            f"p95 {np.percentile(d, 95):.3f} m")


def one(h):
    out = os.path.join(HERE, "runs", f"jd_{h}m")
    rep = open(os.path.join(HERE, f"step7_{h}m.txt"), "w")

    def say(x=""):
        print(x, flush=True)
        rep.write(x + "\n")
        rep.flush()
    files = sorted(glob.glob(os.path.join(out, "raw_sim_*.npz")))
    s_true = S.load(os.path.join(out, "settings.json"))
    s_true["min_height_m"] = 3.0
    s_true["hold_cfg"] = dict(s_true["hold_cfg"], h_min=2.0)
    S.save(s_true, os.path.join(out, "settings_true.json"))
    s_wrong = copy.deepcopy(s_true)
    s_wrong["video_delay_s"] = round(s_true["video_delay_s"] + WRONG_S, 4)
    S.save(s_wrong, os.path.join(out, "settings_wrong.json"))
    say(f"height {h} m, recording {out} ({len(files)} chunks)")
    say(f"true: video_delay_s {s_true['video_delay_s']}, readout_s {s_true['readout_s']}, focal_scale 1.0 "
        f"(lens_measured = the calib-like lens of the sim)")
    say(f"wrong on purpose: video_delay_s {s_wrong['video_delay_s']} (+{WRONG_S * 1e3:.0f} ms)")

    say("\n[replay_tune.py]")
    wait_load(say)
    t1 = time.time()
    r = subprocess.run([PY, "tools/replay_tune.py", os.path.join(out, "raw_sim_*.npz"), "--settings",
                        os.path.join(out, "settings_wrong.json"), "--jobs", "6", "--out", os.path.join(out, "tune")],
                       cwd=PROJ, capture_output=True, text=True)
    say(f"  exit {r.returncode}, wall {time.time() - t1:.0f} s")
    say("  " + "\n  ".join((r.stdout + r.stderr).strip().splitlines()[-4:]))
    patch = json.load(open(os.path.join(out, "tune", "tune_patch.json")))

    say("\n[apb_main.py timing, all chunks]")
    wait_load(say)
    tw = os.path.join(out, "settings_timing.json")
    S.save(s_wrong, tw)
    t1 = time.time()
    r = subprocess.run([PY, "apb_main.py", "timing", *files, "--settings", tw], cwd=PROJ, capture_output=True, text=True)
    say(f"  exit {r.returncode}, wall {time.time() - t1:.0f} s")
    say("  " + "\n  ".join((r.stdout + r.stderr).strip().splitlines()[-4:]))

    say("\n[accuracy against the truth: replay of the same recording]")
    wait_load(say)
    s_patch = copy.deepcopy(s_wrong)
    for k, v in patch.items():
        if k == "hold_cfg":
            s_patch["hold_cfg"].update(v)
        elif not k.startswith("_"):
            s_patch[k] = v
    for i, (name, s) in enumerate((("true settings", s_true), ("wrong settings", s_wrong), ("wrong + tune patch", s_patch))):
        say(f"  {name:20s} {accuracy(s, files, os.path.join(out, f'acc_{i}.csv'))}")
    say(f"  patch: {json.dumps({k: v for k, v in patch.items() if not k.startswith('_')})}")
    rep.close()


if __name__ == "__main__":
    for h in sys.argv[1:]:
        one(h)
