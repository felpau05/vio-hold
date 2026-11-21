#!/usr/bin/env python3
"""After each SITL flight of apflights.py: the commands of EVENT_DAY step 7 and EIGHT_HOURS hour 5, on the
flight's own files, with copies of the flight's settings (timing writes settings when it passes).

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/after_flight.py LABEL [--tune]

1. apb_main.py delay on the run log (VISO_DELAY_MS advice).
2. apb_main.py timing on: the chunk(s) that hold the wiggle (by the flight's wiggle_t), the wiggle chunk plus the next
   one (what the file-time rule picks: files saved 0 to 20 s after the wiggle start), and all chunks.
3. --tune: tools/replay_tune.py on all chunks of the flight (as EIGHT_HOURS hour 5 types it).
Truth: video delay 40.0 ms, readout 14.0 ms (px4sim/sitl.py VIDEO_DEFAULTS); the flight flew with 70.0 ms.
Writes after_<LABEL>.txt beside this script.
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
PY = sys.executable


def main():
    label, tune = sys.argv[1], "--tune" in sys.argv
    runs = os.path.join(PROJ, "apsim", "runs")
    raw = sorted(glob.glob(os.path.join(HERE, "runs", label + "_raw", "raw_*.npz")))
    r = json.load(open(os.path.join(HERE, label + ".json")))
    s_flight = os.path.join(runs, label + "_settings.json")
    rep = open(os.path.join(HERE, f"after_{label}.txt"), "w")

    def say(x=""):
        print(x, flush=True)
        rep.write(x + "\n")

    def run(args, settings=None):
        if settings:
            tmp = os.path.join(HERE, "runs", label + "_settings_copy.json")
            shutil.copyfile(settings, tmp)
            args = args + ["--settings", tmp]
        say(f"  load {' '.join(f'{x:.2f}' for x in os.getloadavg())}")
        t1 = time.time()
        p = subprocess.run([PY, *args], cwd=PROJ, capture_output=True, text=True)
        say(f"  exit {p.returncode}, {time.time() - t1:.0f} s")
        for line in (p.stdout + p.stderr).strip().splitlines()[-6:]:
            say("  | " + line)
    w0, w1 = r["wiggle_t"]
    spans = [(f, *np.load(f)["t_grab"][[0, -1]]) for f in raw]
    wig = [f for f, a, b in spans if a < w1 and b > w0]
    nxt = [f for f, a, b in spans if a >= w1][:1]
    say(f"{label}: hold RMS {r.get('rms', float('nan')):.2f} m, max {r.get('max', float('nan')):.2f} m; wiggle {w0:.1f}-{w1:.1f} s; "
        f"{len(raw)} raw chunks; flight settings video_delay_s {json.load(open(s_flight))['video_delay_s']}")
    say("\n[apb_main.py delay <run log>]")
    run(["apb_main.py", "delay", os.path.join(runs, label + "_onboard.csv")], s_flight)
    for name, files in (("wiggle chunk(s) " + " ".join(os.path.basename(f)[-7:-4] for f in wig), wig),
                        ("wiggle chunk(s) + the next one", wig + nxt), ("all chunks", raw)):
        say(f"\n[apb_main.py timing, {name}]")
        run(["apb_main.py", "timing", *files], s_flight)
    if tune:
        say("\n[tools/replay_tune.py, all chunks]")
        out = os.path.join(HERE, "runs", label + "_tune")
        run(["tools/replay_tune.py", os.path.join(HERE, "runs", label + "_raw", "raw_*.npz"), "--jobs", "6", "--out", out],
            s_flight)
    rep.close()


if __name__ == "__main__":
    main()
