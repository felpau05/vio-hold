#!/usr/bin/env python3
"""An existing analog open-loop case, before and after the Justice D change: px4sim/replay.py (re-renders the PAL
camera from a logged closed-loop run and runs Hold with three attitude sources) on apsim/runs/<run>.npz.

"before" = replay.py, camera.py and sitl.py from git HEAD (loaded from a temporary folder), "after" = the files in
px4sim/ now. hold.py is the same file for both. Every fix error must agree exactly.
Run: ~/.venvs/vio/bin/python integ/justice_d/check_replay_same.py [run.npz] [seconds]   (writes check_replay_same.txt)
"""
import contextlib
import importlib
import io
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
RUN = os.path.join(PROJ, "apsim", "runs", "c_d140_100m_w20g40_s11.npz")


def run(folder, path, secs):
    for m in ("camera", "sitl", "replay"):
        sys.modules.pop(m, None)
    sys.path[:0] = [folder, PROJ]
    try:
        replay = importlib.import_module("replay")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            out = replay.main(path, secs)
        return out, buf.getvalue().strip().splitlines()
    finally:
        sys.path.remove(folder)
        sys.path.remove(PROJ)


def main(path=RUN, secs=20.0):
    with tempfile.TemporaryDirectory() as d:
        for f in ("camera.py", "sitl.py", "replay.py"):
            src = subprocess.run(["git", "-C", PROJ, "show", f"HEAD:px4sim/{f}"], capture_output=True, text=True, check=True).stdout
            src = src.replace("HERE = os.path.dirname(os.path.abspath(__file__))", f"HERE = {os.path.join(PROJ, 'px4sim')!r}")
            open(os.path.join(d, f), "w").write(src)
        before, lb = run(d, path, secs)
    after, la = run(os.path.join(PROJ, "px4sim"), path, secs)
    same = all(np.array_equal(before[k], after[k]) for k in before) and before.keys() == after.keys()
    lines = [f"px4sim/replay.py {os.path.relpath(path, PROJ)} {secs:g} s (PAL camera, existing analog open-loop case)"]
    lines += ["before (git HEAD camera.py, sitl.py, replay.py):"] + ["  " + x for x in lb]
    lines += ["after (px4sim/ now):"] + ["  " + x for x in la]
    lines.append("SAME: every fix error agrees exactly" if same else "DIFFERENT")
    print("\n".join(lines))
    open(os.path.join(HERE, "check_replay_same.txt"), "w").write("\n".join(lines) + "\n")
    return 0 if same else 1


if __name__ == "__main__":
    a = sys.argv[1:]
    sys.exit(main(a[0] if a else RUN, float(a[1]) if len(a) > 1 else 20.0))
