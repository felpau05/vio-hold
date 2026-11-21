#!/usr/bin/env python3
"""Proof that the --video option of tools/sim_rec.py did not change the default (analog) output.

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/check_sim_rec_analog.py [RUN] [SECONDS]

Runs the committed sim_rec.py (git show HEAD:tools/sim_rec.py) and the working copy on the same run, with no
--video option, then compares settings.json and every array of every chunk. Prints SAME or DIFFERENT per item.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
SIMREC = os.path.join(PROJ, "tools", "sim_rec.py")


def run(src, run_file, out, seconds):
    """sim_rec's main() from this source text, with __file__ in tools/ (it finds the project from there)."""
    code = (f"import sys; sys.argv = ['sim_rec.py', {run_file!r}, {out!r}, '--seconds', {seconds!r}]\n"
            f"exec(compile(open({src!r}).read(), {SIMREC!r}, 'exec'), {{'__name__': '__main__', '__file__': {SIMREC!r}}})")
    subprocess.run([sys.executable, "-c", code], check=True, cwd=PROJ)


def main():
    run_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PROJ, "px4sim", "runs", "apb_100m_w20g40.npz")
    seconds = sys.argv[2] if len(sys.argv) > 2 else "20"
    tmp = tempfile.mkdtemp()
    old_src = os.path.join(tmp, "sim_rec_head.py")
    with open(old_src, "w") as f:
        f.write(subprocess.run(["git", "show", "HEAD:tools/sim_rec.py"], cwd=PROJ, check=True,
                               capture_output=True, text=True).stdout)
    run(old_src, run_file, os.path.join(tmp, "old"), seconds)
    run(SIMREC, run_file, os.path.join(tmp, "new"), seconds)
    bad = 0
    a, b = (json.load(open(os.path.join(tmp, x, "settings.json"))) for x in ("old", "new"))
    print(f"settings.json: {'SAME' if a == b else 'DIFFERENT'}")
    bad += a != b
    names = sorted(f for f in os.listdir(os.path.join(tmp, "old")) if f.endswith(".npz"))
    assert names == sorted(f for f in os.listdir(os.path.join(tmp, "new")) if f.endswith(".npz")), "chunk names differ"
    for n in names:
        x, y = np.load(os.path.join(tmp, "old", n)), np.load(os.path.join(tmp, "new", n))
        same = x.files == y.files and all(x[k].shape == y[k].shape and np.array_equal(x[k], y[k]) for k in x.files)
        print(f"{n}: {'SAME' if same else 'DIFFERENT'} ({len(x['imgs'])} frames, keys {x.files})")
        bad += not same
    print("RESULT: the default analog output is " + ("UNCHANGED" if bad == 0 else "CHANGED"))
    return bad


if __name__ == "__main__":
    sys.exit(main())
