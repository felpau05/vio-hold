#!/usr/bin/env python3
"""Table of closed-loop runs: RMS and max hold error, RMS true speed, and the main period of the north error
(a sign of oscillation). Usage: summarize.py OUT.txt closed_TAG.json ... (paths under betasim/results/)."""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
rows = []
for name in sys.argv[2:]:
    d = json.load(open(os.path.join(HERE, "results", name)))
    lg = d["log"]
    t = np.array([r["t"] for r in lg])
    e = np.array([[r["dn"], r["de"]] for r in lg])
    v = np.array([[r["vn"], r["ve"]] for r in lg])
    err = np.hypot(*e.T)
    x = e[:, 0] - e[:, 0].mean()
    f = np.fft.rfftfreq(len(x), np.median(np.diff(t)))
    p = np.abs(np.fft.rfft(x)) ** 2
    k = 1 + np.argmax(p[1:])
    rows.append(f"{name[7:-5]:14s} RMS {np.sqrt((err ** 2).mean()):.2f} m  max {err.max():.2f} m  "
                f"speed RMS {np.sqrt((v ** 2).sum(1).mean()):.2f} m/s  north error main period {1 / f[k]:.1f} s  "
                f"samples {len(lg)}, {t[-1]:.0f} s")
open(os.path.join(HERE, "results", sys.argv[1]), "w").write("\n".join(rows) + "\n")
print("\n".join(rows))
