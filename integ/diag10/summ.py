#!/usr/bin/env python3
"""Table of replay results (integ/diag10/results/*.json.gz): one line per file, sorted by name.
Columns: split, error RMS of sent fixes (m), max (m), sent fixes per s, longest gap (s), re-anchors, keyframe changes,
'halves disagree' and 'points disagree' refusals. Run: summ.py [substring ...] (all given substrings must match)."""
import glob, gzip, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
keys = sys.argv[1:]
for f in sorted(glob.glob(os.path.join(HERE, "results", "*.json.gz"))):
    b = os.path.basename(f)[:-8]
    if not all(k in b for k in keys):
        continue
    r = json.load(gzip.open(f))
    s = r["summary"]
    rows = r["rows"]
    sent = [x for x in rows if x["sent"]]
    e = np.array([np.hypot(*x["err"]) for x in sent]) if sent else np.zeros(0)
    ts = np.r_[0.0, [x["t"] for x in sent], r["seconds"]]
    halves = sum(1 for x in rows if x["why"].startswith("whole picture: halves"))
    pts = sum(1 for x in rows if x["why"].startswith("whole picture and points"))
    print(f"{b:95s} {r['split'][:5]} rms {s['err_rms']} max {e.max() if len(e) else float('nan'):.2f} fix/s {s['fix_hz']:5.2f} "
          f"gap {np.diff(ts).max():.2f} re {s['reanchors']:2d} kf {s['kf_changes']:3d} halves {halves:3d} points {pts:3d}")
