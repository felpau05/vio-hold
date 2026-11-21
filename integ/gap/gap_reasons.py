#!/usr/bin/env python3
"""Q1: which check stops the fixes in the longest gap around the hold start of each closed-loop run.

Gap: as integ/justice_d/fix_error.py (longest time between two sent fixes, both in [-10, +3] s of the hold start).
For every onboard log row inside the gap: the reason (why, first words) and the Est state. Also the height and the
true motion in the gap (truth from the run's npz), to separate the scenario (climb, height gate, video start) from the
checks. Reads apsim/runs/<label>_onboard.csv, .npz, .json only.
Run: ~/.venvs/vio/bin/python integ/gap/gap_reasons.py > integ/gap/results/gap_reasons.txt
"""
import csv
import json
import math
import os
from collections import Counter

import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.join(PROJ, "apsim", "runs")
LABELS = ["an_apb_100m_w20g40_s11", "an_apb_20m_calm_s11", "an_apb_50m_w20g40_s11", "jd_apb_100m_w20g40_s11",
          "jd_apb_100m_w20g40_s12", "jd_apb_100m_w20g40_s13", "jd_apb_20m_calm_s11", "jd_apb_50m_w20g40_s11",
          "jdcal2_apb_100m_w20g40_s11", "jdcal3_apb_100m_w20g40_s11", "jdcal_apb_100m_w20g40_s11",
          "jdfull_apb_100m_w20g40_s11", "c_d140_100m_w20g40_s11", "c_d140_100m_w20g40_s12", "c_d140_100m_w20g40_s13"]


def main():
    for lab in LABELS:
        d = np.load(os.path.join(RUNS, lab + ".npz"))
        r = json.load(open(os.path.join(RUNS, lab + ".json")))
        tr, te = d["truth"], float(d["t_engage"])
        rows = [x for x in csv.DictReader(open(os.path.join(RUNS, lab + "_onboard.csv"))) if x["t"]]
        ts = np.array([float(x["t"]) for x in rows if x["sent"] == "1"])
        pre = ts[(ts > te - 10) & (ts < te + 3)]
        g = np.diff(pre)
        k = int(np.argmax(g))
        lo, hi = pre[k], pre[k + 1]
        inside = [x for x in rows if lo < float(x["t"]) < hi]
        why = Counter(x["why"].split(" (")[0] for x in inside)
        state = Counter(x["state"] or "-" for x in inside)
        # all failed rows in [-10, +3] s, not only in the longest gap
        win = [x for x in rows if te - 10 < float(x["t"]) < te + 3 and x["sent"] != "1"]
        why_win = Counter(x["why"].split(" (")[0] for x in win)
        hs = [float(x["height"]) for x in inside if x["height"]]
        m = (tr[:, 0] >= lo) & (tr[:, 0] <= hi)
        hor = float(np.hypot(*(tr[m, 1:3].max(0) - tr[m, 1:3].min(0)))) if m.sum() > 1 else math.nan
        t_climb_end = None
        print(f"{lab}: gap {g[k]:.2f} s, from {lo - te:+.2f} to {hi - te:+.2f} s of the hold start "
              f"({len(inside)} processed frames; climb_s {r.get('climb_s')}, vib_deg {r['cfg'].get('vib_deg')}, "
              f"still_start {r['cfg'].get('still_start')}, wiggle_s {r['cfg'].get('wiggle_s')})")
        print(f"   reasons in the gap: {dict(why.most_common())}")
        print(f"   states in the gap:  {dict(state)}")
        if hs:
            print(f"   height in the gap: {min(hs):.2f} to {max(hs):.2f} m (min_height_m 15 for these runs); "
                  f"true horizontal span in the gap {hor:.2f} m")
        print(f"   all unsent rows in [-10, +3] s: {dict(why_win.most_common())}")
        print(f"   hold-start events: {[e[1] for e in r.get('events', [])][:3]}")


if __name__ == "__main__":
    main()
