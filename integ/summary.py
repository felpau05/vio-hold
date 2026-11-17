"""Headline numbers from px4sim/results.json: per height and wind, the onboard program (apb, medium
vehicle, ottawa2019fall) over every seed, against the truth feed; the highest height that holds within
1 m RMS and 2 m max in every windy run. Run: ~/.venvs/vio/bin/python integ/summary.py > integ/summary.txt"""
import json
import os
import re

R = json.load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "px4sim", "results.json")))
RMS_LIM, MAX_LIM = 1.0, 2.0          # our own placeholders until the real limits arrive
rows = {}
for k, r in R.items():
    m = re.fullmatch(r"(apb|truth)_(\d+)m_(calm|w20g40|w30g55)(_seed\d)?", k)
    if m and isinstance(r.get("rms"), float):
        rows.setdefault((m[1], int(m[2]), m[3]), []).append((k, r["rms"], r["max"], r.get("pct_frames_fix")))
print(f"{'variant':6s} {'height':>6s} {'wind':7s} runs  RMS m (each seed)         max m (each seed)        holds 1 m / 2 m")
best = {}
for (v, h, w) in sorted(rows, key=lambda x: (x[0], x[2], x[1])):
    x = rows[(v, h, w)]
    ok = all(a <= RMS_LIM and b <= MAX_LIM for _, a, b, _ in x)
    print(f"{v:6s} {h:6d} {w:7s} {len(x):4d}  {' '.join(f'{a:.2f}' for _, a, _, _ in x):26s} "
          f"{' '.join(f'{b:.2f}' for _, _, b, _ in x):24s} {'yes' if ok else 'NO'}")
    if v == "apb" and w != "calm":
        best.setdefault(w, []).append((h, ok))
for w, hs in best.items():
    good = [h for h, ok in sorted(hs)]
    top = max([h for h, ok in hs if ok and all(ok2 for h2, ok2 in hs if h2 <= h)], default=None)
    print(f"{w}: highest tested height where every run (and every lower height) holds within "
          f"{RMS_LIM:g} m RMS and {MAX_LIM:g} m max: {top} m (tested {good})")
