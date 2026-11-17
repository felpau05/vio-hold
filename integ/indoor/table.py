"""Results table of every closed-loop run of this build: table.py > integ/indoor/results.txt
drift = true distance from the hold point (RMS / worst, cm); fix err = sent fix minus truth (fixerr.py)."""
import glob
import json
import os

from fixerr import fixerr

HERE = os.path.dirname(os.path.abspath(__file__))
print(f"{'run':24s} {'drift rms':>9s} {'worst':>6s} | {'fix err rms':>11s} {'worst':>6s} | fix/s  ok   proc_ms | sensors seen by PX4 | events")
for p in sorted(glob.glob(os.path.join(HERE, "cl_*.txt")), key=os.path.getmtime):
    lab = os.path.basename(p)[3:-4]
    t = open(p).read()
    i = t.rfind("\n{\n")
    try:
        d = json.loads(t[i + 1:])
    except ValueError:
        print(f"{lab:24s} no result")
        continue
    try:
        f = fixerr(lab)
    except (OSError, KeyError, ValueError):
        f = None
    ev = "; ".join(e[1][:50] for e in d.get("events", [])[1:])
    cm = lambda k: f"{100 * d[k]:6.1f}" if isinstance(d.get(k), float) else "     -"   # noqa: E731
    fe = f"{f['fix_rms_cm']:11.1f} {f['fix_max_cm']:6.1f}" if f else f"{'-':>11s} {'-':>6s}"
    fr = d.get("fix_rate_hz")
    sens = f"hgt {d.get('px4_hgt_err_m')} yaw {d.get('px4_yaw_err_deg')}" if "px4_hgt_err_m" in d else ""
    print(f"{lab:24s} {cm('rms'):>9s} {cm('max')} | {fe} | {fr if fr is None else round(fr, 1)!s:5s} "
          f"{d.get('frac_ok', 0):.2f} {d.get('proc_ms_mean') or 0:5.0f} | {sens} | {ev}")
