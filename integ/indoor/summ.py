"""One line per closed-loop run: summ.py integ/indoor/cl_*.txt  (RMS and worst drift in cm)."""
import json
import sys

for path in sys.argv[1:]:
    t = open(path).read()
    i = t.rfind("\n{\n")
    try:
        d = json.loads(t[i + 1:])
    except ValueError:
        print(f"{path.split('cl_')[-1][:-4]:28s} NO RESULT: {t.strip().splitlines()[-1][:120] if t.strip() else 'empty'}")
        continue
    ev = [e[1][:60] for e in d.get("events", [])[1:]]
    why = ", ".join(f"{k[:34]} {v}" for k, v in list(d.get("hold_why", {}).items())[:3])
    g = lambda k, f="{:.0f}", m=1: f.format(d[k] * m) if isinstance(d.get(k), (int, float)) else "-"   # noqa: E731
    extra = ""
    if "px4_hgt_err_m" in d:
        extra = f" hgtErr {d['px4_hgt_err_m']} yawErr {d.get('px4_yaw_err_deg')}"
    print(f"{path.split('cl_')[-1][:-4]:28s} rms {g('rms', '{:6.1f}', 100)} max {g('max', '{:6.1f}', 100)} "
          f"fix/s {g('fix_rate_hz', '{:5.1f}')} ok {g('frac_ok', '{:.2f}')} proc {g('proc_ms_mean')} age {g('age_ms_p50')}"
          f"{extra} | {'; '.join(ev)} | {why}")
