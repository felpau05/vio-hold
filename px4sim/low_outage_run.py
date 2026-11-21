#!/usr/bin/env python3
"""Low-height holds (5 m, 10 m) and the 3 s video outage with the digital Justice D, PX4 SITL.

Each case goes through sitl.run_case. Results merge into px4sim/low_outage_results.json; the table is
px4sim/low_outage_results.txt. px4sim/results.json and results.txt stay as they are.

Run:  ~/.venvs/vio/bin/python px4sim/low_outage_run.py run <label> [<label> ...]
      ~/.venvs/vio/bin/python px4sim/low_outage_run.py table
Before each run: wait until the 1-minute load is under LOAD_MAX and no px4_sim container runs
(docker_start removes px4_sim, so a run must not start over another run's PX4 run).
A run does not start after DEADLINE (local time HH:MM).
"""
import csv
import glob
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import traceback

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path[:0] = [PROJ, HERE]
RESULTS = os.path.join(HERE, "low_outage_results.json")
TABLE = os.path.join(HERE, "low_outage_results.txt")
RUNS = os.path.join(HERE, "runs")
LOAD_MAX = 4.0          # as integ/justice_d/run_ap.py (16 cores)
DEADLINE = "06:05"
LOW = dict(settings={"min_height_m": 3.0}, hold_cfg={"h_min": 2.0})   # fixes from 3 m up; Hold's floor 2 m
OUT3 = dict(outage=(20.0, 3.0))


def jd(var, h, wind, seed, extra="", **kw):
    label = f"lowout_{var}_jd_{h}m_{wind}" + (f"_{extra}" if extra else "") + f"_s{seed}"
    return label, dict(variant=var, height=float(h), wind=wind, hold_s=60.0, seed=seed, video="JUSTICE_D",
                       label=label, **kw)


CASES = dict([
    jd("apb", 5, "w20g40", 1, **LOW), jd("apb", 5, "w20g40", 2, **LOW),
    jd("apb", 10, "w20g40", 1, **LOW), jd("apb", 10, "w20g40", 2, **LOW),
    jd("apb", 60, "w20g40", 1, "out3s", **OUT3), jd("apb", 60, "w20g40", 2, "out3s", **OUT3),
    jd("truth", 5, "w20g40", 1), jd("truth", 10, "w20g40", 1),
    jd("apb", 5, "calm", 1, **LOW), jd("apb", 10, "calm", 1, **LOW),
    jd("apb", 100, "w20g40", 1, "out3s", **OUT3),
])
# Note: lowout_apb_jd_5m_w20g40_s1 started 03:10:20 while another run changed settings.py (03:10:23),
# apb_main.py (03:11:45) and hold.py (03:12:32); the other runs used the later versions (md5 in the results).


def md5(p):
    return hashlib.md5(open(os.path.join(PROJ, p), "rb").read()).hexdigest()[:10]


def load():
    try:
        return json.load(open(RESULTS))
    except (OSError, ValueError):
        return {}


def px4_busy():
    r = subprocess.run(["docker", "ps", "-q", "-f", "name=^px4_sim$"], capture_output=True, text=True)
    return bool(r.stdout.strip())


def wait_ready(say=print, max_wait_s=480):
    t0 = time.time()
    while True:
        one = os.getloadavg()[0]
        busy = px4_busy()
        if one < LOAD_MAX and not busy:
            return os.getloadavg(), round(time.time() - t0)
        if time.time() - t0 > max_wait_s:
            return os.getloadavg(), -round(time.time() - t0)     # negative: started although not ready
        say(f"wait: load {one:.2f} (max {LOAD_MAX}), px4_sim running {busy}")
        time.sleep(30)


def outage_metrics(r, c):
    """Time to the first sent fix after the outage end, drift during and after the outage, restart lines."""
    d = np.load(os.path.join(RUNS, c["label"] + ".npz"))
    te, hp = float(d["t_engage"]), d["hold_pt"]
    tr = d["truth"]
    t, dist = tr[:, 0] - te, np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    t_a, t_b = c["outage"][0], c["outage"][0] + c["outage"][1]
    at = lambda x: float(np.interp(x, t, dist))                                     # noqa: E731
    pos = lambda x: np.array([np.interp(x, t, tr[:, 1]), np.interp(x, t, tr[:, 2])])   # noqa: E731
    mx = lambda a, b: float(dist[(t >= a) & (t <= b)].max())                         # noqa: E731
    rows = []
    for p in sorted(glob.glob(os.path.join(RUNS, c["label"] + "_onboard*.csv"))):
        for x in csv.DictReader(open(p)):
            if x["t"]:
                x["file"] = os.path.basename(p)
                rows.append(x)
    sent = sorted((float(x["t"]) - te + float(x["age_ms"] or 0) / 1e3, x["file"]) for x in rows if x["sent"] == "1")
    before = [s for s in sent if s[0] < t_b]       # a frame stamped just before t_a is sent just after it
    after = [s for s in sent if s[0] >= t_b]
    first_frame = min((float(x["t"]) - te for x in rows if float(x["t"]) - te >= t_b), default=None)
    m = dict(dist_at_outage_start=at(t_a), dist_at_outage_end=at(t_b),
             move_during_outage=float(np.linalg.norm(pos(t_b) - pos(t_a))),
             max_dist_before=mx(0, t_a), max_dist_during=mx(t_a, t_b),
             max_dist_10s_after=mx(t_b, t_b + 10), max_dist_rest=mx(t_b + 10, t[-1]),
             last_fix_before=before[-1][0] if before else None,
             first_frame_after=first_frame,
             first_fix_after=after[0][0] if after else None,
             first_fix_after_file=after[0][1] if after else None,
             s_to_first_fix_after_outage_end=(after[0][0] - t_b) if after else None,
             no_fix_gap_s=(after[0][0] - before[-1][0]) if (after and before) else None)
    if after:
        m["dist_at_first_fix"] = at(after[0][0])
    m["restart_lines"] = [(round(tt - te, 2), k, x) for tt, k, x in r.get("onboard", []) if k in ("run", "hang", "error", "slow")]
    first_after = sorted((float(x["t"]) - te, x["file"], x["state"], x["why"], x["note"]) for x in rows
                         if float(x["t"]) - te >= t_b - 0.2)[:3]
    m["first_rows_after"] = [(round(a, 3), f, s, w[:60], n[:80]) for a, f, s, w, n in first_after]
    notes = sorted((float(x["t"]) - te, x["file"], x["note"]) for x in rows
                   if x["note"].startswith("re-anchored") and t_a - 1 <= float(x["t"]) - te <= t_b + 5)
    m["reanchor_rows"] = [(round(a, 3), f, n) for a, f, n in notes]
    return m


def run(label):
    import sitl
    c = CASES[label]
    if os.path.exists(os.path.join(RUNS, label + ".json")) and label in load():
        print(f"{label}: result exists, skipped", flush=True)
        return
    if time.strftime("%H:%M") >= DEADLINE:
        print(f"{label}: not started, past {DEADLINE}", flush=True)
        return
    la, waited = wait_ready()
    info = dict(load_at_start=[round(x, 2) for x in la], waited_s=waited,
                started=time.strftime("%Y-%m-%d %H:%M:%S"),
                md5={p: md5(p) for p in ("hold.py", "apb_main.py", "px4sim/sitl.py", "px4sim/camera.py",
                                         "px4sim/onboard.py", "capture.py", "fc_link.py", "settings.py")})
    t0 = time.time()
    try:
        r = sitl.run_case(dict(c))
    except Exception as e:
        r = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")], trace=traceback.format_exc()[-3000:])
    r = {k: v for k, v in r.items() if k != "cfg"}
    r.update(info, load_at_end=[round(x, 2) for x in os.getloadavg()], wall_s=round(time.time() - t0, 1),
             case_args=c)
    if c.get("outage") and "rms" in r:
        try:
            r["outage_metrics"] = outage_metrics(r, c)
        except Exception as e:
            r["outage_metrics"] = f"ERROR {type(e).__name__}: {e}"
    res = load()
    res[label] = r
    json.dump(res, open(RESULTS, "w"), indent=1, default=str)
    print(f"DONE {label} rms {r.get('rms')} max {r.get('max')} pct_fix {r.get('pct_frames_fix')} "
          f"load {r['load_at_start']} wall {r['wall_s']} s events {r.get('events')}", flush=True)
    table()


def table():
    res = load()
    f = lambda x, n=3: f"{x:.{n}g}" if isinstance(x, (int, float)) else "-"   # noqa: E731
    lines = ["run name                                 var    h_m  wind    outage  rms_m  max_m  %fix   fixHz  "
             "load1/5/15 start   events"]
    for k in CASES:
        if k not in res:
            continue
        r, c = res[k], CASES[k]
        ev = "; ".join(f"{t:g}s {w}" for t, w in r.get("events", [])[1:])[:150] or "-"
        if r.get("params_fail"):
            ev += f"; params_fail {r['params_fail']}"
        la = "/".join(f"{x:.1f}" for x in r["load_at_start"])
        out = "20+3s" if c.get("outage") else "-"
        lines.append(f"{k:40s} {c['variant']:5s} {c['height']:5.0f}  {c['wind']:6s}  {out:6s} {f(r.get('rms')):6s} "
                     f"{f(r.get('max')):6s} {f(r.get('pct_frames_fix')):6s} {f(r.get('fix_rate_hz')):6s} "
                     f"{la:17s}  {ev}")
    for k in CASES:
        m = res.get(k, {}).get("outage_metrics")
        if m:
            lines.append("")
            lines.append(f"{k} outage (20 s to 23 s after the hold start):")
            lines += [f"  {a}: {b}" for a, b in m.items()]
    lines.append("")
    lines.append("rms/max: true horizontal distance from the hold point over the 60 s hold (sitl.score). % fix: fixes")
    lines.append("sent per rendered frame (60 fps). apb: onboard program, Justice D 1280x720 at 60 fps given to Hold as")
    lines.append("640x360, cpu_factor 10. Low-height apb runs: settings min_height_m 3, hold_cfg h_min 2. Outage runs:")
    lines.append("settings.py defaults (min_height_m 15). truth: the true position as the fix, Justice D timing.")
    lines.append("Ground: ottawa2019fall photo, 0.293 m per pixel.")
    open(TABLE, "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    if sys.argv[1] == "table":
        table()
    elif sys.argv[1] == "metrics":      # recompute outage_metrics from the saved run files (no SITL run)
        res = load()
        for lb, r in res.items():
            if CASES.get(lb, {}).get("outage") and "rms" in r:
                r["outage_metrics"] = outage_metrics(r, CASES[lb])
        json.dump(res, open(RESULTS, "w"), indent=1, default=str)
        table()
    elif sys.argv[1] == "run":
        for lb in (sys.argv[2:] or list(CASES)):
            run(lb)
