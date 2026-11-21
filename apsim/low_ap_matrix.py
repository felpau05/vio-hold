#!/usr/bin/env python3
"""Low-height holds (5 m, 10 m) on ArduCopter 4.6 SITL with the new outdoor floor: settings min_height_m 3,
hold_cfg h_min 2 (tools/profiles/outdoor_day.json since 2026-11-19 04:10). The PX4 evidence for that change is
px4sim/low_outage_results.txt; this is the ArduCopter check.
  ~/.venvs/vio/bin/python apsim/low_ap_matrix.py run <label> [<label> ...]   one at a time
  ~/.venvs/vio/bin/python apsim/low_ap_matrix.py table
Labels lowap_<apb|truth>_<h>m_<calm|w20g40>_s<seed>. A wrapper of apsim/engage_matrix.py (one(), metrics, FC
texts and modes): variant apb or truth, video JUSTICE_D, ground ottawa2019fall, 60 s hold, SITL instance 1.
Both variants get the same settings and hold_cfg (truth: they only reach the parameter file; no program runs).
Results: apsim/low_ap_results.json / low_ap_results.txt. Run files: apsim/runs/<label>.*, run logs
apsim/logs/lowap_<label>.txt. tools/log_summary.py output on the onboard log is kept per run."""
import fcntl
import glob
import json
import os
import re
import subprocess
import sys
import time

import numpy as np

import engage_matrix as E

HERE, PROJ, RUNS = E.HERE, E.PROJ, E.RUNS
E.OUT, E.TXT = os.path.join(HERE, "low_ap_results.json"), os.path.join(HERE, "low_ap_results.txt")
LOW_SETTINGS, LOW_HOLD = {"min_height_m": 3.0}, {"h_min": 2.0}
DEADLINE = "06:05"     # no new run starts after this local time
SUITE_FILE = os.path.join(PROJ, "tests_apb", "out_timing_test.txt")
SUITE_AFTER = "2026-11-19 04:16"   # coordinator: no SITL until the test suite has written this file after 04:16


def case(label):
    m = re.match(r"lowap_(apb|truth)_(\d+)m_(calm|w20g40)_s(\d+)$", label)
    if not m:
        raise ValueError(f"unknown label {label}")
    return dict(variant=m.group(1), height=float(m.group(2)), wind=m.group(3), hold_s=60.0, seed=int(m.group(4)),
                label=label, instance=E.INSTANCE, video="JUSTICE_D", settings=dict(LOW_SETTINGS),
                hold_cfg=dict(LOW_HOLD))


E.case = case        # E.one builds the case with E.case


def log_summary(label):
    logs = sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv")))
    if not logs:
        return None
    r = subprocess.run([E.PY, os.path.join(PROJ, "tools", "log_summary.py"), *logs], capture_output=True, text=True)
    return (r.stdout + r.stderr).strip()


def extra(label, r):
    """Height in the hold (truth), sent-fix facts at the low height, log_summary output; truth runs: no fix gap."""
    try:
        d = np.load(os.path.join(RUNS, label + ".npz"))
        tr, te = d["truth"], float(d["t_engage"])
        T = r.get("hold_s_flown") or 0.0
        w = (tr[:, 0] >= te) & (tr[:, 0] <= te + T)
        if w.any():
            h = tr[w, 3]
            r.update(h_mean=float(h.mean()), h_min=float(h.min()), h_max=float(h.max()))
        r["h_at_switch"] = float(np.interp(te, tr[:, 0], tr[:, 3]))
    except (OSError, KeyError, ValueError, TypeError):
        pass
    rows = E.rows_of(label)
    if r.get("variant") == "truth" or not rows:
        for k in ("fix_gap_max_s", "fix_gap_at_s", "fix_gaps_over_1s"):
            r[k] = None
    if rows:
        sent = [x for x in rows if x["sent"] == "1"]
        if sent:
            r["first_sent_t"] = float(sent[0]["t"])
            r["first_sent_height"] = sent[0]["height"]
        te = None
        try:
            te = float(np.load(os.path.join(RUNS, label + ".npz"))["t_engage"])
        except (OSError, KeyError, ValueError):
            pass
        if te is not None:
            r["first_sent_rel_switch"] = round(float(sent[0]["t"]) - te, 2) if sent else None
            hold = [x for x in rows if te <= float(x["t"]) <= te + (r.get("hold_s_flown") or 0)]
            why = [x["why"].split(" (")[0] for x in hold if x["sent"] != "1" and x["why"]]
            r["why_not_sent_hold"] = {k: why.count(k) for k in dict.fromkeys(why)}
            low = [x for x in rows if x["why"].startswith("height ")]
            r["height_blocked_rows"] = len(low)
            r["height_blocked_last"] = (low[-1]["t"], low[-1]["why"]) if low else None
    r["log_summary"] = log_summary(label)
    r["hold_cfg"], r["variant"] = dict(LOW_HOLD), r.get("variant")
    return r


def one(label):
    c = case(label)
    E.one(label)
    with open(E.OUT + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        allr = json.load(open(E.OUT))
        r = allr[label]
        r["variant"], r["wind"], r["option"] = c["variant"], c["wind"], "min_height_m 3, h_min 2"
        try:
            j = json.load(open(os.path.join(RUNS, label + ".json")))
            r["fix_rate_hz"] = j.get("fix_rate_hz", r.get("fix_rate_hz"))
            r["ap_text_tail"] = j.get("ap_text")
            r["settings_file_hold_cfg"] = json.load(open(os.path.join(RUNS, label + "_settings.json")))["hold_cfg"]
            r["settings_file_min_height_m"] = json.load(open(os.path.join(RUNS, label + "_settings.json")))["min_height_m"]
        except (OSError, KeyError, ValueError):
            pass
        allr[label] = extra(label, r)
        json.dump(allr, open(E.OUT, "w"), indent=1, default=str)
    table()


def table():
    try:
        allr = json.load(open(E.OUT))
    except (OSError, ValueError):
        return
    f = lambda r, x, n=7, p=2: (f"{r[x]:{n}.{p}f}" if isinstance(r.get(x), (int, float)) else "-".rjust(n))  # noqa: E731
    lines = ["run name".ljust(26) + "rms m".rjust(7) + "max m".rjust(7) + "% fix".rjust(7) + "fix Hz".rjust(7) +
             "gap s".rjust(7) + "at s".rjust(7) + "h mean".rjust(7) + "h min".rjust(7) + "climb".rjust(7) +
             "load".rjust(6)]
    for k in sorted(allr):
        r = allr[k]
        lines.append(k.ljust(26) + f(r, "rms") + f(r, "max") + f(r, "pct_frames_fix", 7, 1) + f(r, "fix_rate_hz", 7, 1) +
                     f(r, "fix_gap_max_s") + f(r, "fix_gap_at_s", 7, 1) + f(r, "h_mean") + f(r, "h_min") +
                     f(r, "climb_s", 7, 1) + (f"{r['load_start'][0]:6.1f}" if r.get("load_start") else "-".rjust(6)))
    lines += ["", "Per run (times in s after the switch to LOITER):"]
    for k in sorted(allr):
        r = allr[k]
        lines.append(f"{k}:")
        lines.append(f"  events: {r.get('events')}")
        lines.append(f"  FC modes: {r.get('fc_modes')}")
        lines.append(f"  onboard modes: {r.get('onboard_modes')}")
        lines.append(f"  program notes: {r.get('notes')}")
        lines.append(f"  first sent fix: {r.get('first_sent_rel_switch')} s, at height {r.get('first_sent_height')} m; "
                     f"rows blocked by height: {r.get('height_blocked_rows')} (last {r.get('height_blocked_last')}); "
                     f"not sent in hold: {r.get('why_not_sent_hold')}; gaps over 1 s: {r.get('fix_gaps_over_1s')}")
        lines.append(f"  settings file: min_height_m {r.get('settings_file_min_height_m')}, hold_cfg "
                     f"{r.get('settings_file_hold_cfg')}")
        lines.append("  FC texts (from 15 s before the switch):")
        lines += [f"    {t:8.2f} {x}" for t, x in (r.get("texts_all") or []) if t >= -15]
        if r.get("log_summary"):
            lines.append("  tools/log_summary.py on the onboard log (whole flight, ground to landing):")
            lines += ["    " + x for x in r["log_summary"].splitlines()]
    lines += ["", "apsim/low_ap_matrix.py. ArduCopter 4.6 SITL (instance 1), variant apb (onboard program, board model",
              "fixed 80 ms per Hold.process) or truth (true position as the fix, 25 Hz), video JUSTICE_D, ground",
              "ottawa2019fall, 60 s hold, settings min_height_m 3, hold_cfg h_min 2 (on top of justice_d.json).",
              "rms/max: true horizontal distance from the hold point over the hold. % fix: sent fixes per rendered frame",
              "(60 fps) in the hold. gap s / at s: longest time in the hold without a sent fix (from the switch to the",
              "end) and when it starts. h mean/min: true height above the start point in the hold. climb: s from",
              "the climb start to the end of the 4 s settle. load: 1-minute load at the start (16 cores)."]
    open(E.TXT, "w").write("\n".join(lines) + "\n")


E.table = table      # E.one calls table() at its end


def suite_busy():
    me = str(os.getpid())
    for p in os.listdir("/proc"):
        if p.isdigit() and p != me:
            try:
                a = open(f"/proc/{p}/cmdline", "rb").read()
            except OSError:
                continue
            if b"run_all.sh" in a and b"low_ap_matrix" not in a:
                return True
    return False


def suite_done():
    try:
        return time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(SUITE_FILE))) > SUITE_AFTER
    except OSError:
        return False


def run(labels):
    done = set()
    try:
        done = set(json.load(open(E.OUT)))
    except (OSError, ValueError):
        pass
    for label in labels:
        if label in done:
            print("skip (done)", label, flush=True)
            continue
        t0 = time.time()
        while True:
            if time.strftime("%H:%M") >= DEADLINE:
                print("deadline", DEADLINE, "reached; stop", flush=True)
                return
            if not suite_done() or suite_busy():
                print(time.strftime("%H:%M:%S"), "test suite not finished; wait", flush=True)
                time.sleep(20)
                t0 = time.time()
                continue
            sitl = E.sitl_running()
            if any(E.mine(x) for x in sitl):
                print(time.strftime("%H:%M:%S"), "instance", E.INSTANCE, "is in use; wait", flush=True)
                time.sleep(30)
                continue
            others, busy, one_min, waited = [x for x in sitl if not E.mine(x)], E.px4_busy(), os.getloadavg()[0], \
                time.time() - t0
            if (len(others) + busy <= 1 and one_min < E.LOAD_MAX) or waited >= 300:
                break
            print(time.strftime("%H:%M:%S"), f"wait: load {one_min:.1f}, other SITL {len(others)}, px4_sim {busy}",
                  flush=True)
            time.sleep(20)
        print(time.strftime("%H:%M:%S"), "start", label, "load", os.getloadavg(), flush=True)
        log = open(os.path.join(HERE, "logs", f"lowap_{label}.txt"), "w")
        subprocess.run([E.PY, os.path.abspath(__file__), "one", label], stdout=log, stderr=subprocess.STDOUT)
        print(time.strftime("%H:%M:%S"), "end", label, flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "one":
        one(a[1])
    elif a[0] == "run":
        run(a[1:])
    elif a[0] == "table":
        table()
        print(open(E.TXT).read())
    elif a[0] == "case":
        for x in a[1:]:
            print(x, case(x))
