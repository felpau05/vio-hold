#!/usr/bin/env python3
"""hold_cfg {"gap_jump_s": 0.5} in the recovery demonstration on ArduCopter SITL (instance 1).
  ~/.venvs/vio/bin/python apsim/gapjump_matrix.py run <label> [<label> ...]   one at a time
  ~/.venvs/vio/bin/python apsim/gapjump_matrix.py table
A wrapper of apsim/engage_matrix.py: the same case, metrics and table. Labels rec_<h>m_gj_s<seed> are the case
rec_<h>m_s<seed> (outage_test_s 3, outage_after_s 20, JUSTICE_D, ottawa2019fall, w20g40, 60 s hold) plus case
hold_cfg {"gap_jump_s": 0.5}; px4sim/sitl.sim_settings merges it over the justice_d overlay's hold_cfg.
Results: apsim/gapjump_results.json / gapjump_results.txt. rec_60m_s1/s2 and eng_60m_off_s1/s2 are copied from
apsim/engage_results.json (not run again). Run files: apsim/runs/<label>.*, run logs apsim/logs/gj_<label>.txt."""
import fcntl
import json
import os
import subprocess
import sys
import time

import numpy as np

import engage_matrix as E

HERE = E.HERE
E.OUT, E.TXT = os.path.join(HERE, "gapjump_results.json"), os.path.join(HERE, "gapjump_results.txt")
GJ = 0.5
COPY = ("rec_60m_s1", "rec_60m_s2", "eng_60m_off_s1", "eng_60m_off_s2")
DEADLINE = "06:00"     # no new run starts after this local time
_case = E.case


def case(label):
    c = _case(label)
    if "_gj_" in label:
        c["hold_cfg"] = {"gap_jump_s": GJ}
    return c


E.case = case


def after_back(label, n=10):
    """The first n onboard-log rows from the 'video back' row: (t after the switch, state, why, sent, note)."""
    te = float(np.load(os.path.join(E.RUNS, label + ".npz"))["t_engage"])
    rows = E.rows_of(label)
    i = next((k for k, x in enumerate(rows) if "video back" in x["note"]), None)
    if i is None:
        return []
    return [(round(float(x["t"]) - te, 2), x["state"], x["why"], x["sent"], x["note"]) for x in rows[i:i + n]]


def shift_nodrop(label, ta=20.0, tb=23.0):
    """The 'shift' of engage_matrix.fix_err (mean fix error 5 s before ta against 1-7 s after tb) in a run without
    a drop: how far the fix error moves in these windows with no outage at all."""
    import math
    d = np.load(os.path.join(E.RUNS, label + ".npz"))
    r = json.load(open(os.path.join(E.RUNS, label + ".json")))
    tr, te = d["truth"], float(d["t_engage"])
    dy, ro = math.radians(r["ekf_yaw_minus_true_deg"]), r["cfg"]["readout_s"]
    rows = [x for x in E.rows_of(label) if x["sent"] == "1"]
    t = np.array([float(x["t"]) for x in rows]) + ro / 2 - te
    p = np.array([[float(x["north"]), float(x["east"])] for x in rows])
    N, E_ = np.interp(t + te, tr[:, 0], tr[:, 1]), np.interp(t + te, tr[:, 0], tr[:, 2])
    c, s_ = math.cos(dy), math.sin(dy)
    e = p - np.c_[c * N - s_ * E_, s_ * N + c * E_]
    a, b = e[(t >= ta - 5) & (t < ta)], e[(t >= tb + 1) & (t < tb + 7)]
    return float(np.linalg.norm(b.mean(0) - a.mean(0)))


def table():
    E.table()
    allr = json.load(open(E.OUT))
    lines = ["", "Per run: hold_cfg from runs/<label>_settings.json; re-anchors after video back (run-log notes);",
             "the first 10 onboard-log rows from 'video back' (t after the switch, state, sent, why | note):"]
    for k in sorted(allr):
        r = allr[k]
        if r.get("outage_end") is None:
            continue
        tb = r["outage_end"]
        re_n = sum(1 for t, x in r.get("notes", []) if t >= tb - 0.01 and "re-anchored" in x)
        lines.append(f"{k}: hold_cfg {r.get('hold_cfg')}; re-anchors after video back {re_n}; "
                     f"back->fix {r.get('back_to_fix_s')} s; shift {r.get('anchor_shift_m', float('nan')):.2f} m")
        for t, st, why, sent, note in r.get("rows_after_back", []):
            lines.append(f"   {t:7.2f} {st or '-':9s} {sent} {why}" + (f" | {note}" if note else ""))
    lines += ["", "Shift with no drop (same windows: 5 s before 20 s against 24-30 s after the switch):"]
    for k in sorted(allr):
        if k.startswith("eng_") and allr[k].get("hold_s_flown"):
            try:
                lines.append(f"{k}: shift {shift_nodrop(k):.2f} m")
            except (OSError, KeyError, ValueError) as e:
                lines.append(f"{k}: - ({e})")
    open(E.TXT, "a").write("\n".join(lines) + "\n")


def tag(label):
    """Add hold_cfg, the option name and the rows after video back to the result of label."""
    with open(E.OUT + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        allr = json.load(open(E.OUT))
        r = allr.get(label)
        if r is None:
            return
        try:
            r["hold_cfg"] = json.load(open(os.path.join(E.RUNS, label + "_settings.json")))["hold_cfg"]
        except (OSError, KeyError, ValueError):
            pass
        if "_gj_" in label:
            r["option"] = f"outage + gap_jump {GJ:g}"
        try:
            r["rows_after_back"] = after_back(label)
        except (OSError, KeyError, ValueError):
            pass
        json.dump(allr, open(E.OUT, "w"), indent=1, default=str)


def seed_copies():
    with open(E.OUT + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            allr = json.load(open(E.OUT))
        except (OSError, ValueError):
            allr = {}
        old = json.load(open(os.path.join(HERE, "engage_results.json")))
        for k in COPY:
            if k not in allr and k in old:
                allr[k] = old[k]
        json.dump(allr, open(E.OUT, "w"), indent=1, default=str)
    for k in COPY:
        if k.startswith("rec_"):
            tag(k)


def run(labels):
    seed_copies()
    done = set(json.load(open(E.OUT)))
    for label in labels:
        if label in done:
            print("skip (done)", label, flush=True)
            continue
        t0 = time.time()
        while True:
            if time.strftime("%H:%M") >= DEADLINE:
                print("deadline", DEADLINE, "reached; stop", flush=True)
                return
            sitl = E.sitl_running()
            if any(E.mine(x) for x in sitl):
                print(time.strftime("%H:%M:%S"), "instance", E.INSTANCE, "is in use; wait", flush=True)
                time.sleep(30)
                continue
            others, one_min, waited = [x for x in sitl if not E.mine(x)], os.getloadavg()[0], time.time() - t0
            if (len(others) <= 1 and one_min < E.LOAD_MAX) or waited >= 300:
                break
            print(time.strftime("%H:%M:%S"), f"wait: load {one_min:.1f}, other SITL {len(others)}", flush=True)
            time.sleep(20)
        print(time.strftime("%H:%M:%S"), "start", label, "load", os.getloadavg(), flush=True)
        log = open(os.path.join(HERE, "logs", f"gj_{label}.txt"), "w")
        subprocess.run([E.PY, os.path.abspath(__file__), "one", label], stdout=log, stderr=subprocess.STDOUT)
        tag(label)
        table()
        print(time.strftime("%H:%M:%S"), "end", label, flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "one":
        E.one(a[1])
    elif a[0] == "run":
        run(a[1:])
    elif a[0] == "table":
        seed_copies()
        table()
        print(open(E.TXT).read())
    elif a[0] == "case":
        for x in a[1:]:
            print(x, case(x))
