#!/usr/bin/env python3
"""Pilot-switch options on ArduCopter SITL: fresh_kf_at_engage A/B and the recovery demonstration (outage_test_s).
  ~/.venvs/vio/bin/python apsim/engage_matrix.py run <label> [<label> ...]   one at a time, waits for a quiet laptop
  ~/.venvs/vio/bin/python apsim/engage_matrix.py table
Each case is apsim/sitl_ap.run_case (variant apb, video JUSTICE_D, ground ottawa2019fall = tuning site, 60 s hold,
w20g40) with program settings in case["settings"]. SITL instance 1 (ports 15020-15022). Results go to
apsim/engage_results.json / engage_results.txt (not results.json). Run files: apsim/runs/<label>.*.
<label>_kf.csv: when Hold's home keyframe and active keyframe change (a wrapper of Hold.process in the onboard
process; it reads Hold's state and changes nothing)."""
import csv
import fcntl
import glob
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
PY = os.path.expanduser("~/.venvs/vio/bin/python")
OUT, TXT = os.path.join(HERE, "engage_results.json"), os.path.join(HERE, "engage_results.txt")
RUNS = os.path.join(HERE, "runs")
INSTANCE = 1           # ports 15020-15022; the other runs use instance 0 (PX4: px4_sim) and 90 (drag_matrix.py)
DEADLINE = "06:05"     # no new run starts after this local time
LOAD_MAX = 8.0         # 1-minute load before a start (16 cores), with at most one other sim job running. Two other
                       # other runs start SITL/PX4 cases back to back: the gate of integ/justice_d/run_ap.py (load 4, no
                       # other job) never opened in 10 min. Each result keeps the load and the jobs at its start.
OUTAGE = dict(outage_test_s=3.0, outage_after_s=20.0)


def case(label):
    """eng_<h>m_<off|on>_s<seed>[r]: fresh_kf_at_engage false / true. rec_<h>m_s<seed>: outage_test_s 3,
    outage_after_s 20. eng_test_30m_s1: 25 s hold with both options, to see the run-log notes."""
    h = float(re.search(r"_(\d+)m_", label).group(1))
    seed = int(re.search(r"_s(\d+)r?$", label).group(1))
    c = dict(variant="apb", height=h, wind="w20g40", hold_s=60.0, seed=seed, label=label, instance=INSTANCE,
             video="JUSTICE_D")
    if label.startswith("rec_"):
        c["settings"] = dict(OUTAGE)
    elif label.startswith("eng_test_"):
        c.update(hold_s=25.0, settings=dict(fresh_kf_at_engage=True, outage_test_s=3.0, outage_after_s=10.0))
    elif re.match(r"eng_\d+m_(on|off)_s\d+r?$", label):
        c["settings"] = dict(fresh_kf_at_engage="_on_" in label)
    else:
        raise ValueError(f"unknown label {label}")
    return c


# ------------------------------------------------------------------ onboard process (spawned by sitl_ap.run_case)
def onboard_main(settings_path, conn, stop, log_path, opts, status_q, clk):
    """apsim/onboard_cpu.main, with Hold.process wrapped to log keyframe changes to <label>_kf.csv."""
    sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), HERE]
    import hold
    orig = hold.Hold.process
    f = open(log_path[:-len("_onboard.csv")] + "_kf.csv", "w")
    f.write("t_frame,event,kf_h,kf_n0,home_h,kf_wanted\n")

    def process(self, img, t0, *a):
        want0, home0, kf0 = getattr(self, "kf_wanted", False), getattr(self, "home", None), getattr(self, "kf", None)
        r = orig(self, img, t0, *a)
        home, kf, want = getattr(self, "home", None), getattr(self, "kf", None), getattr(self, "kf_wanted", False)
        ev = [x for x, ch in (("home", home is not home0), ("kf", kf is not kf0), ("wanted" if want else "served",
                                                                                     want != want0)) if ch]
        if ev:
            g = lambda k, x: f"{k[x]:.2f}" if k is not None and k.get(x) is not None else ""   # noqa: E731
            f.write(f"{t0:.4f},{'+'.join(ev)},{g(kf, 'h')},{kf['n0'] if kf is not None else ''},{g(home, 'h')},"
                    f"{int(want)}\n")
            f.flush()
        return r
    hold.Hold.process = process
    import onboard_cpu
    onboard_cpu.main(settings_path, conn, stop, log_path, opts, status_q, clk)


# ------------------------------------------------------------------ one run
def sitl_running():
    """ArduCopter SITL command lines on this machine (the container's processes show in the host /proc)."""
    out = []
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                a = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if a and a[0].endswith(b"arducopter"):
                out.append(b" ".join(a).decode(errors="replace"))
    return out


def mine(cmd):
    return f" -I {INSTANCE} " in cmd


def px4_busy():
    r = subprocess.run(["docker", "ps", "-q", "-f", "name=^px4_sim$"], capture_output=True, text=True)
    return bool(r.stdout.strip())


KEYS = ("Dead Reckoning", "aiding", "EKF variance", "ailsafe", "Mode", "lane", "primary", "position", "yaw reset",
        "stopped", "Vision", "visual", "Odometry")


def rows_of(label):
    rows = []
    for p in sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv"))):
        for x in csv.DictReader(open(p)):
            if x["t"]:
                rows.append(x)
    return sorted(rows, key=lambda x: float(x["t"]))


def metrics(label, te, t_end, texts, modes):
    d = np.load(os.path.join(RUNS, label + ".npz"))
    tr, hp = d["truth"], d["hold_pt"]
    t, dist = tr[:, 0] - te, np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    T = t_end - te
    inw = lambda a, b: (t >= a) & (t <= b)                                              # noqa: E731
    rms = lambda a, b: float(np.sqrt(np.mean(dist[inw(a, b)] ** 2)))                     # noqa: E731
    mx = lambda a, b: float(dist[inw(a, b)].max())                                      # noqa: E731
    at = lambda x: float(np.interp(x, t, dist))                                         # noqa: E731
    pos = lambda x: np.array([np.interp(x, t, tr[:, 1]), np.interp(x, t, tr[:, 2])])     # noqa: E731
    m = dict(rms_10s=rms(0, 10), max_10s=mx(0, 10), rms_10_60=rms(10, T), max_10_60=mx(10, T))
    rows = rows_of(label)
    rel = lambda x: round(float(x["t"]) - te, 2)                                        # noqa: E731
    notes = [(rel(x), x["note"]) for x in rows if x["note"] and not x["note"].startswith("VISO_DELAY_MS advice")]
    m["notes"] = notes
    m["notes_all_n"] = sum(1 for x in rows if x["note"])
    prev, mode_changes = None, []
    for x in rows:
        if x.get("mode") != prev:
            mode_changes.append((rel(x), x.get("mode")))
            prev = x.get("mode")
    m["onboard_modes"] = mode_changes
    m["fc_modes"] = [(round(a - te, 2), b) for a, b in modes]
    sent = [float(x["t"]) - te + float(x["age_ms"] or 0) / 1e3 for x in rows if x["sent"] == "1"]
    edges = [0.0] + [s for s in sent if 0 <= s <= T] + [T]
    gaps = [(b - a, a) for a, b in zip(edges, edges[1:])]
    m["fix_gap_max_s"], m["fix_gap_at_s"] = (round(x, 2) for x in max(gaps))
    m["fix_gaps_over_1s"] = [(round(a, 1), round(g, 1)) for g, a in gaps if g > 1.0]
    eng = [a for a, n in notes if n.startswith("engaged")]
    m["engaged_note_at"] = eng[0] if eng else None
    kf_path = os.path.join(RUNS, label + "_kf.csv")
    if os.path.exists(kf_path):
        kr = [dict(x, t=float(x["t_frame"]) - te) for x in csv.DictReader(open(kf_path))]
        m["kf_events"] = [(round(x["t"], 2), x["event"], x["kf_h"], x["home_h"]) for x in kr if -15 <= x["t"] <= T]
        srv = [x for x in kr if "served" in x["event"] and x["t"] >= -1]
        m["fresh_kf_served_at"] = round(srv[0]["t"], 2) if srv else None
        m["kf_changes_in_hold"] = sum(1 for x in kr if "kf" in x["event"].split("+") and 0 <= x["t"] <= T)
        m["home_changes_in_hold"] = sum(1 for x in kr if "home" in x["event"].split("+") and 0 <= x["t"] <= T)
        before = [x for x in kr if x["t"] < 0 and x["home_h"]]
        m["home_h_at_switch"] = before[-1]["home_h"] if before else None
    a_ = [a for a, n in notes if "video dropped for" in n]
    b_ = [a for a, n in notes if "video back" in n]
    if a_:
        ta = a_[0]
        tb = b_[0] if b_ else None
        m.update(outage_start=ta, outage_end=tb)
        if tb is not None:
            first = [s for s in sent if s >= tb]
            m.update(dist_at_outage_start=at(ta), dist_at_outage_end=at(tb), max_during=mx(ta, tb),
                     move_during=float(np.linalg.norm(pos(tb) - pos(ta))), rms_during=rms(ta, tb),
                     max_10s_after=mx(tb, tb + 10), rms_10s_after=rms(tb, tb + 10), dist_10s_after=at(tb + 10),
                     max_before=mx(0, ta), rms_before=rms(0, ta), max_rest=mx(tb + 10, T) if tb + 10 < T else None,
                     first_fix_after=round(first[0], 3) if first else None,
                     back_to_fix_s=round(first[0] - tb, 3) if first else None,
                     last_fix_before=round(max([s for s in sent if s < ta], default=math.nan), 3))
            why = [x["why"].split(" (")[0] for x in rows if tb <= float(x["t"]) - te <= (first[0] if first else tb + 5)]
            m["why_after_back"] = {w: why.count(w) for w in dict.fromkeys(why)}
    m["fc_texts_hold"] = [(a, x) for a, x in texts if a >= -1 and any(k in x for k in KEYS)]
    return m


def window(label, a, b):
    """(rms, max) of the true distance from the hold point in [a, b] s after the switch of run label."""
    d = np.load(os.path.join(RUNS, label + ".npz"))
    tr, hp, te = d["truth"], d["hold_pt"], float(d["t_engage"])
    t, dist = tr[:, 0] - te, np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    x = dist[(t >= a) & (t <= b)]
    return float(np.sqrt(np.mean(x ** 2))), float(x.max())


def fix_err(label):
    """Error of each sent fix against the truth (in the FC heading frame, as integ/justice_d/fix_error.py), relative
    to the error at the switch (mean over the last 1 s before it): the hold follows this change. RMS / worst (m) in
    [0, 10] s and [0, 60] s after the switch, and the step over the switch (first 1 s after minus last 1 s before)."""
    d = np.load(os.path.join(RUNS, label + ".npz"))
    r = json.load(open(os.path.join(RUNS, label + ".json")))
    tr, te = d["truth"], float(d["t_engage"])
    dy, ro = math.radians(r["ekf_yaw_minus_true_deg"]), r["cfg"]["readout_s"]
    allrows = rows_of(label)
    drop = [float(x["t"]) - te for x in allrows if "video dropped for" in x["note"]]
    back = [float(x["t"]) - te for x in allrows if "video back" in x["note"]]
    rows = [x for x in allrows if x["sent"] == "1"]
    t = np.array([float(x["t"]) for x in rows]) + ro / 2 - te
    p = np.array([[float(x["north"]), float(x["east"])] for x in rows])
    N, E = np.interp(t + te, tr[:, 0], tr[:, 1]), np.interp(t + te, tr[:, 0], tr[:, 2])
    c, s_ = math.cos(dy), math.sin(dy)
    e = p - np.c_[c * N - s_ * E, s_ * N + c * E]
    pre, post = e[(t >= -1) & (t < 0)], e[(t >= 0) & (t < 1)]
    if not len(pre):
        return {}
    e = e - pre.mean(0)
    n = np.linalg.norm(e, axis=1)
    w = lambda a, b: n[(t >= a) & (t <= b)]                                              # noqa: E731
    extra = {}
    if drop and back:   # the frame shift of the re-anchor after the drop: mean fix error 5 s before against 1-7 s after
        a, b = e[(t >= drop[0] - 5) & (t < drop[0])], e[(t >= back[0] + 1) & (t < back[0] + 7)]
        if len(a) and len(b):
            extra = dict(ferr_mean_before_drop=a.mean(0).round(3).tolist(), ferr_mean_after_back=b.mean(0).round(3).tolist(),
                         anchor_shift_m=float(np.linalg.norm(b.mean(0) - a.mean(0))))
    return dict(**extra, ferr_rms_10s=float(np.sqrt(np.mean(w(0, 10) ** 2))), ferr_max_10s=float(w(0, 10).max()),
                ferr_rms_60s=float(np.sqrt(np.mean(w(0, 60) ** 2))), ferr_max_60s=float(w(0, 60).max()),
                ferr_step=float(np.linalg.norm(post.mean(0))) if len(post) else None)


def one(label):
    sys.path.insert(0, HERE)
    import sitl_ap as A
    links = []

    class Link(A.APLink):       # keeps every FC text and the FC mode changes (supervisor side)
        def __init__(self, c, port):
            super().__init__(c, port)
            self.modes = []
            links.append(self)
            threading.Thread(target=self._modes, daemon=True).start()

        def _modes(self):
            last = None
            while not self.stop_all:
                if getattr(self, "mode", None) != last:
                    last = self.mode
                    try:
                        self.modes.append((self.now(), {2: "ALT_HOLD", 4: "GUIDED", 5: "LOITER"}.get(last, str(last))))
                    except Exception:
                        pass
                time.sleep(0.02)

    A.APLink = Link
    sys.modules["onboard_cpu"] = types.SimpleNamespace(main=onboard_main)   # run_case: import onboard_cpu as onboard
    c = case(label)
    t_wall, load0 = time.time(), os.getloadavg()
    jobs0 = dict(other_sitl=sum(1 for x in sitl_running() if not mine(x)), px4_sim=px4_busy())
    try:
        res = A.run_case(c)
    except Exception as e:      # a broken run is logged, not lost
        res = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")])
    r = {k: res.get(k) for k in ("case", "rms", "p95", "max", "end", "pct_frames_fix", "fix_rate_hz", "events",
                                   "hold_s_flown", "climb_s", "age_ms_p50", "drift_m_per_min", "proc_ms_mean",
                                   "handover_ms_p50", "handover_ms_p95", "reanchors", "hold_why", "onboard_exit",
                                   "frac_ok", "onboard")}
    r.update(height=c["height"], seed=c["seed"], settings=c.get("settings", {}), video=c["video"],
             wall_s=round(time.time() - t_wall), load_start=load0, load_end=os.getloadavg(), jobs_start=jobs0,
             jobs_end=dict(other_sitl=sum(1 for x in sitl_running() if not mine(x)), px4_sim=px4_busy()),
             option=("outage 3 s at 20 s" if label.startswith("rec") else
                     "test" if "_test_" in label else "fresh_kf " + ("true" if "_on_" in label else "false")))
    if links:
        k = links[-1]
        te = k.t_engage if k.t_engage < math.inf else (k.texts[-1][0] if k.texts else 0.0)
        r["texts_all"] = [(round(t - te, 2), x) for t, x in k.texts]
        if k.t_engage < math.inf and res.get("hold_s_flown"):
            try:
                r.update(metrics(label, k.t_engage, k.t_engage + res["hold_s_flown"], r["texts_all"], k.modes))
                r.update(fix_err(label))
            except Exception as e:
                r["metrics_error"] = f"{type(e).__name__}: {e}"
    with open(OUT + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            allr = json.load(open(OUT))
        except (OSError, ValueError):
            allr = {}
        allr[label] = r
        json.dump(allr, open(OUT, "w"), indent=1, default=str)
    table()
    print(json.dumps({k: v for k, v in r.items() if k not in ("texts_all", "onboard")}, default=str))


# ------------------------------------------------------------------ table
def table():
    try:
        allr = json.load(open(OUT))
    except (OSError, ValueError):
        return
    f = lambda r, x, n=7, p=2: (f"{r[x]:{n}.{p}f}" if isinstance(r.get(x), (int, float)) else "-".rjust(n))  # noqa: E731
    lines = ["run name".ljust(18) + "h m".rjust(5) + "  option".ljust(22) + "seed".rjust(5) + "rms m".rjust(7) +
             "max m".rjust(7) + "rms10s".rjust(7) + "% fix".rjust(7) + "fix Hz".rjust(7) + "kf at".rjust(7) +
             "load".rjust(6) + "ferr60".rjust(7) + "ferr10".rjust(7) + "  events (s after the switch)"]
    for k in sorted(allr, key=lambda k: (allr[k].get("height") or 0, k)):
        r = allr[k]
        ev = [f"{t:g}s {w}" for t, w in (r.get("events") or [])[1:]]
        ev += [f"{t:g}s {w}" for t, w in r.get("notes", [])]
        ev += [f"{t:g}s FC {m}" for t, m in r.get("fc_modes", []) if t > 0.5]
        ev += [f"{t:g}s FC '{x}'" for t, x in r.get("fc_texts_hold", [])]
        lines.append(k.ljust(18) + f"{r.get('height', 0):5.0f}" + "  " + str(r.get("option", "")).ljust(20) +
                     f"{r.get('seed', 0):5d}" + f(r, "rms") + f(r, "max") + f(r, "rms_10s") + f(r, "pct_frames_fix", 7, 1) +
                     f(r, "fix_rate_hz", 7, 1) + f(r, "fresh_kf_served_at", 7, 2) +
                     (f"{r['load_start'][0]:6.1f}" if r.get("load_start") else "-".rjust(6)) + f(r, "ferr_rms_60s") +
                     f(r, "ferr_rms_10s") + "  " + "; ".join(ev)[:400])
    rec = [k for k in sorted(allr) if allr[k].get("outage_start") is not None]
    if rec:
        lines += ["", "Recovery (s and m; times after the switch; drift = true distance from the hold point; d start/end: at the",
                  "drop and at video back; move in: displacement during the drop; back->fix: video back to the first sent",
                  "fix; shift: change of the mean fix error, 5 s before the drop against 1-7 s after video back):",
                  "run name".ljust(18) + "".join(n.rjust(9) for n in ("drop at", "back at", "d start", "d end", "max in",
                                                                     "move in", "max 10s", "rms 10s", "back->fix",
                                                                     "max all", "max bef", "shift"))]
        for k in rec:
            r = allr[k]
            g = lambda x: (f"{r[x]:9.2f}" if isinstance(r.get(x), (int, float)) else "-".rjust(9))   # noqa: E731
            lines.append(k.ljust(18) + "".join(g(x) for x in ("outage_start", "outage_end", "dist_at_outage_start",
                                                               "dist_at_outage_end", "max_during", "move_during",
                                                               "max_10s_after", "rms_10s_after", "back_to_fix_s",
                                                               "max", "max_before", "anchor_shift_m")))
        lines += ["Same windows in the run without the drop (same height and seed, fresh_kf false): rms / max m",
                  "run name".ljust(18) + "reference".ljust(18) + "during drop".rjust(16) + "10 s after".rjust(16) +
                  "0 s to drop".rjust(16) + "whole hold".rjust(16)]
        for k in [k for k in rec if k.startswith("rec_")]:
            r = allr[k]
            ref = f"eng_{r['height']:.0f}m_off_s{r['seed']}"
            if ref not in allr or not isinstance(r.get("outage_end"), (int, float)):
                continue
            ta, tb = r["outage_start"], r["outage_end"]
            try:
                w = [window(ref, a, b) for a, b in ((ta, tb), (tb, tb + 10), (0, ta))]
                w2 = [window(k, a, b) for a, b in ((ta, tb), (tb, tb + 10), (0, ta))]
            except (OSError, KeyError, ValueError):
                continue
            cell = lambda x: f"{x[0]:.2f} / {x[1]:.2f}".rjust(16)                         # noqa: E731
            lines.append(k.ljust(18) + "(this run)".ljust(18) + "".join(cell(x) for x in w2) +
                         cell((r["rms"], r["max"])))
            lines.append("".ljust(18) + ref.ljust(18) + "".join(cell(x) for x in w) +
                         cell((allr[ref].get("rms") or math.nan, allr[ref].get("max") or math.nan)))
    lines += ["", "apsim/engage_matrix.py. ArduCopter 4.6 SITL, variant apb (onboard program, board model fixed 80 ms per",
              "Hold.process), video JUSTICE_D, ground ottawa2019fall (tuning site), w20g40, 60 s hold, SITL instance 1.",
              "rms/max: true horizontal distance from the hold point over the hold (sitl.score). rms10s: over the first",
              "10 s after the switch. % fix: fixes sent per rendered frame (60 fps). kf at: time after the switch when",
              "Hold served the fresh-keyframe request (<label>_kf.csv). load: 1-minute load at the start (16 cores).",
              "ferr60/ferr10: RMS of the sent-fix error against the truth, relative to the error at the switch, over",
              "the hold / its first 10 s (fix_err: the part of the estimator error that moves the drone).",
              "Events: sitl_ap events, run-log notes (no VISO_DELAY_MS advice), FC mode changes, FC texts."]
    open(TXT, "w").write("\n".join(lines) + "\n")


def run(labels):
    done = set()
    try:
        done = set(json.load(open(OUT)))
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
            sitl = sitl_running()
            if any(mine(x) for x in sitl):        # run_sitl.sh stop would kill it: never start over it
                print(time.strftime("%H:%M:%S"), "instance", INSTANCE, "is in use; wait", flush=True)
                time.sleep(30)
                continue
            others, busy, one_min = [x for x in sitl if not mine(x)], px4_busy(), os.getloadavg()[0]
            waited = time.time() - t0
            lim = LOAD_MAX if waited < 300 else 12.0 if waited < 600 else math.inf
            if (len(others) + busy <= 1 and one_min < lim) or waited >= 600:
                break
            print(time.strftime("%H:%M:%S"), f"wait: load {one_min:.1f} (max {lim}), other SITL {len(others)}, "
                  f"px4_sim {busy}", flush=True)
            time.sleep(20)
        print(time.strftime("%H:%M:%S"), "start", label, "load", os.getloadavg(), "waited", round(time.time() - t0),
              flush=True)
        log = open(os.path.join(HERE, "logs", f"eng_{label}.txt"), "w")
        subprocess.run([PY, os.path.abspath(__file__), "one", label], stdout=log, stderr=subprocess.STDOUT)
        print(time.strftime("%H:%M:%S"), "end", label, flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "one":
        one(a[1])
    elif a[0] == "run":
        run(a[1:])
    elif a[0] == "table":
        table()
        print(open(TXT).read())
    elif a[0] == "diag":                     # fix_err for runs already in the results
        with open(OUT + ".lock", "w") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            allr = json.load(open(OUT))
            for k in allr:
                if allr[k].get("hold_s_flown"):
                    allr[k].update(fix_err(k))
            json.dump(allr, open(OUT, "w"), indent=1, default=str)
        table()
        print(open(TXT).read())
    elif a[0] == "case":
        for x in a[1:]:
            print(x, case(x))
