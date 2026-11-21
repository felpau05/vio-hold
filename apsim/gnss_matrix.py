#!/usr/bin/env python3
"""GNSS levels (settings gnss_mode, apb_main.GnssLevel) on ArduCopter 4.6 SITL. integ/gnss_ladder/README.txt.
  ~/.venvs/vio/bin/python apsim/gnss_matrix.py run <label> [<label> ...]   one at a time, waits for load < 3
  ~/.venvs/vio/bin/python apsim/gnss_matrix.py table
Labels: gn_<none|setup|fallback|velocity>_<h>m_s<seed>[_glitch][_lh]; _lh adds the profile low_height (added
2026-11-21 07:15: the level together with low_height); gn_gps_<h>m_s<seed>_glitch (GNSS-only Loiter, no
program: the control for the glitch). Each case is apsim/sitl_ap.run_case: variant apb, video JUSTICE_D, ground
ottawa2019fall (tuning site), seeds 1-9 (tuning), w20g40, 40 s hold, SITL instance 1, the outdoor_day floor
(min_height_m 3, hold_cfg h_min 2) for every level and height. fallback: video dropped for 6 s, 10 s after the switch
(outage_test_s), gnss_fallback_s 3. _glitch: SIM_GPS_GLITCH_X = 20 m north (SIM_GPS.cpp 423-428 adds it to the
latitude), 10 s after the switch, for the rest of the hold. The FC log (BIN) is copied to apsim/runs/<label>.BIN and
its EV events read (85/86 = EK3 sources set to primary / secondary, AP_Logger.h 94-95).
Results: apsim/gnss_results.json / gnss_results.txt (not results.json). Run files: apsim/runs/<label>.*.
Measures over the 30 s from the switch: rms/max true distance from the position at the switch; EKF step = largest
change between two LOCAL_POSITION_NED samples (30 Hz) within 1 s of the event, less the true motion between them;
height error = largest |true height - height at the switch|."""
import csv
import glob
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path[:0] = [PROJ, HERE]
PY = os.path.expanduser("~/.venvs/vio/bin/python")
OUT, TXT = os.path.join(HERE, "gnss_results.json"), os.path.join(HERE, "gnss_results.txt")
RUNS = os.path.join(HERE, "runs")
INSTANCE = 1
LOAD_MAX = 3.0
DEADLINE = "08:10"
WIN = 30.0
GLITCH_AT, GLITCH_M = 10.0, 20.0
RAMP_AT, RAMP_S = 5.0, 20.0            # _ramp: the same 20 m, reached at 1 m/s from 5 s to 25 s after the switch
FLOOR = {"min_height_m": 3.0}, {"h_min": 2.0}            # tools/profiles/outdoor_day.json
OUTAGE = dict(outage_test_s=6.0, outage_after_s=10.0)


def case(label):
    m = re.match(r"gn_(none|setup|fallback|velocity|gps)_(\d+)m_s(\d+)(_glitch|_ramp)?(_lh)?$", label)
    if not m:
        raise ValueError(f"unknown label {label}")
    import apb_main
    import settings as S
    lvl, h, seed, glitch, ramp = m.group(1), float(m.group(2)), int(m.group(3)), bool(m.group(4)), m.group(4) == "_ramp"
    st = dict(FLOOR[0])
    if lvl in ("setup", "fallback", "velocity"):
        st.update(gnss_mode=lvl, params_ardupilot=dict(S.DEFAULTS["params_ardupilot"], **apb_main.GNSS_AP[lvl]))
    if lvl == "fallback":
        st.update(OUTAGE)
    hc = dict(FLOOR[1])
    if m.group(5):                         # _lh: the profile tools/profiles/low_height.json on top of the level
        st.update(att_source="gyro_fcyaw")
        hc.update(dense_local=True, dense_turn=True, dense_nadir_deg=0, kf_map=False)
    return dict(variant="gps" if lvl == "gps" else "apb", height=h, wind="w20g40", hold_s=40.0, seed=seed, label=label,
                instance=INSTANCE, video="JUSTICE_D", settings=st, hold_cfg=hc,
                glitch_deg=GLITCH_M / 111320.0 if glitch else 0.0, ramp=ramp, level=lvl)


def onboard_rows(label):
    rows = []
    for p in sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv"))):
        rows += [x for x in csv.DictReader(open(p)) if x["t"]]
    return sorted(rows, key=lambda x: float(x["t"]))


def bin_events(label):
    """EV ids and their times (s of FC time) from the newest BIN log of the instance, copied to runs/<label>.BIN."""
    r = subprocess.run(["docker", "exec", "ap_sim", "sh", "-c", f"ls -t /tmp/ap{INSTANCE}/logs/*.BIN | head -1"],
                       capture_output=True, text=True)
    src = r.stdout.strip()
    if not src:
        return None
    dst = os.path.join(RUNS, label + ".BIN")
    subprocess.run(["docker", "cp", f"ap_sim:{src}", dst], capture_output=True)
    return read_bin(dst)


def innov(path, te, t_set2=None):
    """EKF core 0 velocity innovations (XKF3 IVN, IVE): number of distinct values and their spread, before the change to
    set 2 (GNSS phase) and over the 30 s hold. A velocity that is fused changes its innovation at each fusion; a constant
    value means that no horizontal velocity was fused. IPN (position innovation) for comparison."""
    from pymavlink import DFReader
    log, rows = DFReader.DFReader_binary(path), []
    while (m := log.recv_match(type=["XKF3"])) is not None:
        if m.C == 0:
            rows.append((m.TimeUS * 1e-6 - te, m.IVN, m.IVE, m.IPN))
    a, out = np.array(rows), {}
    wins = {"hold_0_30": (0.0, WIN)}
    if t_set2 is not None:
        wins["gnss_phase_8s_before_set2"] = (t_set2 - 8.0, t_set2)
    for k, (lo, hi) in wins.items():
        s = a[(a[:, 0] >= lo) & (a[:, 0] < hi)] if len(a) else a
        if len(s):
            out[k] = dict(n=len(s), ivn_distinct=len(np.unique(s[:, 1])), ive_distinct=len(np.unique(s[:, 2])),
                          iv_std=round(float(np.hypot(s[:, 1].std(), s[:, 2].std())), 4),
                          ipn_distinct=len(np.unique(s[:, 3])))
    return out


def read_bin(path):
    """[(FC time s, "EV", id) or (FC time s, "MODE", mode number)] of a BIN log."""
    from pymavlink import DFReader
    log, out = DFReader.DFReader_binary(path), []
    while (m := log.recv_match(type=["EV", "MODE"])) is not None:
        out.append((round(m.TimeUS * 1e-6, 3), m.get_type(), int(m.Id if m.get_type() == "EV" else m.Mode)))
    return out


def metrics(label, texts):
    d = np.load(os.path.join(RUNS, label + ".npz"))
    tr, hp, te = d["truth"], d["hold_pt"], float(d["t_engage"])
    t = tr[:, 0] - te
    dist = np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    w = (t >= 0) & (t <= WIN)
    h0 = float(np.interp(0.0, t, tr[:, 3]))
    m = dict(rms30=float(np.sqrt(np.mean(dist[w] ** 2))), max30=float(dist[w].max()),
             herr30=float(np.abs(tr[w, 3] - h0).max()), h_switch=h0)
    ekf = d["ekf"]

    def step(tc):
        """Largest EKF position change between two samples within 1 s of tc, less the true motion between them."""
        if not len(ekf):
            return None
        te_ = ekf[:, 0] - te
        sel = np.where(np.abs(te_ - tc) <= 1.0)[0]
        best = 0.0
        for i, j in zip(sel[:-1], sel[1:]):
            de = float(np.hypot(*(ekf[j, 1:3] - ekf[i, 1:3])))
            tn = [np.interp(ekf[k, 0] - te, t, tr[:, c]) for k in (i, j) for c in (1, 2)]
            best = max(best, de - math.hypot(tn[2] - tn[0], tn[3] - tn[1]))
        return best

    def moved(tc, span=5.0):
        sel = (t >= tc) & (t <= tc + span)
        p0 = [np.interp(tc, t, tr[:, c]) for c in (1, 2)]
        return float(np.hypot(tr[sel, 1] - p0[0], tr[sel, 2] - p0[1]).max()) if sel.any() else None

    m["ekf_step_switch"] = step(0.0)
    rows = onboard_rows(label)
    rel = lambda x: round(float(x["t"]) - te, 2)                                          # noqa: E731
    notes = [(rel(x), x["note"]) for x in rows if x["note"] and not x["note"].startswith("VISO_DELAY_MS advice")]
    m["notes"] = notes
    to2 = [a for a, n in notes if "source set 2 (our position only): ACCEPTED" in n]
    to1 = [a for a, n in notes if "source set 1 (GNSS): ACCEPTED" in n]
    if to2:
        m.update(t_set2=to2[0], ekf_step_set2=step(to2[0]), move5_set2=moved(to2[0]))
    if to1:
        m.update(t_set1=to1[0], ekf_step_set1=step(to1[0]), move5_set1=moved(to1[0]),
                 max_after_set1=float(dist[(t >= to1[0]) & (t <= WIN)].max()) if (t >= to1[0]).any() else None)
    sent = [float(x["t"]) - te for x in rows if x["sent"] == "1"]
    m["fixes_in_window"] = sum(1 for s in sent if 0 <= s <= WIN)
    drop = [a for a, n in notes if "video dropped for" in n]
    if drop:
        m["drop_at"] = drop[0]
        m["max_from_drop"] = float(dist[(t >= drop[0]) & (t <= WIN)].max())
    m["fc_texts"] = [(a, x) for a, x in texts if a >= -60]
    return m


def glitch_window(label, a, b):
    d = np.load(os.path.join(RUNS, label + ".npz"))
    tr, hp, te = d["truth"], d["hold_pt"], float(d["t_engage"])
    t = tr[:, 0] - te
    dist = np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    sel = (t >= a) & (t <= b)
    ekf = d["ekf"]
    es = (ekf[:, 0] - te >= a - 1) & (ekf[:, 0] - te <= b) if len(ekf) else None
    ekf_move = float(np.ptp(ekf[es, 1])) if es is not None and es.any() else None
    return dict(rms=float(np.sqrt(np.mean(dist[sel] ** 2))), max=float(dist[sel].max()), ekf_north_span=ekf_move)


def one(label):
    import sitl_ap as A
    links, c = [], case(label)

    class Link(A.APLink):        # keeps every FC text; injects the GPS glitch GLITCH_AT s after the switch
        def __init__(self, cc, port):
            super().__init__(cc, port)
            links.append(self)
            if c["glitch_deg"]:
                threading.Thread(target=self._glitch, daemon=True).start()

        def _glitch(self):
            if c["ramp"]:                  # RAMP_AT s after the switch, 1 m/s north for RAMP_S s, written at 2 Hz
                done = -1.0
                while not self.stop_all:
                    tt = self.now() - self.t_engage if self.t_engage < math.inf else -1.0
                    if tt >= RAMP_AT and tt - done >= 0.5:
                        frac = min(1.0, (tt - RAMP_AT) / RAMP_S)
                        self.param("SIM_GPS_GLITCH_X", c["glitch_deg"] * frac)
                        done = tt
                        if frac >= 1.0:
                            self.texts.append((self.now(), f"(supervisor) GNSS ramp done: {GLITCH_M:g} m north"))
                            return
                    time.sleep(0.02)
                return
            while not self.stop_all:
                if self.t_engage < math.inf and self.now() - self.t_engage >= GLITCH_AT:
                    self.param("SIM_GPS_GLITCH_X", c["glitch_deg"])
                    self.texts.append((self.now(), f"(supervisor) SIM_GPS_GLITCH_X {c['glitch_deg']:.7f} deg = "
                                                   f"{GLITCH_M:g} m north"))
                    return
                time.sleep(0.02)

    A.APLink = Link
    t_wall, load0 = time.time(), os.getloadavg()
    try:
        res = A.run_case({k: v for k, v in c.items() if k not in ("glitch_deg", "level", "ramp")})
    except Exception as e:
        res = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")])
    r = {k: res.get(k) for k in ("case", "rms", "max", "pct_frames_fix", "fix_rate_hz", "events", "hold_s_flown",
                                   "climb_s", "age_ms_p50", "reanchors", "onboard_exit", "ekf_yaw_minus_true_deg")}
    r.update(level=c["level"], height=c["height"], seed=c["seed"], glitch=bool(c["glitch_deg"]),
             settings={k: v for k, v in c["settings"].items() if k != "params_ardupilot"},
             params_gnss={k: v for k, v in c["settings"].get("params_ardupilot", {}).items() if k.startswith(("EK3", "GPS"))},
             wall_s=round(time.time() - t_wall), load_start=load0)
    if links:
        k = links[-1]
        te = k.t_engage if k.t_engage < math.inf else 0.0
        r["texts_all"] = [(round(t - te, 2), x) for t, x in k.texts]
        if k.t_engage < math.inf and res.get("hold_s_flown"):
            try:
                r.update(metrics(label, r["texts_all"]))
                if r["glitch"]:
                    at = RAMP_AT if c["ramp"] else GLITCH_AT
                    r["glitch_before"] = glitch_window(label, 0, at)
                    r["glitch_after"] = glitch_window(label, at, WIN)
                    r["glitch_kind"] = f"ramp 1 m/s from {RAMP_AT:g} s to {RAMP_AT + RAMP_S:g} s" if c["ramp"] else \
                        f"step at {GLITCH_AT:g} s"
            except Exception as e:
                r["metrics_error"] = f"{type(e).__name__}: {e}"
            try:
                ev = bin_events(label)
                r["bin_ev"] = None if ev is None else [(round(a - te, 2), k_, i) for a, k_, i in ev]
                if ev is not None:
                    r["innov"] = innov(os.path.join(RUNS, label + ".BIN"), te, r.get("t_set2"))
            except Exception as e:
                r["bin_error"] = f"{type(e).__name__}: {e}"
    try:
        allr = json.load(open(OUT))
    except (OSError, ValueError):
        allr = {}
    allr[label] = r
    json.dump(allr, open(OUT, "w"), indent=1, default=str)
    table()
    print(json.dumps({k: v for k, v in r.items() if k not in ("texts_all",)}, default=str)[:4000])


def table():
    try:
        allr = json.load(open(OUT))
    except (OSError, ValueError):
        return
    f = lambda r, x, n=8, p=2: (f"{r[x]:{n}.{p}f}" if isinstance(r.get(x), (int, float)) else "-".rjust(n))  # noqa: E731
    cols = [("rms30", "rms m"), ("max30", "max m"), ("ekf_step_set2", "stepSet2"), ("t_set2", "set2 at"),
            ("ekf_step_switch", "stepSw"), ("herr30", "herr m"), ("ekf_step_set1", "stepSet1"), ("t_set1", "set1 at"),
            ("max_after_set1", "maxAft1"), ("fixes_in_window", "fixes")]
    lines = ["run name".ljust(26) + "level".ljust(9) + "h m".rjust(5) + "seed".rjust(5) + "".join(n.rjust(9) for _, n in cols)
             + "  events (s after the switch)"]
    order = {"none": 0, "setup": 1, "fallback": 2, "velocity": 3, "gps": 4}
    for k in sorted(allr, key=lambda k: (allr[k].get("glitch", False), allr[k].get("height") or 0,
                                         order.get(allr[k].get("level"), 9), k)):
        r = allr[k]
        ev = [f"{t:g}s {w}" for t, w in (r.get("events") or [])[1:]]
        ev += [f"{t:g}s {w}" for t, w in r.get("notes", []) if "source" in w or "WARN" in w or "video" in w]
        ev += [f"{t:g}s FC '{x}'" for t, x in r.get("fc_texts", []) if t > -30 and "PreArm" not in x][:6]
        lines.append(k.ljust(26) + str(r.get("level")).ljust(9) + f"{r.get('height', 0):5.0f}{r.get('seed', 0):5d}" +
                     "".join(f(r, x, 9, 0 if x == "fixes_in_window" else 2) for x, _ in cols) + "  " + "; ".join(ev)[:500])
    g = [k for k in sorted(allr) if allr[k].get("glitch_after")]
    if g:
        lines += ["", f"GPS error {GLITCH_M:g} m north (_glitch: a step at {GLITCH_AT:g} s; _ramp: 1 m/s from {RAMP_AT:g} s to "
                  f"{RAMP_AT + RAMP_S:g} s after the switch): true distance from the switch position (rms / max m) and the "
                  "span of the EKF north position, before and after the error starts (before: 0-10 s or 0-5 s):",
                  "run name".ljust(26) + "before rms/max".rjust(18) + "after rms/max".rjust(18) + "EKF N span bef".rjust(17)
                  + "EKF N span aft".rjust(18)]
        for k in g:
            a, b = allr[k]["glitch_before"], allr[k]["glitch_after"]
            fmt = lambda x: "-" if x is None else f"{x:.2f}"                                  # noqa: E731
            lines.append(k.ljust(26) + f"{a['rms']:.2f} / {a['max']:.2f}".rjust(18) + f"{b['rms']:.2f} / {b['max']:.2f}".rjust(18)
                         + fmt(a["ekf_north_span"]).rjust(17) + fmt(b["ekf_north_span"]).rjust(18))
    lines += ["", __doc__.split("Measures", 1)[1].strip().replace("\n", " "),
              "stepSet2/stepSet1: EKF step at the FC's ACCEPTED reply to source set 2/1; set2 at / set1 at: its time (s after",
              "the switch; negative = before). stepSw: EKF step at the pilot's switch. maxAft1: largest distance from the",
              "switch position after set 1 (fallback). fixes: fixes sent in the 30 s. Run files: apsim/runs/<run name>.*."]
    open(TXT, "w").write("\n".join(lines) + "\n")


def sitl_running():
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


def run(labels):
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    for label in labels:
        try:
            if label in json.load(open(OUT)):
                print("skip (done)", label, flush=True)
                continue
        except (OSError, ValueError):
            pass
        while True:
            if time.strftime("%H:%M") >= DEADLINE and time.strftime("%H:%M") < "12:00":
                print("deadline", DEADLINE, "reached; stop", flush=True)
                return
            if not sitl_running() and os.getloadavg()[0] < LOAD_MAX:
                break
            print(time.strftime("%H:%M:%S"), f"wait: load {os.getloadavg()[0]:.1f}, sitl {len(sitl_running())}", flush=True)
            time.sleep(15)
        print(time.strftime("%H:%M:%S"), "start", label, "load", os.getloadavg(), flush=True)
        log = open(os.path.join(HERE, "logs", f"gnss_{label}.txt"), "w")
        try:
            subprocess.run([PY, os.path.abspath(__file__), "one", label], stdout=log, stderr=subprocess.STDOUT, timeout=300)
        except subprocess.TimeoutExpired:
            print("TIMEOUT (300 s)", label, flush=True)
        subprocess.run(["sh", os.path.join(HERE, "run_sitl.sh"), "stop", str(INSTANCE)], capture_output=True)
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
    elif a[0] == "post":                     # BIN events again from the copied logs (runs/<label>.BIN)
        allr = json.load(open(OUT))
        for k, r in allr.items():
            p = os.path.join(RUNS, k + ".BIN")
            if os.path.exists(p) and os.path.exists(os.path.join(RUNS, k + ".npz")):
                te = float(np.load(os.path.join(RUNS, k + ".npz"))["t_engage"])
                r["bin_ev"] = [(round(a_ - te, 2), k_, i) for a_, k_, i in read_bin(p)]
                r["innov"] = innov(p, te, r.get("t_set2"))
        json.dump(allr, open(OUT, "w"), indent=1, default=str)
        table()
    elif a[0] == "case":
        for x in a[1:]:
            print(x, case(x))
