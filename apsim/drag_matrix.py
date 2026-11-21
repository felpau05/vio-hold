#!/usr/bin/env python3
"""EKF3 drag fusion A/B on ArduCopter SITL: does the EKF dead-reckon through a fix gap, and does it hurt the hold?
  ~/.venvs/vio/bin/python apsim/drag_matrix.py run <label> [<label> ...]   one at a time, waits for a free SITL
  ~/.venvs/vio/bin/python apsim/drag_matrix.py table
  ~/.venvs/vio/bin/python apsim/drag_matrix.py coefs                          the derivation of DRAG below
Each case is apsim/sitl_ap.run_case (variant apb) with extra ArduPilot parameters in case["ap"]. Results go to
apsim/drag_results.json / drag_results.txt (not results.json). Run files: apsim/runs/<label>.*."""
import fcntl
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.expanduser("~/.venvs/vio/bin/python")
OUT, TXT = os.path.join(HERE, "drag_results.json"), os.path.join(HERE, "drag_results.txt")
INSTANCE = 90          # ports 16800-16802: away from matrix_ap.py instances 10-77 and instance 0
DEADLINE = "05:55"     # no new run starts after this local time

# SITL quad model (libraries/SITL/SIM_Frame.h:81-130, default_model): mass 3.0 kg, refSpd 15.08 m/s at refAngle 45 deg,
# refAlt 593 m, disc_area 0.385 m2, mdrag_coef 0.2. SIM_Frame.cpp:466-481 splits the 45 deg drag into momentum drag
# and areaCd; the SITL log prints "Suggested EK3_DRAG_BCOEF_* = 17.209, EK3_DRAG_MCOEF = 0.209".
M, G, REF_SPD, REF_ANG, REF_ALT, DISC, MDRAG = 3.0, 9.80665, 15.08, 45.0, 593.0, 0.385, 0.2


def rho(alt_amsl):     # ISA troposphere, as AP_Baro::get_air_density_for_alt_amsl
    return 1.225 * (1 - 0.0065 * alt_amsl / 288.15) ** 4.2559


def coefs(alt_amsl=70.0 + 60.0, tilt_deg=10.0):
    """BCOEF: SITL body drag is areaCd*0.5*rho*v^2 per body axis (SIM_Frame.cpp:582-594), EKF predicts
    0.5*rho*v^2/BCOEF (AP_NavEKF3_AirDataFusion.cpp:540): BCOEF = mass/areaCd, the same at any tilt.
    MCOEF: SITL momentum drag summed over 4 motors is mdrag_coef*v*sqrt(rho*disc_area*thrust) (SIM_Motor.cpp:111-114),
    EKF predicts mcoef*v (AirDataFusion.cpp:535, density ratio ~1 here). SITL's printed 0.209 is at its 45 deg
    reference thrust; at hover thrust (m g / cos(tilt)) it is larger."""
    r0 = rho(REF_ALT)
    c = math.cos(math.radians(REF_ANG))
    f_drag = M * G * math.tan(math.radians(REF_ANG))
    f_mom = c * MDRAG * REF_SPD * c * math.sqrt(M * G / c * r0 * DISC)
    area_cd = (f_drag - f_mom) / (0.5 * r0 * REF_SPD ** 2)
    r = rho(alt_amsl)
    m_hover = MDRAG * math.sqrt(r * DISC * M * G / math.cos(math.radians(tilt_deg))) / M
    return dict(bcoef=M / area_cd, mcoef_sitl_print=f_mom / (M * REF_SPD * c) * math.sqrt(1.225 / r0),
                mcoef_hover=m_hover, rho=r)


BCOEF, MCOEF = 17.2, 0.25     # from coefs(): 17.21 and 0.247-0.252 at 100-170 m AMSL, tilt 5-15 deg


def drag(scale=1.0):
    """EK3 drag parameters for a modelled drag force of scale x ours (BCOEF divides, MCOEF multiplies).
    FS_DR_ENABLE 0: the default 2 (RTL) would leave Loiter as soon as the EKF dead-reckons (events.cpp:357-372)."""
    return dict(EK3_DRAG_BCOEF_X=round(BCOEF / scale, 2), EK3_DRAG_BCOEF_Y=round(BCOEF / scale, 2),
                EK3_DRAG_MCOEF=round(MCOEF * scale, 3), FS_DR_ENABLE=0)


DRAGS = {"off": {}, "on": drag(1.0), "half": drag(0.5), "dbl": drag(2.0)}


def case(label):
    """dr_<site>_<drag>_s<seed>: gap = hold-out field 100 m; n60/n30 = tuning site; out60 = 3 s video outage at 60 m;
    gapX = hold-out field 100 m with a 20 s video outage from 20 s (a gap of known length)."""
    _, site, d, s = label.split("_")
    seed = int(s[1:])
    base = dict(gap=dict(height=100.0, ground="holdout_field"), n60=dict(height=60.0), n30=dict(height=30.0),
                out60=dict(height=60.0, outage=[20.0, 3.0]),
                gapX=dict(height=100.0, ground="holdout_field", outage=[20.0, 20.0]))[site]
    return dict(variant="apb", wind="w20g40", hold_s=60.0, seed=seed, label=label, instance=INSTANCE,
                ap=dict(DRAGS[d]), **base)


def sitl_running():
    """ArduCopter SITL processes on this machine (the container's processes show in the host /proc)."""
    out = []
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                a = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if a and a[0].endswith(b"arducopter"):
                out.append(b" ".join(a).decode(errors="replace")[:160])
    return out


KEYS = ("Dead Reckoning", "aiding", "EKF variance", "ailsafe", "Mode", "lane", "primary", "origin", "GPS", "yaw")


def one(label):
    sys.path.insert(0, HERE)
    import sitl_ap as A
    links = []

    class Link(A.APLink):       # keep the link so that all FC texts are kept, not only the last 30
        def __init__(self, c, port):
            super().__init__(c, port)
            links.append(self)

    A.APLink = Link
    c = case(label)
    t_wall = time.time()
    try:
        res = A.run_case(c)
    except Exception as e:      # a broken run is logged, not lost
        res = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")])
    r = {k: res.get(k) for k in ("case", "rms", "p95", "max", "end", "pct_frames_fix", "fix_rate_hz", "events",
                                   "hold_s_flown", "ap_params", "climb_s", "age_ms_p50", "drift_m_per_min")}
    r.update(drag=label.split("_")[2], seed=c["seed"], ap=c["ap"], wall_s=round(time.time() - t_wall),
             loadavg=os.getloadavg())
    if links:
        k = links[-1]
        te = k.t_engage if k.t_engage < math.inf else (k.texts[-1][0] if k.texts else 0.0)
        r["texts_all"] = [(round(t - te, 2), x) for t, x in k.texts]
        r["key_texts"] = [(round(t - te, 1), x) for t, x in k.texts if any(w in x for w in KEYS)]
        if k.t_engage < math.inf and res.get("hold_s_flown"):
            t_end = k.t_engage + res["hold_s_flown"]
            runs = A.RUNS
            logs = sorted(os.path.join(runs, f) for f in os.listdir(runs)
                          if f.startswith(label + "_onboard") and f.endswith(".csv"))
            ts = sorted(float(x["t"]) for x in A.P.onboard_log(logs, k.t_engage, t_end) if x["sent"] == "1")
            edges = [k.t_engage] + ts + [t_end]
            gaps = [(b - a, a - k.t_engage) for a, b in zip(edges, edges[1:])]
            r["fix_gap_max_s"], r["fix_gap_at_s"] = (round(x, 2) for x in max(gaps))
            r["fix_gaps_over_2s"] = [(round(a, 1), round(g, 1)) for g, a in gaps if g > 2.0]
    with open(OUT + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            allr = json.load(open(OUT))
        except (OSError, ValueError):
            allr = {}
        allr[label] = r
        json.dump(allr, open(OUT, "w"), indent=1, default=str)
    table()
    print(json.dumps({k: v for k, v in r.items() if k not in ("texts_all", "ap_params")}, default=str))


def table():
    try:
        allr = json.load(open(OUT))
    except (OSError, ValueError):
        return
    lines = ["case".ljust(24) + "drag".rjust(6) + "rms m".rjust(8) + "max m".rjust(8) + "% fix".rjust(7) +
             "gap s".rjust(7) + "  events (s after hold start)"]
    for k in sorted(allr):
        r = allr[k]
        f = lambda x, n=8: (f"{r[x]:{n}.2f}" if isinstance(r.get(x), (int, float)) else "-".rjust(n))
        ev = [f"{t:g}s {w}" for t, w in (r.get("events") or [])[1:]]
        ev += [f"{t:g}s {w}" for t, w in r.get("key_texts", []) if t >= 0 and not w.startswith("EKF3 IMU1")]
        lines.append(k.ljust(24) + r["drag"].rjust(6) + f("rms") + f("max") + f("pct_frames_fix", 7) +
                     f("fix_gap_max_s", 7) + "  " + "; ".join(ev)[:300])
    lines += ["", "apsim/drag_matrix.py. drag: off = current params_ardupilot; on = " + json.dumps(DRAGS["on"]) +
              "; half/dbl = drag force x0.5 / x2. gap s = longest time without a sent fix in the hold.",
              "Hold 60 s, w20g40, variant apb (onboard program, cpu_model fixed). Events: sitl_ap events + FC texts.",
              "FS_DR_ENABLE: off runs keep the default 2 (RTL; no effect without drag, the EKF never dead-reckons);",
              "on/half/dbl runs set 0 (checked in each dataflash log). gapX: a run ends at a fly-away, so its gap s",
              "is cut there (the video outage is 20 s). gap_* (hold-out field, no forced outage) had no gap > 2.2 s."]
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
        while True:
            if time.strftime("%H:%M") >= DEADLINE:
                print("deadline", DEADLINE, "reached; stop", flush=True)
                return
            busy = sitl_running()
            if not busy:
                break
            print(time.strftime("%H:%M:%S"), "SITL busy, wait:", busy[:2], flush=True)
            time.sleep(30)
        print(time.strftime("%H:%M:%S"), "start", label, "load", os.getloadavg(), flush=True)
        log = open(os.path.join(HERE, "logs", f"drag_{label}.txt"), "w")
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
    elif a[0] == "coefs":
        for alt, tilt in ((70 + 30, 5), (70 + 60, 10), (70 + 100, 15)):
            print(alt, tilt, {k: round(v, 4) for k, v in coefs(alt, tilt).items()})
