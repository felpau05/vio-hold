#!/usr/bin/env python3
"""One ArduCopter SITL closed-loop case (apsim/sitl_ap.run_case), written to integ/justice_d/closed/<label>.json.

It does not call sitl_ap.save(), so apsim/results.json and apsim/results.txt stay as they are. It refuses to write
over an existing result, and it waits until the 1-minute load is under 4 before it starts (busy laptop = wrong
closed-loop results). A failed run is written too (with its error).

Run: ~/.venvs/vio/bin/python integ/justice_d/run_ap.py <label> <height> <wind> [key=value ...]
     e.g. integ/justice_d/run_ap.py jd_apb_100m_w20g40_s11 100 w20g40 video=JUSTICE_D seed=11
"""
import json
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "apsim"), os.path.join(PROJ, "px4sim")]
OUT = os.path.join(HERE, "closed")
LOAD_MAX = 4.0


def wait_load(say=print):
    while True:
        one = os.getloadavg()[0]
        if one < LOAD_MAX:
            return one
        say(f"The 1-minute load is {one:.2f}. It must be under {LOAD_MAX}. The run waits 30 s.")
        time.sleep(30)


def main(argv):
    label, height, wind = argv[0], float(argv[1]), argv[2]
    path = os.path.join(OUT, label + ".json")
    if os.path.exists(path):
        sys.exit(f"{path} exists. Give the run a new label. Do not delete the old file.")
    case = dict(variant="apb", height=height, wind=wind, hold_s=60.0, label=label)
    for kv in argv[3:]:
        k, v = kv.split("=", 1)
        try:
            case[k] = json.loads(v)
        except ValueError:
            case[k] = v
    os.makedirs(OUT, exist_ok=True)
    load = wait_load()
    t0 = time.time()
    import sitl_ap
    try:
        r = sitl_ap.run_case(case)
    except Exception as e:      # a broken run is kept, not lost
        r = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")], trace=traceback.format_exc()[-3000:])
    r["load_1min_at_start"] = round(load, 2)
    r["load_1min_at_end"] = round(os.getloadavg()[0], 2)
    r["wall_s"] = round(time.time() - t0, 1)
    r["started"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0))
    r["case_args"] = case
    json.dump(r, open(path, "w"), indent=1, default=str)
    keys = ("rms", "max", "p95", "fix_rate_hz", "pct_frames_fix", "age_ms_p50", "handover_ms_p50", "handover_ms_p95",
            "proc_ms_mean", "processed", "frames")
    print(label, " ".join(f"{k} {r[k]:.3g}" for k in keys if isinstance(r.get(k), (int, float))), r.get("events"))


if __name__ == "__main__":
    main(sys.argv[1:])
