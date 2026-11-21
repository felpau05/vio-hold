#!/usr/bin/env python3
"""Step 7 "fly" and EVENT_DAY step 7: Justice D flights on ArduCopter SITL (apsim/sitl_ap.run_case), with the
timing wiggle (step 7b) in Altitude mode before the switch, raw recording on, and the video delay set 30 ms too
long before the flight (est_delay_err_s). Then a 60 s hold in Loiter, wind 20 gusting 40 km/h.

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/apflights.py SEED HEIGHT [HEIGHT ...]

Run files: apsim/runs/<label>.* (label reh_jd_wig_<h>m_s<seed>); raw chunks and the program's settings in
integ/rehearsal_2026-11-20/runs/<label>_raw/. Result: integ/rehearsal_2026-11-20/<label>.json. Not saved to
apsim/results.json (the project's result table). One SITL instance (2) at a time.
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(PROJ, "apsim"), os.path.join(PROJ, "px4sim"), PROJ]
import sitl_ap as A   # noqa: E402


def main():
    seed = int(sys.argv[1])
    for h in sys.argv[2:]:
        label = f"reh_jd_wig_{h}m_s{seed}"
        raw = os.path.join(HERE, "runs", label + "_raw")
        while os.getloadavg()[0] > 4.0:
            print(f"load {os.getloadavg()[0]:.2f} > 4: wait 30 s", flush=True)
            time.sleep(30)
        load = os.getloadavg()
        case = dict(variant="apb", height=float(h), wind="w20g40", hold_s=60.0, seed=seed, label=label, instance=2,
                    video="JUSTICE_D", wiggle_s=8.0, est_delay_err_s=0.030,
                    settings=dict(raw_record=True, log_dir=raw, min_height_m=3.0), hold_cfg=dict(h_min=2.0))
        t1 = time.time()
        try:
            r = A.run_case(case, quiet=True)
        except Exception as e:
            r = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")])
        r["load_at_start"] = [round(x, 2) for x in load]
        r["wall_s"] = round(time.time() - t1)
        json.dump(r, open(os.path.join(HERE, label + ".json"), "w"), indent=1, default=str)
        keys = ("rms", "p95", "max", "fixes", "fix_rate_hz", "reanchors", "wiggle_t", "climb_s", "age_ms_p50", "events")
        print(label, json.dumps({k: r.get(k) for k in keys}, default=str)[:900], f"load {r['load_at_start']}", flush=True)


if __name__ == "__main__":
    main()
