import os
import csv, glob, json, sys
import numpy as np
P = os.environ["PROJECT"]
for h in (10, 30, 60, 100):
    lab = f"reh_jd_wig_{h}m_s31"
    js = json.load(open(f"{P}/apsim/runs/{lab}.json"))
    rows = list(csv.DictReader(open(f"{P}/apsim/runs/{lab}_onboard.csv")))
    ts = [float(r["t"]) for r in rows if r["t"]]
    eng = [(r["t"], r["note"]) for r in rows if r["note"] and ("engaged" in r["note"] or "released" in r["note"] or "FC mode" in r["note"])]
    modes = {}
    for r in rows:
        if r["t"]:
            m = r["mode"]; modes.setdefault(m, [float(r["t"]), float(r["t"])]); modes[m][1] = float(r["t"])
    # first row with height > 1 m
    up = next((float(r["t"]) for r in rows if r["t"] and r["height"] and float(r["height"]) > 1.0), None)
    print(f"== {h} m: wiggle_t {js.get('wiggle_t')} climb_s {js.get('climb_s')} log t {ts[0]:.2f}..{ts[-1]:.2f} rows {len(rows)} empty-t rows {sum(1 for r in rows if not r['t'])}")
    print("   notes:", eng[:6])
    print("   modes:", modes, "first height>1m at", up)
    for f in sorted(glob.glob(f"{P}/integ/rehearsal_2026-11-20/runs/{lab}_raw/raw_*.npz")):
        d = np.load(f)
        tg, att = d["t_grab"], d["att"]
        print(f"   {f.split('/')[-1]}: {len(tg)} frames t_grab {tg[0]:.2f}..{tg[-1]:.2f}  att {att[0,0]:.2f}..{att[-1,0]:.2f} ({len(att)})  imgs {d['imgs'].shape}")
