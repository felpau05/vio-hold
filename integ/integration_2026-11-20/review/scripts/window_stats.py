"""Attitude statistics of 15 s windows (the timing window) along each rehearsal flight: RMS turn rate as calib.Rec
computes it (roll, pitch, yaw on a 0.1 s grid), and the spread (std) of roll and pitch angle. Labels each window by
the wiggle seconds inside it and by the FC mode of its rows."""
import os
import csv, glob, json
import numpy as np
P = os.environ["PROJECT"]
for h in (10, 30, 60, 100):
    lab = f"reh_jd_wig_{h}m_s31"
    js = json.load(open(f"{P}/apsim/runs/{lab}.json"))
    ws, we = js["wiggle_t"]
    att = np.concatenate([np.load(f)["att"] for f in sorted(glob.glob(f"{P}/integ/rehearsal_2026-11-20/runs/{lab}_raw/raw_*.npz"))])
    att = att[np.unique(att[:, 0], return_index=True)[1]]
    rows = [r for r in csv.DictReader(open(f"{P}/apsim/runs/{lab}_onboard.csv")) if r["t"]]
    lt = np.array([float(r["t"]) for r in rows]) + 0.070
    hold = np.array([r["mode"] == "LOITER" for r in rows])
    print(f"== {h} m  wiggle {ws:.1f}..{we:.1f}  switch (t_grab) {lt[hold][0]:.1f}")
    print("   window(t_grab)   wiggle_s  hold_rows  rate_dps  roll_std  pitch_std  (deg)")
    for e in np.arange(lt[0] + 16, lt[-1], 2.5):
        a, b = e - 15, e
        tt = np.arange(a, b, 0.1)
        yaw = np.unwrap(att[:, 3])
        ang = [np.interp(tt, att[:, 0], c) for c in (att[:, 1], att[:, 2], yaw)]
        w = np.diff(np.c_[ang].T, axis=0) / 0.1
        rate = float(np.degrees(np.sqrt(np.sum(np.mean(w ** 2, axis=0)))))
        wig = max(0.0, min(b, we) - max(a, ws))
        m = (lt >= a) & (lt <= b)
        print(f"   {a:6.1f}..{b:6.1f}   {wig:5.1f}   {hold[m].mean():5.2f}   {rate:6.1f}   {np.degrees(np.std(ang[0])):6.2f}   {np.degrees(np.std(ang[1])):6.2f}")
