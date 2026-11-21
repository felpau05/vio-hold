"""Task 2a on the real files: raw t_grab minus the nearest run-log t, for every raw frame of each flight."""
import os
import csv, glob, json
import numpy as np
P = os.environ["PROJECT"]
for h in (10, 30, 60, 100):
    lab = f"reh_jd_wig_{h}m_s31"
    s = json.load(open(f"{P}/apsim/runs/{lab}_settings.json"))
    lt = np.array([float(r["t"]) for r in csv.DictReader(open(f"{P}/apsim/runs/{lab}_onboard.csv")) if r["t"]])
    tg = np.concatenate([np.load(f)["t_grab"] for f in sorted(glob.glob(f"{P}/integ/rehearsal_2026-11-20/runs/{lab}_raw/raw_*.npz"))])
    i = np.clip(np.searchsorted(lt, tg - s["video_delay_s"]), 1, len(lt) - 1)
    near = np.where(np.abs(lt[i] - (tg - s["video_delay_s"])) < np.abs(lt[i - 1] - (tg - s["video_delay_s"])), lt[i], lt[i - 1])
    d = tg - near
    print(f"{h:3d} m: {len(tg)} raw frames, {len(lt)} log rows with t; t_grab - nearest log t: min {d.min()*1e3:.2f} ms, "
          f"max {d.max()*1e3:.2f} ms (video_delay_s in the flight settings {s['video_delay_s']*1e3:.0f} ms)")
