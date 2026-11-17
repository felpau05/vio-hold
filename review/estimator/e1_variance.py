#!/usr/bin/env python3
"""Check 1: is the variance Hold sends honest, and which error sources does it miss?

Part A: 6 seeds x 4 heights over ottawa2019fall (20 s hover, sim_ground settings). For each run:
        RMS, false-fix (same test as sim_ground), the run's constant offset ("anchor" = mean error
        vector) and the frame-to-frame part ("jitter" = error minus that mean).
Part B: ablation at 300 m: switch off one sim error source at a time (16 seeds x 3 s).
Writes e1_variance.json for e1b_honest_var.py.
Run: ~/.venvs/vio/bin/python e1_variance.py      (about 100 s on 16 threads)
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

SITE = "ottawa2019fall"


def job(h0, seed, T, cfgkw, tag):
    cfg = C.Cfg(**{k: (np.array(v) if k == "att_bias" else v) for k, v in cfgkw.items()})
    rec, _ = C.scenario(SITE, h0, T=T, cfg=cfg, seed=seed)
    d = C.run_stats(rec, h0)
    d.update(tag=tag, seed=seed)
    return d


ABL = {
    "all (sim_ground)": {},
    "no vibration": dict(vib_deg=0.0),
    "no FC attitude noise": dict(att_noise=0.0),
    "no timing error/lag": dict(frame_t_err=0.0, att_lag=0.0),
    "no attitude bias": dict(att_bias=[0, 0, 0]),
    "no video noise": dict(video_noise=0.0),
    "all off": dict(vib_deg=0.0, att_noise=0.0, frame_t_err=0.0, att_lag=0.0, att_bias=[0, 0, 0],
                    video_noise=0.0, baro_bias=0.0, baro_noise=0.0),
}

if __name__ == "__main__":
    heights, seeds = (100, 200, 300, 400), range(1, 7)
    jobs = [dict(h0=h, seed=s, T=20.0, cfgkw={}, tag="A") for h in heights for s in seeds]
    jobs += [dict(h0=300, seed=s, T=3.0, cfgkw=kw, tag=name) for name, kw in ABL.items() for s in range(11, 27)]
    res = C.pmap(job, jobs)
    json.dump(res, open(os.path.join(C.HERE, "e1_variance.json"), "w"))

    print("PART A  ottawa2019fall, 20 s hover, 6 seeds per height (sim_ground settings)")
    print(" h(m)  RMS per seed (m)                         false-fix per seed            "
          "anchor offset RMS  jitter RMS  sent var (m^2, trace)  pooled false-fix")
    for h in heights:
        rs = [r for r in res if r["tag"] == "A" and r["h"] == h]
        rms = " ".join(f"{r['rms']:4.2f}" for r in rs)
        ff = " ".join(f"{100 * r['false_fix']:4.0f}%" for r in rs)
        anc = math.sqrt(np.mean([r["bias"] ** 2 for r in rs]))
        jit = math.sqrt(np.mean([r["jit"] ** 2 for r in rs]))
        pooled = np.mean([r["false_fix"] for r in rs])
        print(f" {h:4d}  {rms:40s} {ff:30s} {anc:6.2f}            {jit:5.2f}       "
              f"{np.mean([r['var_mean'] for r in rs]):7.3f}               {pooled:5.1%}")
    print("  -> per-axis error/height: anchor sigma = anchor RMS/(sqrt2 h), frame sigma = jitter RMS/(sqrt2 h)")
    for h in heights:
        rs = [r for r in res if r["tag"] == "A" and r["h"] == h]
        anc = math.sqrt(np.mean([r["bias"] ** 2 for r in rs])) / math.sqrt(2) / h
        jit = math.sqrt(np.mean([r["jit"] ** 2 for r in rs])) / math.sqrt(2) / h
        print(f"     {h:4d} m: anchor {math.degrees(anc):.3f} deg, frame {math.degrees(jit):.3f} deg "
              f"(Hold assumes ATT_NOISE 0.150 deg once, as trace = 0.106 deg per axis)")

    print("\nPART B  ablation at 300 m, 3 s, 16 seeds each (pooled; each run gives one anchor sample)")
    print(" source switched off            RMS(m)  anchor RMS  jitter RMS  false-fix")
    for name in ABL:
        rs = [r for r in res if r["tag"] == name]
        e2 = np.concatenate([r["e2"] for r in rs])
        print(f" {name:30s} {math.sqrt(e2.mean()):5.2f}   {math.sqrt(np.mean([r['bias']**2 for r in rs])):5.2f}"
              f"       {math.sqrt(np.mean([r['jit']**2 for r in rs])):5.2f}      "
              f"{np.mean([r['false_fix'] for r in rs]):5.1%}")
