#!/usr/bin/env python3
"""tests/bad_pictures.py (its cases, scenes and table, unchanged) for integ/kfgraph/hold_graph.py, graph off and on.

The module name 'hold' is the snapshot copy (integ/kfgraph/hold_snapshot.py), so tests/harness.py does not import the
root hold.py that another agent edits today. harness.Hold is replaced by hold_graph.Hold with kf_graph off or on.
One process, one thread (coordinator rule). Scenes: ottawa2019fall at 100 m (TUNING site) and the E7 blackout at
20, 50 and 100 m, seeds 1-3 (TUNING seeds), as tests/bad_pictures.py.
Run: nice -n 19 ~/.venvs/vio/bin/python integ/kfgraph/bad_graph.py [off|on|snap ...]
Writes integ/kfgraph/results/bad_pictures_<version>.txt and .json (refuses to write over a file).
"""
import os
import sys

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import cv2   # noqa: E402

cv2.setNumThreads(1)
import importlib.util   # noqa: E402
import json   # noqa: E402
import time   # noqa: E402

import numpy as np   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.dont_write_bytecode = True


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


sys.modules["hold"] = load(os.path.join(HERE, "hold_snapshot.py"), "hold")
sys.path.insert(0, os.path.join(PROJ, "tests"))
import harness as X   # noqa: E402
import bad_pictures as B   # noqa: E402

HG = load(os.path.join(HERE, "hold_graph.py"), "hold_graph")
VERSIONS = {"snap": lambda *a: sys.modules["hold"].Hold(*a),
            "off": lambda *a: HG.Hold(*a),
            "on": lambda *a: HG.Hold(*a, kf_graph=True)}


def main():
    names = sys.argv[1:] or ["off", "on"]
    for name in names:
        out_txt = os.path.join(HERE, "results", f"bad_pictures_{name}.txt")
        if os.path.exists(out_txt):
            raise SystemExit(f"{out_txt} exists")
        X.Hold = VERSIONS[name]
        w0 = time.time()
        res = [B.job(case=c, seed=s) for c in B.CASES for s in (1, 2, 3)]
        e7 = [B.job_e7(h0=h, seed=s) for h in (20, 50, 100) for s in (1, 2)]
        lines = [f"version {name} (integ/kfgraph/bad_graph.py; tests/bad_pictures.py cases)",
                 f"Bad video from {B.BAD[0]:.0f} to {B.BAD[1]:.0f} s at 100 m, drone drifting 0.5 m/s east, 3 seeds per case, "
                 "no reanchor() call.",
                 " case                    fixes 5-5.5 s  fixes sent 5.5-10 s  keyframes made 5.5-10 s  fixes back after (s, per seed)  "
                 "RMS 10-14 s (m)  confident-wrong  fixes from broken frames  RMS 5-10 s (m)"]
        for c in B.CASES:
            rs = [r for r in res if r["case"] == c]
            lines.append(f" {c:22s}  {sum(r['sent_bad'] - r['sent_late'] for r in rs):4d}           "
                         f"{sum(r['sent_late'] for r in rs):4d} of {sum(r['frames_late'] for r in rs):4d}"
                         f"          {sum(r['kf_late'] for r in rs):3d}                     "
                         f"{' '.join(f'{r['back']:.2f}' for r in rs):24s}       {np.nanmean([r['rms_after'] for r in rs]):5.2f}"
                         f"            {sum(r['conf_wrong'] for r in rs):3d}           "
                         f"{sum(r['sent_broken'] for r in rs)} of {sum(r['broken'] for r in rs)}"
                         f"                 {np.nanmax([r['rms_bad'] for r in rs] + [np.nan]):5.2f}")
        lines.append("E7: 3 s of static (std 30) from 5 s while flying 1 m/s north; no reanchor() call; 2 seeds")
        lines.append(" h(m)  fixes back after (s)  sent after  RMS after (m)  confident-wrong")
        for h in (20, 50, 100):
            rs = [r for r in e7 if r["h"] == h]
            lines.append(f" {h:4d}  {' '.join(f'{r['back']:.2f}' for r in rs):20s}  {np.mean([r['sent_after'] for r in rs]):6.1%}"
                         f"      {np.nanmean([r['err_after'] for r in rs]):5.2f}           {sum(r['conf_wrong'] for r in rs)}")
        lines.append(f"wall {time.time() - w0:.0f} s")
        out = "\n".join(lines)
        print(out, flush=True)
        open(out_txt, "w").write(out + "\n")
        json.dump(dict(res=res, e7=e7), open(out_txt[:-4] + ".json", "w"), default=float)


if __name__ == "__main__":
    main()
