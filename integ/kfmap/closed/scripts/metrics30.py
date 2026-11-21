#!/usr/bin/env python3
"""Closed-loop table of integ/kfmap: the SITL runs of the copies kf_off, kf_dl, kf_both, measured over the 30 s from the
switch to LOITER (and over the whole 60 s hold), with the formulas of apsim/engage_matrix.metrics: true horizontal
distance from the hold point (RMS, largest), longest time without a sent fix (fix time + its age, as the runner), the
re-anchors and the relocalization notes (KF_MAP), the 1-min load at the start and the end of the run.
Run: KF_CLOSED=<folder of the copies> ~/.venvs/vio/bin/python integ/kfmap/closed/scripts/metrics30.py
Writes integ/kfmap/closed/table30.txt and table30.json, and copies each run's results entry, onboard log (gzip) and
truth (.npz) to integ/kfmap/closed/runs/<copy>_<label>.*"""
import csv
import glob
import gzip
import json
import os
import shutil
import sys

import numpy as np

SP = os.environ["KF_CLOSED"]
OUT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COPIES = ["kf_off", "kf_dl", "kf_both"]
NAMES = {"kf_off": "off", "kf_dl": "DENSE_LOCAL", "kf_both": "both on"}


def rows_of(runs, label):
    rows = []
    for p in sorted(glob.glob(os.path.join(runs, label + "_onboard*.csv"))):
        rows += [x for x in csv.DictReader(open(p)) if x["t"]]
    return sorted(rows, key=lambda x: float(x["t"]))


def one(copy, label, r):
    runs = os.path.join(SP, copy, "apsim", "runs")
    d = np.load(os.path.join(runs, label + ".npz"))
    tr, hp, te = d["truth"], d["hold_pt"], float(d["t_engage"])
    t, dist = tr[:, 0] - te, np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    rows = rows_of(runs, label)
    T60 = r.get("hold_s_flown") or 60.0
    out = dict(copy=copy, version=NAMES[copy], label=label, load_start=(r.get("load_start") or [None])[0],
               load_end=(r.get("load_end") or [None])[0], hold_s=T60, hold_cfg=r.get("settings_file_hold_cfg"),
               fc_modes=r.get("fc_modes"), fc_texts_hold=r.get("fc_texts_hold"))
    sent = [float(x["t"]) - te + float(x["age_ms"] or 0) / 1e3 for x in rows if x["sent"] == "1"]
    for T, k in ((30.0, "30"), (T60, "60")):
        w = (t >= 0) & (t <= T)
        edges = [0.0] + [s for s in sent if 0 <= s <= T] + [T]
        gaps = [(b - a, a) for a, b in zip(edges, edges[1:])]
        g = max(gaps)
        inh = [x for x in rows if 0 <= float(x["t"]) - te <= T]
        out.update({f"rms{k}": float(np.sqrt(np.mean(dist[w] ** 2))), f"max{k}": float(dist[w].max()),
                    f"gap{k}": g[0], f"gap_at{k}": g[1], f"fix_hz{k}": sum(1 for s in sent if 0 <= s <= T) / T,
                    f"reanchors{k}": sum(1 for x in inh if "re-anchored" in (x["note"] or "")),
                    f"reloc_tries{k}": sum(1 for x in inh if "reloc " in (x["note"] or "")),
                    f"reloc_acc{k}": sum(1 for x in inh if "reloc ACCEPTED" in (x["note"] or ""))})
    out["reloc_notes"] = [(round(float(x["t"]) - te, 2), x["note"]) for x in rows if "reloc " in (x["note"] or "")]
    dst = os.path.join(OUT, "runs")
    os.makedirs(dst, exist_ok=True)
    for p in glob.glob(os.path.join(runs, label + "_onboard*.csv")):
        with open(p, "rb") as f, gzip.open(os.path.join(dst, f"{copy}_{os.path.basename(p)}.gz"), "wb") as g:
            shutil.copyfileobj(f, g)
    shutil.copy(os.path.join(runs, label + ".npz"), os.path.join(dst, f"{copy}_{label}.npz"))
    json.dump(r, open(os.path.join(dst, f"{copy}_{label}_result.json"), "w"), indent=1, default=str)
    return out


def main():
    res = []
    for copy in COPIES:
        try:
            allr = json.load(open(os.path.join(SP, copy, "apsim", "low_ap_results.json")))
        except (OSError, ValueError):
            continue
        for label in sorted(allr):
            try:
                res.append(one(copy, label, allr[label]))
            except (OSError, KeyError, ValueError) as e:
                print("skip", copy, label, e, file=sys.stderr)
    f = lambda x, n=6, p=2: "-".rjust(n) if x is None else f"{x:{n}.{p}f}"   # noqa: E731
    lines = ["case                             version       rms30  max30  gap30 fixHz30 rean30 reloc30 |  rms60  max60  gap60 rean60 reloc60(tries) | load start/end  FC modes"]
    order = {c: i for i, c in enumerate(COPIES)}
    for x in sorted(res, key=lambda x: (x["label"], order[x["copy"]])):
        lines.append(f"{x['label']:32s} {x['version']:12s} {f(x['rms30'])} {f(x['max30'])} {f(x['gap30'])} {f(x['fix_hz30'], 7, 1)}"
                     f" {x['reanchors30']:6d} {x['reloc_acc30']:7d} | {f(x['rms60'])} {f(x['max60'])} {f(x['gap60'])} {x['reanchors60']:6d}"
                     f" {x['reloc_acc60']:4d}({x['reloc_tries60']:3d})    | {f(x['load_start'], 5, 1)} / {f(x['load_end'], 4, 1)}  "
                     f"{[m for _, m in (x['fc_modes'] or [])][-3:]}")
    lines += ["", "rms/max: true horizontal distance from the hold point (m), from the switch to LOITER over 30 s and over the hold",
              "(60 s). gap: longest time without a sent fix (s; fix time + its age). fixHz: sent fixes per s. rean: re-anchors.",
              "reloc: accepted relocalizations (tries in brackets). load: 1-min load (16 cores) at the start / end of the run;",
              "a run that started at load 3 or more is not valid. Files: integ/kfmap/closed/runs/."]
    open(os.path.join(OUT, "table30.txt"), "w").write("\n".join(lines) + "\n")
    json.dump(res, open(os.path.join(OUT, "table30.json"), "w"), indent=1, default=str)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
