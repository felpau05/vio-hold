#!/usr/bin/env python3
"""Closed-loop A/B table of integ/kfmap (ArduCopter SITL, copy 'ab' in the scratchpad, labels
lowap_apb_<h>m_<wind>_s<seed>_<version>). Over the 30 s from the switch to LOITER (and over the 60 s hold):
  rms, max     true horizontal distance from the hold point (m; apsim/engage_matrix.metrics)
  gap          longest time without a sent fix (s; fix time + its age, as the runner)
  ferr         RMS of the sent-fix error against the truth, relative to the error at the switch (mean of the last 1 s
               before it), in the FC heading frame (apsim/engage_matrix.fix_err)
  fmax, bad    the largest of these sent-fix errors (m), and the count of sent fixes more than 0.5 m wrong / sent fixes
  abs fmax     the same without removing the error at the switch (absolute, FC heading frame; origin = home)
  yaw err      FC yaw (ATTITUDE) minus the true yaw (SIM_STATE quaternion of the truth log): at the switch, and its
               largest change from the switch over 30 s and 60 s (deg)
  align        FC text 'yaw alignment' before the switch: time (s) and the step of the yaw error over it (error 1 s after
               minus 1 s before, deg)
  extnav       time from the first FC text 'is using external nav data' to the switch (s)
  gate load    1-min load (16 cores) that the driver's gate saw when it started the run (every run: under 3)
  runner load  1-min load at the runner's start (some seconds later, after the SITL start) and at its end
Run: KF_CLOSED=<folder of the copies> ~/.venvs/vio/bin/python integ/kfmap/closed/scripts/metrics_ab.py
Writes integ/kfmap/closed/ab_table.txt and ab_table.json; copies each run (onboard log gzip, truth npz, results entry,
runner json) to integ/kfmap/closed/runs/."""
import csv
import glob
import gzip
import json
import math
import os
import re
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
PROJ = os.path.dirname(os.path.dirname(os.path.dirname(OUT)))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import camera as C   # noqa: E402

SP = os.environ["KF_CLOSED"]
COPY = "ab"
ORDER = ["base", "py", "h15", "b", "pyb", "pybh15", "pyn55"]
NAMES = {"base": "base", "py": "py (DENSE_LOCAL+DENSE_TURN)", "h15": "base, hover 15 s", "b": "(b) gyro_fcyaw",
         "pyb": "py + (b)", "pyn55": "pyn55 (py+NADIR 55)",
         "pybh15": "py + (b) + hover 15 s"}


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def one(label, r):
    runs = os.path.join(SP, COPY, "apsim", "runs")
    d = np.load(os.path.join(runs, label + ".npz"))
    rj = json.load(open(os.path.join(runs, label + ".json")))
    tr, hp, te, att = d["truth"], d["hold_pt"], float(d["t_engage"]), d["att"]
    t, dist = tr[:, 0] - te, np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    rows = []
    for p in sorted(glob.glob(os.path.join(runs, label + "_onboard*.csv"))):
        rows += [x for x in csv.DictReader(open(p)) if x["t"]]
    rows.sort(key=lambda x: float(x["t"]))
    sent = [x for x in rows if x["sent"] == "1"]
    st = [float(x["t"]) - te + float(x["age_ms"] or 0) / 1e3 for x in sent]
    # fix error (engage_matrix.fix_err)
    dy, ro = math.radians(rj["ekf_yaw_minus_true_deg"]), rj["cfg"]["readout_s"]
    tf = np.array([float(x["t"]) for x in sent]) + ro / 2 - te
    pf = np.array([[float(x["north"]), float(x["east"])] for x in sent])
    N, E = np.interp(tf + te, tr[:, 0], tr[:, 1]), np.interp(tf + te, tr[:, 0], tr[:, 2])
    c, s_ = math.cos(dy), math.sin(dy)
    ef = pf - np.c_[c * N - s_ * E, s_ * N + c * E] if len(pf) else np.zeros((0, 2))
    pre = ef[(tf >= -1) & (tf < 0)] if len(pf) else ef
    ef_rel = ef - pre.mean(0) if len(pre) else None
    # FC yaw error against the truth
    ty = C.q_yaw(tr[:, 4:])
    ya = np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3]))
    ye = np.degrees(np.unwrap(wrap(ya - ty)))
    ye0 = float(np.interp(0.0, t, ye))
    texts = r.get("texts_all") or []
    ext = [a for a, x in texts if "is using external nav data" in x]
    al = [a for a, x in texts if "yaw align" in x.lower() and a < 0]
    mv = re.match(r".*_s\d+_([a-z0-9]+)(?:_(ottawa2022|farmland))?$", label)
    out = dict(label=label, version=mv.group(1), ground=mv.group(2) or "ottawa2019fall", load_start=(r.get("load_start") or [None])[0],
               load_end=(r.get("load_end") or [None])[0], hold_s=r.get("hold_s_flown"), hold_cfg=r.get("settings_file_hold_cfg"),
               att_source=(r.get("settings") or {}).get("att_source", "gyro (default)"), yaw_err_at_switch=ye0,
               extnav_to_switch=(-ext[0] if ext else None), extnav_all=ext,
               align_t=(al[-1] if al else None),
               align_step=(float(np.interp(al[-1] + 1, t, ye) - np.interp(al[-1] - 1, t, ye)) if al else None),
               fc_modes=r.get("fc_modes"), fc_texts_hold=r.get("fc_texts_hold"), reanchors_runner=r.get("reanchors"))
    T60 = r.get("hold_s_flown") or 60.0
    for T, k in ((30.0, "30"), (T60, "60")):
        w = (t >= 0) & (t <= T)
        edges = [0.0] + [x for x in st if 0 <= x <= T] + [T]
        g = max((b - a, a) for a, b in zip(edges, edges[1:]))
        wf = (tf >= 0) & (tf <= T)
        out.update({f"rms{k}": float(np.sqrt(np.mean(dist[w] ** 2))), f"max{k}": float(dist[w].max()), f"gap{k}": g[0],
                    f"gap_at{k}": g[1], f"fix_hz{k}": sum(1 for x in st if 0 <= x <= T) / T,
                    f"ferr{k}": (float(np.sqrt(np.mean(np.sum(ef_rel[wf] ** 2, 1)))) if ef_rel is not None and wf.any() else None),
                    f"ferr_max{k}": (float(np.linalg.norm(ef_rel[wf], axis=1).max()) if ef_rel is not None and wf.any() else None),
                    f"nbad{k}": (int(np.sum(np.linalg.norm(ef_rel[wf], axis=1) > 0.5)) if ef_rel is not None else -1),
                    f"nsent{k}": int(wf.sum()),
                    f"fabs_max{k}": (float(np.linalg.norm(ef[wf], axis=1).max()) if wf.any() else None),
                    f"nbad_abs{k}": int(np.sum(np.linalg.norm(ef[wf], axis=1) > 0.5)) if wf.any() else 0,
                    f"yaw_chg{k}": float(np.max(np.abs(ye[w] - ye0))),
                    f"reanchors{k}": sum(1 for x in rows if 0 <= float(x["t"]) - te <= T and "re-anchored" in (x["note"] or ""))})
    dst = os.path.join(OUT, "runs")
    os.makedirs(dst, exist_ok=True)
    for p in glob.glob(os.path.join(runs, label + "_onboard*.csv")):
        with open(p, "rb") as f, gzip.open(os.path.join(dst, os.path.basename(p) + ".gz"), "wb") as gz:
            shutil.copyfileobj(f, gz)
    for ext_ in (".npz", ".json", "_settings.json"):
        if os.path.exists(os.path.join(runs, label + ext_)):
            shutil.copy(os.path.join(runs, label + ext_), os.path.join(dst, label + ext_))
    json.dump(r, open(os.path.join(dst, label + "_result.json"), "w"), indent=1, default=str)
    return out


def gate_loads():
    """1-min load that the driver's gate saw when it started each run (integ/kfmap/closed/driver.log)."""
    g = {}
    try:
        for line in open(os.path.join(OUT, "driver.log")):
            e = json.loads(line)
            if e.get("ev") == "start":
                g[e["label"]] = e["load"][0]
    except (OSError, ValueError):
        pass
    return g


def main():
    allr = json.load(open(os.path.join(SP, COPY, "apsim", "low_ap_results.json")))
    gl = gate_loads()
    res = []
    for label in sorted(allr):
        try:
            res.append(dict(one(label, allr[label]), gate_load=gl.get(label)))
        except (OSError, KeyError, ValueError, TypeError) as e:
            print("skip", label, repr(e), file=sys.stderr)
    f = lambda x, n=6, p=2: "-".rjust(n) if x is None else f"{x:{n}.{p}f}"   # noqa: E731
    key = lambda x: (x["ground"], int(re.search(r"_(\d+)m_", x["label"]).group(1)), re.sub(r"_s\d+_\w+$", "", x["label"]),
                     int(re.search(r"_s(\d+)_", x["label"]).group(1)), ORDER.index(x["version"]) if x["version"] in ORDER else 9)
    lines = ["case (ArduCopter SITL)                 version                       rms30  max30  gap30 ferr30 fmax30 bad30/sent | "
             "abs fmax30 bad | rean30 | yaw err: at sw  chg30 | extnav->sw align(t, step) | rms60  max60  gap60 ferr60 fmax60 bad60 | gate load | runner load start/end"]
    prev = None
    for x in sorted(res, key=key):
        case = re.sub(r"_" + x["version"] + r"(_|$)", r"\1", x["label"])
        if prev and case.rsplit("_", 1)[0] != prev.rsplit("_", 1)[0]:
            lines.append("")
        prev = case
        al = "-" if x["align_t"] is None else f"{x['align_t']:.1f}s {x['align_step']:+.1f}"
        lines.append(f"{case:39s} {NAMES.get(x['version'], x['version']):28s} {f(x['rms30'])} {f(x['max30'])} {f(x['gap30'])}"
                     f" {f(x['ferr30'])} {f(x['ferr_max30'])} {x['nbad30']:5d}/{x['nsent30']:<4d} | {f(x['fabs_max30'], 9)} {x['nbad_abs30']:4d} |"
                     f" {x['reanchors30']:6d} | {f(x['yaw_err_at_switch'], 7, 1)} {f(x['yaw_chg30'], 6, 1)} |"
                     f" {f(x['extnav_to_switch'], 9, 1)} {al:>12s} | {f(x['rms60'])} {f(x['max60'])} {f(x['gap60'])} {f(x['ferr60'])}"
                     f" {f(x['ferr_max60'])} {x['nbad60']:5d} | {f(x['gate_load'], 5, 2)} | {f(x['load_start'], 5, 1)} / {f(x['load_end'], 4, 1)}")
    lines += ["", __doc__.split("Run:")[0].strip(), "", "Files: integ/kfmap/closed/runs/<label>.* (onboard log, truth, results)."]
    open(os.path.join(OUT, "ab_table.txt"), "w").write("\n".join(lines) + "\n")
    json.dump(res, open(os.path.join(OUT, "ab_table.json"), "w"), indent=1, default=str)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
