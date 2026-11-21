#!/usr/bin/env python3
"""Builds integ/justice_d/summary.txt from the run files in integ/justice_d/ (open/, closed/, compute_*.json,
calib_loop_d_*.txt). Each case: its numbers and the file that holds it; a case with several runs (seeds or repeats)
also gets the median and the worst. Reads apsim/results.json for the existing analog closed-loop cases (not changed).
Run: ~/.venvs/vio/bin/python integ/justice_d/summary.py
"""
import glob
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
rel = lambda p: os.path.relpath(p, PROJ)    # noqa: E731


def med_worst(xs):
    xs = [x for x in xs if x is not None]
    return (float(np.median(xs)), float(np.max(xs))) if xs else (float("nan"), float("nan"))


def open_loop(lines):
    runs = {}
    for p in sorted(glob.glob(os.path.join(HERE, "open", "*.json"))):
        r = json.load(open(p))
        tag = r["label"].split(f"_s{r['seed']}", 1)[1]
        for v, x in r["variants"].items():
            runs.setdefault((tag, r["camera"], r["height"], v), []).append((r["seed"], x, p))
    for tag in sorted({k[0] for k in runs}):
        lines += ["", f"OPEN LOOP{' ' + tag if tag else ''} (integ/justice_d/open_loop.py{' --diag' if tag else ''}): ground "
                  "ottawa2022 (validation), flight = recorded ArduCopter SITL 100 m w20g40 of the same seed, height shifted; "
                  "55 s per run; error of every fix Hold lets through (m)",
                  f"{'camera':8s} {'height':>6s} {'variant':20s} {'seed':>4s} {'Hz':>5s} {'% fix':>6s} {'RMS m':>7s} {'worst m':>8s}  file"]
        for (t, cam, h, v), rs in sorted(runs.items(), key=lambda kv: (kv[0][2], kv[0][1], kv[0][3])):
            if t != tag:
                continue
            for seed, x, p in sorted(rs):
                lines.append(f"{cam:8s} {h:6d} {v:20s} {seed:4d} {x['hz']:5.0f} {x['pct_frames_fix']:6.1f} "
                             f"{x.get('rms_m', float('nan')):7.3f} {x.get('worst_m', float('nan')):8.3f}  {rel(p)}")
            if len(rs) > 1:
                (mr, wr), (mw, ww) = med_worst([x.get("rms_m") for _, x, _ in rs]), med_worst([x.get("worst_m") for _, x, _ in rs])
                lines.append(f"{cam:8s} {h:6d} {v:20s} {'all':>4s} {'':5s} {'':6s} RMS median {mr:.3f} worst {wr:.3f}; "
                             f"worst-error median {mw:.3f} worst {ww:.3f}  ({len(rs)} seeds)")


def closed_loop(lines):
    lines += ["", "CLOSED LOOP: ArduCopter 4.6 SITL, variant apb (onboard program in its own process), board model fixed "
              "80 ms per Hold.process, ground ottawa2019fall (the ground of the existing analog runs), 60 s hold. Drift = "
              "true distance from the hold point (m).",
              "fix err = each sent fix against the truth, constant offset removed (fix_error.py -> fix_error.json); gap = longest "
              "time without a sent fix in [-10, +3] s of the hold start.",
              f"{'case':34s} {'RMS m':>6s} {'max m':>6s} {'fix Hz':>6s} {'age ms':>6s} {'frames':>6s} {'handover p50/p95 ms':>20s} "
              f"{'load start/end':>14s} {'fix err RMS/worst':>17s} {'gap s':>5s}  file / events"]
    rows = {}
    fe = json.load(open(os.path.join(HERE, "fix_error.json"))) if os.path.exists(os.path.join(HERE, "fix_error.json")) else {}
    for p in sorted(glob.glob(os.path.join(HERE, "closed", "*.json"))):
        r = json.load(open(p))
        ev = "; ".join(str(e[1]) for e in r.get("events", [])[1:])[:100]
        f = lambda k, fmt: (fmt % r[k]) if isinstance(r.get(k), (int, float)) else "-"   # noqa: E731
        lines.append(f"{r['case']:34s} {f('rms', '%6.3f'):>6s} {f('max', '%6.3f'):>6s} {f('fix_rate_hz', '%6.1f'):>6s} "
                     f"{f('age_ms_p50', '%6.0f'):>6s} {f('frames', '%6d'):>6s} "
                     f"{f('handover_ms_p50', '%.1f') + ' / ' + f('handover_ms_p95', '%.1f'):>20s} "
                     f"{str(r.get('load_1min_at_start')) + ' / ' + str(r.get('load_1min_at_end')):>14s} "
                     f"{(str(fe[r['case']]['rms_m']) + ' / ' + str(fe[r['case']]['worst_m'])) if r['case'] in fe else '-':>17s} "
                     f"{fe.get(r['case'], {}).get('gap_before_hold_s', '-'):>5}  {rel(p)} {ev}")
        rows[r["case"]] = r
    seeds = [rows[k] for k in rows if k.startswith("jd_apb_100m_w20g40_s")]
    if len(seeds) > 1:
        (mr, wr), (mm, wm) = med_worst([r.get("rms") for r in seeds]), med_worst([r.get("max") for r in seeds])
        lines.append(f"{'jd_apb_100m_w20g40 seeds 11-13':34s} RMS median {mr:.3f} worst {wr:.3f}; max median {mm:.3f} worst {wm:.3f}")
    cal = [rows[k] for k in rows if k.startswith("jdcal")]
    if len(cal) > 1:
        (mr, wr), (mm, wm) = med_worst([r.get("rms") for r in cal]), med_worst([r.get("max") for r in cal])
        lines.append(f"{'jdcal* (T4 calibrated, all runs)':34s} RMS median {mr:.3f} worst {wr:.3f}; max median {mm:.3f} worst {wm:.3f}"
                     "  (jdcal: frames at 30 Hz, Frames bug, since fixed)")
    for k in ("c_d140_100m_w20g40_s11", "c_d140_100m_w20g40_s12", "c_d140_100m_w20g40_s13"):
        if k in fe:
            lines.append(f"{k:34s} (existing analog run) fix err RMS/worst {fe[k]['rms_m']} / {fe[k]['worst_m']}, gap {fe[k]['gap_before_hold_s']} s")
    try:
        old = json.load(open(os.path.join(PROJ, "apsim", "results.json")))
        a = [old[f"c_d140_100m_w20g40_s{s}"] for s in (11, 12, 13)]
        (mr, wr), (mm, wm) = med_worst([r["rms"] for r in a]), med_worst([r["max"] for r in a])
        lines.append(f"{'existing analog c_d140 seeds 11-13':34s} RMS median {mr:.3f} worst {wr:.3f}; max median {mm:.3f} worst {wm:.3f}"
                     "  (apsim/results.txt, PAL, same settings)")
    except (OSError, KeyError, ValueError):
        pass


def compute(lines):
    for p in sorted(glob.glob(os.path.join(HERE, "compute_*.json"))):
        r = json.load(open(p))
        lines += ["", f"COMPUTE ({rel(p)}): {r['cpu']}, {r['threads']}, load at start {r['load_1min_at_start']}, "
                  f"{r['frames']} frames (first {r['skip_first']} not counted)"]
        for k, v in r["sizes"].items():
            lines.append(f"  Hold.process {k:9s} median {v['median_ms']:6.2f} ms  p95 {v['p95_ms']:6.2f} ms  "
                         f"ratio to 640x360 {r['ratio_median_to_640x360'][k]:.2f}")
        v = r["luma_reduce_1280x720_yuyv"]
        lines.append(f"  capture.luma + INTER_AREA, one 1280x720 YUYV buffer: median {v['median_ms']:.3f} ms  p95 {v['p95_ms']:.3f} ms")


def calib(lines):
    for p in sorted(glob.glob(os.path.join(HERE, "calib_loop_d_2*.txt"))):
        lines += ["", f"CALIBRATION IN THE LOOP ({rel(p)}):"] + ["  " + x for x in open(p).read().splitlines()
                                                                   if x.startswith(("  ", "PASS", "FAIL", "ALL", "recording", "calib took"))]
    for p in sorted(glob.glob(os.path.join(HERE, "calib_loop_d_rays_*.txt"))):
        r = json.load(open(p))["rays_within_55deg_of_nadir"]
        lines.append(f"  rays (lens + mount) against the truth within 55 deg of nadir ({rel(p)}): calibrated median "
                     f"{r['calibrated']['median_deg']} max {r['calibrated']['max_deg']} deg; start settings median "
                     f"{r['start_settings']['median_deg']} deg")


def main():
    lines = ["Justice D measurements (simulation only). Built by integ/justice_d/summary.py from the run files named below.",
             "Validation seeds 11-13; no hold-out site or seed used."]
    open_loop(lines)
    compute(lines)
    calib(lines)
    closed_loop(lines)
    p = os.path.join(HERE, "check_analog_same.txt")
    if os.path.exists(p):
        lines += ["", f"ANALOG RENDER UNCHANGED ({rel(p)}): " + open(p).read().splitlines()[-1]]
    p = os.path.join(HERE, "check_replay_same.txt")
    if os.path.exists(p):
        lines += [f"EXISTING ANALOG OPEN-LOOP CASE ({rel(p)}): " + open(p).read().splitlines()[-1]] + \
                 ["  " + x for x in open(p).read().splitlines() if "attitude:" in x]
    p = os.path.join(HERE, "run_all_final.txt")
    if os.path.exists(p):
        lines += ["", f"sh tests_apb/run_all.sh at the end ({rel(p)}):"] + ["  " + x for x in open(p).read().splitlines() if ": rc=" in x]
    open(os.path.join(HERE, "summary.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
