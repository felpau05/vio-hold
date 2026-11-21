#!/usr/bin/env python3
"""Tables of the ol_graph.py runs: graph off against graph on, per ground / height / re-anchor mode; and the check that
'off' gives the snapshot's fixes. Run: ~/.venvs/vio/bin/python integ/kfgraph/summarize.py [split] -> results/table_<split>.txt
Each cell: median over the seeds [worst seed]. Worst = largest error / longest gap / most false fixes / fewest fixes.
"""
import glob
import gzip
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results", "ol")


def load(p):
    with gzip.open(p, "rt") as f:
        return json.load(f)


def main():
    split = sys.argv[1] if len(sys.argv) > 1 else "validation"
    pre = f"kg_{split[:3]}_"
    runs = {}
    for p in sorted(glob.glob(os.path.join(OUT, pre + "*.json.gz"))):
        d = load(p)
        if d.get("bad") or "_smoke" in d["label"] or "_check" in d["label"]:
            continue
        key = (d["ground"], d["height"], d["reanchor_at"])
        runs.setdefault(key, {}).setdefault(d["seed"], {})[d["version"]] = d
    lines = [f"ol_graph.py runs, split {split.upper()} (files: integ/kfgraph/results/ol/{pre}*.json.gz). 55 s hover replay, "
             "12 Hz, crop lens. median over seeds [worst seed]. reanchor: 'last' = Hold's last fix (as apb_main.Pipe without "
             "an FC position), 'truth' = the true position (a perfect FC).",
             "ground       h   reanchor seeds | err RMS m off -> on       | err max m off -> on       | fixes/s off -> on        "
             "| longest gap s off -> on  | false fixes off -> on | ms/frame off -> on | graph solve ms med/p95/max | used/frames"]
    ident = []
    for key in sorted(runs, key=lambda k: (k[2] != "last", k[0], k[1])):
        seeds = runs[key]
        S = {v: [seeds[s][v]["summary"] for s in sorted(seeds) if v in seeds[s]] for v in ("off", "on")}
        if not S["off"] or not S["on"] or len(S["off"]) != len(S["on"]):
            continue

        def cell(f, worst=max, fmt="{:.3f}"):
            a = [s[f] if s[f] is not None else np.nan for s in S["off"]]
            b = [s[f] if s[f] is not None else np.nan for s in S["on"]]
            return (f"{fmt.format(np.median(a))} [{fmt.format(worst(a))}] -> {fmt.format(np.median(b))} [{fmt.format(worst(b))}]")
        gs = [s["graph"] for s in S["on"] if s.get("graph")]
        gtxt = (f"{np.median([g['solve_ms_median'] for g in gs]):.2f}/{np.median([g['solve_ms_p95'] for g in gs]):.2f}/"
                f"{max(g['solve_ms_max'] for g in gs):.2f}" if gs else "-")
        used = f"{sum(g['used'] for g in gs)}/{sum(g['frames'] for g in gs)}" if gs else "-"
        lines.append(f"{key[0]:11s} {key[1]:3g}  {key[2]:5s}   {len(S['off'])}    | {cell('err_rms_m')} | {cell('err_max_m')} | "
                     f"{cell('fix_hz', min, '{:.2f}')} | {cell('longest_gap_s', max, '{:.2f}')} | "
                     f"{cell('false_fix', max, '{:.0f}')} | {cell('ms_mean', max, '{:.1f}')} | {gtxt} | {used}")
        for s in sorted(seeds):
            if "snap" in seeds[s] and "off" in seeds[s]:
                a, b = seeds[s]["snap"]["rows"], seeds[s]["off"]["rows"]
                same = len(a) == len(b) and all((x["sent"], x["err_all"], x["state"], x["why"], x["n"], x["d"]) ==
                                                (y["sent"], y["err_all"], y["state"], y["why"], y["n"], y["d"]) for x, y in zip(a, b))
                ident.append(f"  {key[0]} {key[1]:g} m {key[2]} seed {s}: off == snapshot on every frame "
                             f"(sent, error vector, state, reason, n): {same} ({len(a)} frames)")
    lines.append("Graph off against the snapshot hold.py (same frames):")
    lines += ident or ["  (no run with the snapshot)"]
    out = "\n".join(lines)
    print(out)
    open(os.path.join(HERE, "results", f"table_{split}.txt"), "w").write(out + "\n")


if __name__ == "__main__":
    main()
