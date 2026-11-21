#!/usr/bin/env python3
"""One ol_graph.py run, versions side by side over time: error of each sent fix (every 6th frame), re-anchors (R),
new keyframes (K change, A anchor) and the graph's correction. Run: python integ/kfgraph/inspect_run.py <label stem>
e.g. kg_val_ottawa2022_crop_5m_s11_relast (reads results/ol/<stem>_<version>.json.gz for every version found)."""
import glob
import gzip
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    stem = sys.argv[1]
    step = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    V = {}
    for p in sorted(glob.glob(os.path.join(HERE, "results", "ol", stem + "_*.json.gz"))):
        with gzip.open(p, "rt") as f:
            d = json.load(f)
        V[d["version"]] = d
    for name, d in V.items():
        rows = d["rows"]
        s = d["summary"]
        print(f"{name}: {s['err_rms_m']} m RMS, max {s['err_max_m']}, fix {s['fix_hz']}/s, gap {s['longest_gap_s']} s, "
              f"false {s['false_fix']}, reanchors {s['reanchors']}, ms {s['ms_mean']}; g_stats {d.get('g_stats')}")
        out = []
        for i in range(0, len(rows), step):
            w = rows[i:i + step]
            e = [r["err_all"] for r in w if r["err_all"] is not None]
            tag = ("R" if any(r["reanchor"] for r in w) else "") + ("A" if any(r["new_kf"] == "anchor" for r in w) else "") \
                + ("K" if any(r["new_kf"] == "change" for r in w) else "")
            g = [r["graph"] for r in w if r.get("graph")]
            gc = max((x["corr"] for x in g), default=None)
            out.append(f"{(sum(e) / len(e)) if e else float('nan'):.2f}{tag}" + (f"({gc:.2f},{max(x['n_kept'] for x in g)})" if g else ""))
        print(" ".join(out))


if __name__ == "__main__":
    main()
