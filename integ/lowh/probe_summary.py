#!/usr/bin/env python3
"""Tables of the probe results (integ/lowh/results/probe/*.json): the still-hover gap (noise floor) per height, and the
gap against the drift share d / h per direction, with the geometric prediction (geom_gap: 1 m forward step as hold.py;
tangent: a central step).
Run: ~/.venvs/vio/bin/python integ/lowh/probe_summary.py [files...]   (default: every probe file) > results/probe_summary.txt
"""
import glob
import json
import os
import sys

import numpy as np

from jsonz import jload

HERE = os.path.dirname(os.path.abspath(__file__))


def q(x, p):
    x = [v for v in x if v is not None]
    return float(np.percentile(x, p)) if x else float("nan")


def main():
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "results", "probe", "*.json*")))
    for f in files:
        d = jload((f))
        rows = d["rows"]
        print(f"== {d['label']}  (hold {os.path.relpath(d['hold_py'], os.path.dirname(os.path.dirname(HERE)))}, "
              f"ground {d['ground']}, lens {d['lens']}, vib {d['vib_deg']}, tilt {d['tilt_deg']}, exact lens "
              f"{d['exact_lens']}, Hold.pos {d['pos']}; seeds {sorted({r['seed'] for r in rows})})")
        hs = sorted({r["h"] for r in rows})
        print("  still hover (d = 0): halves gap px median / p90 / max, refused, keyframe corners (median)")
        for h in hs:
            st = [r for r in rows if r["h"] == h and r["dir"] == "still"]
            g = [r["gap"] for r in st]
            print(f"    {h:5.0f} m  {q(g, 50):5.2f} / {q(g, 90):5.2f} / {q(g, 100):5.2f}   refused {sum(not r['ok'] for r in st)}/{len(st)}"
                  f"   corners {int(np.median([r['kf_corners'] for r in st]))}")
        print("  drift d = share x h: gap px median over seeds [geometric prediction 1 m step / tangent], refused n/3, "
              "fix error along the drift (m, median)")
        for h in hs:
            for sh in sorted({r["share"] for r in rows if r["share"] > 0}):
                parts = []
                for dn in ("fwd", "back", "right", "left"):
                    s = [r for r in rows if r["h"] == h and r["share"] == sh and r["dir"] == dn]
                    if not s:
                        continue
                    e = [r.get("err_fwd") if dn in ("fwd", "back") else r.get("err_right") for r in s]
                    e = [v * (1 if dn in ("fwd", "right") else -1) for v in e if v is not None]
                    parts.append(f"{dn:5s} {q([r['gap'] for r in s], 50):5.1f} [{q([r['geom_gap'] for r in s], 50):4.1f}/"
                                 f"{q([r['geom_gap_tangent'] for r in s], 50):4.1f}] ref {sum(not r['ok'] for r in s)}"
                                 f" e {np.median(e) if e else float('nan'):+.2f}")
                print(f"    {h:4.0f} m  d/h {sh:4.2f} ({sh * h:4.2f} m): " + " | ".join(parts))
        whys = {}
        for r in rows:
            if r["why"]:
                k = r["why"].split(" (")[0]
                whys[k] = whys.get(k, 0) + 1
        print("  refusals:", whys)
        print()


if __name__ == "__main__":
    main()
