#!/usr/bin/env python3
"""Print the relocalization log of ol_kfmap.py result files: every try (time, stored keyframe, step, n, yaw, scale,
map-position error against the truth, the fix error then) and the refusal counts.
Run: ~/.venvs/vio/bin/python integ/kfmap/show_relocs.py <result .json.gz> [...] [--all]"""
import collections
import gzip
import json
import sys

for p in [x for x in sys.argv[1:] if not x.startswith("--")]:
    d = json.load(gzip.open(p, "rt"))
    rl = d["relocs"]
    print(p.split("/")[-1], "tries", len(rl), "accepted", sum(r["accepted"] for r in rl))
    why = collections.Counter((r["why"] or "").split(" (")[0].split(" over")[0] if not r["ok"] else "pass" for r in rl)
    print("  ", dict(why.most_common()))
    for r in rl:
        if "--all" in sys.argv or r["ok"] or r["accepted"]:
            print(f"   t {r['t_s']:6.2f} kf_h {r['kf_h']} home {r.get('home')} acc {int(r['accepted'])} ok {int(r['ok'])} step {r['step']} "
                  f"n {r['n']} phi {r['phi_deg']} s {r['scale']} dense {r.get('dense')} map_err {r.get('map_err')} "
                  f"fix_err {r.get('fix_err_now')} ms {r.get('ms')} probe {r.get('probe', False)} | {r['why']}")
