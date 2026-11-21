#!/usr/bin/env python3
"""Tables of the open-loop study (integ/kfmap/results/ol):
  main   per ground and height, per version: median over seeds 11-13 [worst seed] of error RMS, max error, fixes per s,
         longest gap, re-anchors, relocalizations; false acceptances (sum)
  false  the wrong-relocalization tests (place, bad, rep): tries, passes, acceptances, and the false ones
A relocalization counts as FALSE when the map position of the accepted try is more than FALSE_M from the truth (the
ground truth of the replay). In the 'place' set every acceptance is false (the camera sees another place). In the 'rep'
set (repeated texture, period P) the limit is min(FALSE_M, P / 2): a match one period off is false.
Run: ~/.venvs/vio/bin/python integ/kfmap/tables.py main|false   -> results/table_<set>.txt"""
import glob
import gzip
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OL = os.path.join(HERE, "results", "ol")
FALSE_M = 0.5        # m; assumed: above the error of the first chain's fixes in these replays (0.05-0.3 m)


def load(p):
    with gzip.open(p, "rt") as f:
        return json.load(f)


def false_of(d, lim=FALSE_M, every=False):
    acc = [r for r in d["relocs"] if r.get("accepted")]
    return [r for r in acc if every or r.get("map_err") is None or r["map_err"] > lim], acc


def main_table():
    out = ["Open loop, VALIDATION (ottawa2022, farmland; seeds 11-13), re-anchor at Hold's last fix (case B), 55 s, 12 Hz.",
           "Median over the 3 seeds [worst seed]. false: accepted relocalizations with a map position more than "
           f"{FALSE_M} m from the truth (sum over seeds).", ""]
    hdr = (f"{'ground / h':18s} {'version':7s} {'err RMS m':>14s} {'max err m':>14s} {'fixes/s':>14s} {'longest gap s':>14s}"
           f" {'re-anchors':>11s} {'relocs':>8s} {'false':>6s} {'map err of accepted (m)':s}")
    out.append(hdr)
    for g in ("ottawa2022", "farmland"):
        for h in (5, 10):
            for v in ("off", "dl", "both", "kfonly"):
                ds = [load(p) for p in sorted(glob.glob(os.path.join(OL, f"kf_{g}_crop_{h}m_s1[123]_main_relast_{v}.json.gz")))]
                if not ds:
                    continue
                S = [d["summary"] for d in ds]

                def col(k, worst=max, p=2):
                    x = [s[k] for s in S if s[k] is not None]
                    return f"{np.median(x):.{p}f} [{worst(x):.{p}f}]".rjust(14) if x else "-".rjust(14)
                fl = [false_of(d) for d in ds]
                me = [r["map_err"] for d in ds for r in d["relocs"] if r.get("accepted")]
                out.append(f"{g + ' ' + str(h) + ' m':18s} {v:7s} {col('err_rms_m')} {col('err_max_m')} {col('fix_hz', min)}"
                           f" {col('longest_gap_s')} {col('reanchors', max, 0)[3:]:>11s} {col('relocalizations', max, 0)[3:]:>8s}"
                           f" {sum(len(f) for f, _ in fl):6d} {[round(x, 2) for x in me]}  (n={len(ds)})")
            out.append("")
    out += ["Versions: off = both options off (the code before); dl = DENSE_LOCAL; both = DENSE_LOCAL + KF_MAP; kfonly = KF_MAP.",
            "Files: results/ol/kf_<ground>_crop_<h>m_s<seed>_main_relast_<version>.json.gz (ol_kfmap.py; jobs.py set main)."]
    return out


def false_table():
    out = ["Wrong relocalizations (open loop). Every case: a forced loss at 10 s (re-anchor at Hold's last fix, Pipe-like),",
           "then the named pictures. tries / passes: single tries (probe tries in brackets, see 'bad'); acc: accepted; false: see",
           f"the header of tables.py (FALSE_M {FALSE_M} m).", ""]
    rows = {}
    for p in sorted(glob.glob(os.path.join(OL, "kf_*_relast_*.json.gz"))):
        n = os.path.basename(p)
        m = re.match(r"kf_(.+?)_crop_(\d+)m_s(\d+)_(pl\d+|plg|bf\d+|bn\d+|r\d)_relast_(\w+)\.json\.gz$", n)
        if not m:
            continue
        ground, h, seed, case, ver = m.groups()
        d = load(p)
        if case.startswith("pl"):
            what = f"place {'other photo' if case == 'plg' else case[2:] + ' m away'}"
            f, acc = false_of(d, every=True)
        elif case.startswith("b"):
            what = f"bad {'forced+probe' if case[1] == 'f' else 'natural'}: {d['bad']}"
            f, acc = false_of(d)
        else:
            P = float(ground[3:])
            what = f"repeated P={P:g} m, offset {['0', 'P/2', 'P'][int(case[1])]}"
            f, acc = false_of(d, lim=min(FALSE_M, P / 2))
        tries = [r for r in d["relocs"] if not r.get("probe")]
        probe = [r for r in d["relocs"] if r.get("probe")]
        bw = d.get("bad_window")
        pb = [r for r in probe if bw and bw[0] <= r["t_s"] < bw[1]]
        k = (what, h, ver)
        a = rows.setdefault(k, dict(runs=0, tries=0, passes=0, acc=0, false=0, probe=0, probe_pass=0, map_err=[]))
        a["runs"] += 1
        a["tries"] += len(tries)
        a["passes"] += sum(1 for r in tries if r["ok"])
        a["acc"] += len(acc)
        a["false"] += len(f)
        a["probe"] += len(pb)
        a["probe_pass"] += sum(1 for r in pb if r["ok"])
        a["map_err"] += [r["map_err"] for r in acc]
    out.append(f"{'case':42s} {'h':>3s} {'version':7s} {'runs':>4s} {'tries':>6s} {'passes':>6s} {'acc':>4s} {'FALSE':>5s}"
               f" {'probe tries in bad window (passes)':>36s}  map err of accepted (m)")
    tot = dict(acc=0, false=0)
    for k in sorted(rows):
        a = rows[k]
        tot["acc"] += a["acc"]
        tot["false"] += a["false"]
        out.append(f"{k[0]:42s} {k[1]:>3s} {k[2]:7s} {a['runs']:4d} {a['tries']:6d} {a['passes']:6d} {a['acc']:4d} {a['false']:5d}"
                   f" {(str(a['probe']) + ' (' + str(a['probe_pass']) + ')') if a['probe'] else '-':>36s}  "
                   f"{sorted(round(x, 2) for x in a['map_err'])[-6:]}")
    out += ["", f"TOTAL accepted {tot['acc']}, FALSE {tot['false']}",
            "Files: results/ol/kf_*_{pl*,plg,bf*,bn*,r*}_relast_*.json.gz (jobs.py sets place, bad, rep)."]
    return out


def bad_table():
    """Bad pictures without a forced loss (set bad, cases bn*): per kind, height and version, summed over seeds 11-13: Hold
    fixes in the 10 s bad window, sent fixes, the largest error of a Hold fix there, relocalizations in the window."""
    out = ["Bad pictures 12-22 s (ottawa2022, VALIDATION; seeds 11-13 summed), no forced loss, re-anchor at Hold's last fix.",
           "Hold fixes: frames with a fix in the window (120 frames per seed); sent: after the Pipe's 10 good fixes; max err:",
           "largest error of those Hold fixes (m); relocs: accepted relocalizations inside the window.", ""]
    out.append(f"{'kind':24s} {'h':>3s} {'version':7s} {'Hold fixes':>10s} {'sent':>6s} {'max err m':>9s} {'relocs':>6s}")
    import collections
    agg = collections.OrderedDict()
    for p in sorted(glob.glob(os.path.join(OL, "kf_ottawa2022_crop_*m_s1[123]_bn*_relast_*.json.gz"))):
        m = re.match(r"kf_ottawa2022_crop_(\d+)m_s(\d+)_bn(\d+)_relast_(\w+)\.json\.gz$", os.path.basename(p))
        h, seed, k, ver = m.groups()
        d = load(p)
        b = d["bad_summary"]
        a = agg.setdefault((d["bad"], int(h), ver), dict(fx=0, sent=0, mx=0.0, rel=0, n=0))
        a["fx"] += b["hold_fixes"]
        a["sent"] += b["sent"]
        a["mx"] = max(a["mx"], b["max_err_m"])
        a["rel"] += sum(1 for r in d["relocs"] if r.get("accepted") and 12 <= r["t_s"] < 22)
        a["n"] += 1
    order = {"off": 0, "dl": 1, "both": 2, "kfonly": 3}
    for (kind, h, ver), a in sorted(agg.items(), key=lambda kv: (kv[0][0], kv[0][1], order.get(kv[0][2], 9))):
        out.append(f"{kind:24s} {h:3d} {ver:7s} {a['fx']:10d} {a['sent']:6d} {a['mx']:9.2f} {a['rel']:6d}   (seeds {a['n']})")
    out += ["", "Files: results/ol/kf_ottawa2022_crop_<h>m_s<seed>_bn<k>_relast_<version>.json.gz (jobs.py set bad)."]
    return out


if __name__ == "__main__":
    which = sys.argv[1]
    lines = main_table() if which == "main" else bad_table() if which == "bad" else false_table()
    open(os.path.join(HERE, "results", f"table_{which}.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
