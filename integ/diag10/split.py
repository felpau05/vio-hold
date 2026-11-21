#!/usr/bin/env python3
"""Error split of a replay result (integ/diag10/results/*.json.gz): the error of each sent fix is the sum of its
increments between consecutive sent fixes, classed by what happened in between: 'reanchor' (Pipe re-anchored),
'kf' (a keyframe change or new anchor), 'dense' / 'corner' / 'mixed' (the path of the two fixes). Prints per class:
count, sum vector (m), RMS of the class's cumulative sum around its mean (m), and the final error RMS.
Run: ~/.venvs/vio/bin/python integ/diag10/split.py <file.json.gz> ..."""
import gzip, json, os, sys
import numpy as np


def split(rows):
    sent = [r for r in rows if r["sent"]]
    if len(sent) < 3:
        return {}
    ev = [(r["t"], "reanchor" if r["reanchor"] else "kf") for r in rows if r["reanchor"] or r["new_kf"]]
    out, cum, traj = {}, {}, {}
    for a, b in zip(sent[:-1], sent[1:]):
        kinds = {k for t, k in ev if a["t"] < t <= b["t"]}
        k = "reanchor" if "reanchor" in kinds else "kf" if "kf" in kinds else \
            "dense" if a["path"] == b["path"] == "dense" else "corner" if a["path"] == b["path"] == "corner" else "mixed"
        d = np.array(b["err"]) - np.array(a["err"])
        out.setdefault(k, []).append(d)
        cum[k] = cum.get(k, np.zeros(2)) + d
        for kk in ("reanchor", "kf", "dense", "corner", "mixed"):
            traj.setdefault(kk, []).append(cum.get(kk, np.zeros(2)).copy())
    res = {}
    for k, v in out.items():
        tj = np.array(traj[k])
        res[k] = dict(n=len(v), sum=np.round(np.sum(v, 0), 3).tolist(),
                      level_rms=round(float(np.sqrt(np.mean(np.sum((tj - tj.mean(0)) ** 2, 1)))), 3),
                      end_abs=round(float(np.linalg.norm(tj[-1])), 3))
    e = np.array([r["err"] for r in sent])
    res["total"] = dict(err_rms=round(float(np.sqrt(np.mean(np.sum(e ** 2, 1)))), 3),
                        first=sent[0]["err"], last=sent[-1]["err"])
    return res


if __name__ == "__main__":
    for f in sys.argv[1:]:
        r = json.load(gzip.open(f))
        s = split(r["rows"])
        print(os.path.basename(f)[:-8])
        for k, v in s.items():
            print("   ", k, v)
