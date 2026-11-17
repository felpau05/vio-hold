#!/usr/bin/env python3
"""Quality numbers of the whole-picture path (Hold._dense): peak, halves gap (px), match, on good ground
(open-loop replay, integ/avail/diag.py) and on bad pictures (tests/bad_pictures.py cases), with the
thresholds opened so every attempt is measured.

Run: ~/.venvs/vio/bin/python integ/lowtex/dstats.py pos RUN.npz GROUND T_FROM T_TO [key=json ...]
     ~/.venvs/vio/bin/python integ/lowtex/dstats.py neg CASE [CASE ...]
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "tests"), os.path.join(PROJ, "integ", "avail")]
import hold  # noqa: E402

OPEN = dict(DENSE_RESP=-1.0, DENSE_NCC=-2.0, DENSE_AGREE_PX=1e9, DENSE_PTS_PX=1e9)
rec, T, WIN = [], [0.0], (-1e9, 1e9)
_orig, _proc = hold.Hold._dense, hold.Hold.process


def _p(self, img, t0, *a):
    T[0] = t0
    return _proc(self, img, t0, *a)


def _wrap(self, *a):
    if not WIN[0] <= T[0] < WIN[1]:
        return _orig(self, *a)
    self.dq = [None, None, None]
    r = _orig(self, *a)
    rec.append(list(self.dq) + [isinstance(r, str) and r or "ok"])
    return r


def summ(tag):
    q = np.array([[np.nan if x is None else x for x in r[:3]] for r in rec], float)
    out = dict(tag=tag, attempts=len(rec))
    for i, k in enumerate(("peak", "gap_px", "match")):
        c = q[:, i][np.isfinite(q[:, i])] if len(q) else np.zeros(0)
        out[k] = [round(float(np.percentile(c, p)), 3) for p in (1, 5, 50, 95, 99)] if len(c) else None
    whys = {}
    for r in rec:
        k = r[3].split(" (")[0]
        whys[k] = whys.get(k, 0) + 1
    out["whys"] = whys
    print(json.dumps(out), flush=True)


if __name__ == "__main__":
    hold.Hold._dense, hold.Hold.process = _wrap, _p
    if sys.argv[1] == "pos":
        import diag
        a = sys.argv[2:]
        kv = {k: json.loads(v) for k, v in (x.split("=", 1) for x in a[4:])}
        st = kv.pop("settings", {})
        st["hold_cfg"] = dict(st.get("hold_cfg", {}), **{k.lower(): v for k, v in OPEN.items()})
        diag.main(a[0], a[1], float(a[2]), float(a[3]), settings=st, **kv)
        summ(f"pos {a[1]}")
    else:
        for k, v in OPEN.items():
            setattr(hold.Hold, k, v)
        import bad_pictures as B
        WIN = (B.BAD[0] + 0.5, B.BAD[1])          # the bad period only (as bad_pictures counts it)
        for case in sys.argv[2:]:
            rec.clear()
            for seed in range(3):
                B.job(case, seed)
            summ(f"neg {case}")
