#!/usr/bin/env python3
"""Open-loop replay for the keyframe graph: integ/lowh/ol_low.py's replay (its pictures, feeds, Pipe emulation and bad
pictures), with several Hold versions on the very same frames, and the graph's own numbers per frame.

Differences from ol_low.py (which is not changed):
- versions: --ver name=path[@json Hold settings], e.g. --ver off=integ/kfgraph/hold_graph.py
  --ver on=integ/kfgraph/hold_graph.py@'{"kf_graph": true}'. CM.HOLD_CFG is applied first, as in ol_low.py.
- --split validation (default): ol_low.py's data: flight apsim/runs/c_d140_100m_w20g40_s<seed>.npz, seeds 11-13,
  grounds ottawa2022 / farmland (VALIDATION) or lowh_fine. --split tuning: flight apsim/runs/ap_apb_100m_w20g40_s<seed>.npz
  (seeds 1-3, the same 100 m wind case, TUNING seeds), grounds ottawa2019fall / suburb / prairie (TUNING sites).
- per frame also: error vector, Mahalanobis distance under the sent covariance (false fix: m2 > 9.21, chi-square
  2 dof 99 %, as tests/harness.py), the time of Hold.process, and the graph's info (Hold.g_last).
- the root hold.py is not imported (another agent edits it today): the module name 'hold' is the snapshot copy
  integ/kfgraph/hold_snapshot.py (open_loop.py uses it for its Series only).
- OpenCV and BLAS on 1 thread.
Writes integ/kfgraph/results/ol/<label>_<version>.json.gz (refuses to write over a file).
"""
import os
import sys

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import cv2   # noqa: E402

cv2.setNumThreads(1)
import argparse   # noqa: E402
import gzip   # noqa: E402
import importlib.util   # noqa: E402
import json   # noqa: E402
import math   # noqa: E402
import time   # noqa: E402

import numpy as np   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.dont_write_bytecode = True
_spec = importlib.util.spec_from_file_location("hold", os.path.join(HERE, "hold_snapshot.py"))
_hold = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_hold)
sys.modules["hold"] = _hold
sys.path.insert(0, os.path.join(PROJ, "integ", "lowh"))
import ol_low as OLW   # noqa: E402  (BAD, bad_picture, bad_summary; also loads common and open_loop)

CM, OL, C = OLW.CM, OLW.OL, OLW.C
CHI2_99 = 9.21
SPLITS = {"validation": dict(seeds=(11, 12, 13), grounds=("ottawa2022", "farmland", "lowh_fine"),
                             flight=os.path.join(PROJ, "apsim", "runs", "c_d140_100m_w20g40_s{seed}.npz")),
          "tuning": dict(seeds=(1, 2, 3), grounds=("ottawa2019fall", "suburb", "prairie"),
                         flight=os.path.join(PROJ, "apsim", "runs", "ap_apb_100m_w20g40_s{seed}.npz"))}


def summary(rows, seconds):
    sent = [r for r in rows if r["sent"]]
    ts = np.array([r["t"] for r in sent])
    x = np.r_[0.0, ts, seconds]
    errs = np.array([r["err"] for r in sent])
    m2 = np.array([r["m2"] for r in sent])
    why = {}
    for r in rows:
        if not r["sent"]:
            k = r["why"].split(" (")[0]
            why[k] = why.get(k, 0) + 1
    ms = np.array([r["ms"] for r in rows])
    gl = [r["graph"] for r in rows if r.get("graph")]
    out = dict(frames=len(rows), sent=len(sent), fix_hz=round(len(sent) / seconds, 2),
               longest_gap_s=round(float(np.diff(x).max()), 2),
               err_rms_m=round(float(np.sqrt(np.mean(errs ** 2))), 3) if len(errs) else None,
               err_max_m=round(float(errs.max()), 3) if len(errs) else None,
               false_fix=int(np.sum(m2 > CHI2_99)), false_fix_share=round(float(np.mean(m2 > CHI2_99)), 3) if len(m2) else None,
               conf_wrong=int(np.sum((errs > 2.0) & (m2 > CHI2_99))) if len(errs) else 0,
               err_over_1m=int(np.sum(errs > 1.0)) if len(errs) else 0,
               reanchors=sum(1 for r in rows if r.get("reanchor")),
               new_anchors=sum(1 for r in rows if r.get("new_kf") == "anchor"),
               kf_changes=sum(1 for r in rows if r.get("new_kf") == "change"),
               sent_whole_picture=sum(1 for r in sent if r["path"] == "dense"),
               ms_mean=round(float(ms.mean()), 2), ms_p95=round(float(np.percentile(ms, 95)), 2), ms_max=round(float(ms.max()), 2),
               why=dict(sorted(why.items(), key=lambda kv: -kv[1])))
    if gl:
        sm = np.array([g["ms"] for g in gl])
        out["graph"] = dict(frames=len(gl), used=sum(g["used"] for g in gl), meas_mean=round(float(np.mean([g["n_meas"] for g in gl])), 2),
                            kept_mean=round(float(np.mean([g["n_kept"] for g in gl])), 2),
                            solve_ms_median=round(float(np.median(sm)), 3), solve_ms_p95=round(float(np.percentile(sm, 95)), 3),
                            solve_ms_max=round(float(sm.max()), 3),
                            corr_median_m=round(float(np.median([g["corr"] for g in gl])), 3),
                            corr_max_m=round(float(np.max([g["corr"] for g in gl])), 3),
                            gated_edges_max=int(max(g["gated_edges"] for g in gl)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ver", action="append", required=True, help="name=path[@json settings] (repeat)")
    ap.add_argument("--height", type=float, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--split", default="validation", choices=list(SPLITS))
    ap.add_argument("--ground", default=None)
    ap.add_argument("--lens", default="crop", choices=list(CM.LENSES))
    ap.add_argument("--seconds", type=float, default=55.0)
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--reanchor", default="truth", choices=["truth", "last"])
    ap.add_argument("--tag", default="")
    ap.add_argument("--bad", default=None, choices=list(OLW.BAD))
    ap.add_argument("--bad_from", type=float, default=10.0)
    ap.add_argument("--bad_to", type=float, default=20.0)
    a = ap.parse_args()
    sp = SPLITS[a.split]
    ground = a.ground or sp["grounds"][0]
    if a.seed not in sp["seeds"] or ground not in sp["grounds"]:
        raise SystemExit(f"{a.split}: seeds {sp['seeds']}, grounds {sp['grounds']} (DATA_SPLITS.md)")
    OL.FLIGHT = sp["flight"]
    if a.split == "tuning":
        CM.ground = lambda name: C.photo_ground(name)
    label = (f"kg_{a.split[:3]}_{ground}_{a.lens}_{a.height:g}m_s{a.seed}{a.tag}" + (f"_bad_{a.bad}" if a.bad else "")
             + ("_relast" if a.reanchor == "last" else ""))
    outd = os.path.join(HERE, "results", "ol")
    versions = []
    for v in a.ver:
        name, rest = v.split("=", 1)
        path, cfg = (rest.split("@", 1) + ["{}"])[:2]
        versions.append((name, os.path.abspath(path), json.loads(cfg)))
    for name, _, _ in versions:
        if os.path.exists(os.path.join(outd, f"{label}_{name}.json.gz")):
            raise SystemExit(f"{label}_{name}.json.gz exists")
    cam, c = CM.camera(ground, a.lens, a.seed, 0.2)
    pose, att, hgt, dy, te, src = OL.flight(a.seed, a.height)
    Rf = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])

    def truth(t):
        N, E, _, q = pose(t)
        return Rf @ np.array([N[0], E[0]])

    V = {}
    for name, path, cfg in versions:
        hm = CM.load_hold(path, "hold_" + name)
        mk, dn, t1 = hm.Hold._make_kf, hm.Hold._dense, hm.Hold._track1

        def make_kf(self, img, t0, att_, hgt_, tref, pos, acc, chain, pending, _mk=mk):
            k = _mk(self, img, t0, att_, hgt_, tref, pos, acc, chain, pending)
            if k is not None:
                self._newkf = "anchor" if pending else "change"
            return k

        def dense(self, kf, img, idx, guess, J, p_lk, _dn=dn):
            r = _dn(self, kf, img, idx, guess, J, p_lk)
            self._calls.append(dict(cur=kf is self.kf, ok=not isinstance(r, str)))
            return r

        hm.Hold._make_kf, hm.Hold._dense = make_kf, dense
        K, D = CM.est_lens(hm, cam, c)
        h = hm.Hold(K, D, hm.make_R_bc(), c["readout_s"], **dict(CM.HOLD_CFG, **cfg))
        h.reanchor(truth(te))
        V[name] = dict(h=h, rows=[], good=0, cfg=cfg, path=path)
    n = int(a.seconds * cam.fps)
    w0 = time.time()
    store, brng = {}, np.random.default_rng(300 + a.seed)
    for k in range(0, n, a.every):
        t0 = te + k / cam.fps + cam.exposure / 2
        img = cam.finish(cam.render(t0, pose))
        if a.bad and a.bad_from <= t0 - te < a.bad_to:
            img = OLW.bad_picture(a.bad, img, store, brng)
        for name, v in V.items():
            h = v["h"]
            h._calls, h._newkf = [], None
            w = time.perf_counter()
            e = h.process(img, t0, att, hgt)
            ms = 1e3 * (time.perf_counter() - w)
            re = False
            if e.state == "lost":
                h.reanchor(truth(e.t) if a.reanchor == "truth" else h.pos.copy())
                v["good"], re = 0, True
            if e.var is None:
                if e.state != "degraded":
                    v["good"] = 0
                sent, why = False, e.why or e.state
            else:
                v["good"] += 1
                sent = v["good"] >= 10
                why = "" if sent else f"good fix {v['good']} of 10"
            err = m2 = d = None
            if e.var is not None:
                d = e.pos - truth(e.t)
                err = float(np.linalg.norm(d))
                cov = e.cov if e.cov is not None else np.eye(2) * e.var / 2
                m2 = float(d @ np.linalg.solve(cov, d))
            cd = [g for g in h._calls if g["cur"]]
            path = ("dense" if cd and cd[-1]["ok"] else "corner") if e.var is not None else ""
            gl = getattr(h, "g_last", None)
            gl = dict(gl) if gl is not None and abs(gl["t"] - (t0 + c["readout_s"] / 2)) < 1e-9 and e.var is not None else None
            if gl:
                gl.pop("t")
                gl = {kk: (round(vv, 4) if isinstance(vv, float) else vv) for kk, vv in gl.items()}
            v["rows"].append(dict(t=round(t0 - te, 4), sent=sent, err=round(err, 4) if sent else None,
                                  err_all=None if err is None else round(err, 4), d=None if d is None else [round(float(x), 4) for x in d],
                                  m2=None if m2 is None else round(m2, 3), state=e.state, why=why, n=e.n, path=path,
                                  new_kf=h._newkf, reanchor=re, ms=round(ms, 2), graph=gl))
    for name, v in V.items():
        res = dict(label=f"{label}_{name}", version=name, hold_py=os.path.relpath(v["path"], PROJ), cfg=v["cfg"], split=a.split,
                   ground=ground, lens=a.lens, height=a.height, seed=a.seed, flight=src, every=a.every, hz=cam.fps / a.every,
                   seconds=a.seconds, reanchor_at=a.reanchor, hold_cfg=CM.HOLD_CFG, wall_s=round(time.time() - w0, 1),
                   load1_end=CM.load1(), g_stats=getattr(v["h"], "g_stats", None),
                   graph_stats=getattr(getattr(v["h"], "graph", None), "stats", None),
                   summary=summary(v["rows"], a.seconds), rows=v["rows"])
        if a.bad:
            for r in v["rows"]:
                r.setdefault("dense", [])
            res.update(bad=a.bad, bad_text=OLW.BAD[a.bad], bad_window=OLW.bad_summary(v["rows"], a.bad_from, a.bad_to))
        os.makedirs(outd, exist_ok=True)
        with gzip.open(os.path.join(outd, res["label"] + ".json.gz"), "wt") as f:
            json.dump(res, f)
        s = res["summary"]
        print(f"{res['label']}: fix {s['fix_hz']}/s, gap {s['longest_gap_s']} s, err RMS {s['err_rms_m']} max {s['err_max_m']} m, "
              f"false {s['false_fix']}, reanchors {s['reanchors']}, ms {s['ms_mean']} (p95 {s['ms_p95']}), graph {s.get('graph')}"
              + (f"\n    BAD {a.bad}: {res['bad_window']}" if a.bad else ""), flush=True)


if __name__ == "__main__":
    main()
