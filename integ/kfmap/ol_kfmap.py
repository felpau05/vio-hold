#!/usr/bin/env python3
"""Open-loop replays for DENSE_LOCAL and KF_MAP (integ/kfmap): integ/lowh/ol_low.py with a hold_cfg per version, the
relocalization log, forced losses, bad pictures, another place, and repeated texture.

Pictures, motion, FC feeds, camera and Pipe emulation as integ/lowh/ol_low.py (its parts are imported): px4sim/camera.py
renders each frame from the true pose of apsim/runs/c_d140_100m_w20g40_s<seed>.npz with the height moved to --height,
Hold gets the FC's ATTITUDE and height, 12 Hz, send after 10 good fixes, re-anchor after 'lost' at Hold's own last fix
(--reanchor last, case B of integ/lowh/README.txt) or at the true position (truth).

Versions: --hold name=path[:{json hold_cfg}] (the json is merged onto integ/lowh/common.HOLD_CFG).
Extra cases (for the false-acceptance tests; none is a hold result):
  --force T[:dN:dE],...   at T s, re-anchor at Hold's last fix + (dN, dE) m, as Pipe does after a loss (a known step)
  --place_from S --place_shift N:E   from S s, the camera sees the ground N, E m away (another place, same flight)
  --place_ground G        from --place_from, another ground photo (farmland, ottawa2022, lowh_fine)
  --bad KIND --bad_from --bad_to   ol_low.BAD kinds and the kinds of tests/bad_pictures.py (BAD2)
  --ground rep<P>         repeated texture: a random tile of P m repeated over the ground (made here, synthetic)
  --probe                 every frame after a forced loss also calls Hold._reloc (gate only, never accepts): the share
                          of single tries that pass on these pictures
Writes integ/kfmap/results/ol/<label>_<version>.json.gz (refuses to write over a file).
"""
import argparse
import gzip
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(PROJ, "integ", "lowh"))
sys.dont_write_bytecode = True
import ol_low as OLL   # noqa: E402
import common as CM    # noqa: E402

C, OL = CM.C, OLL.OL
OUT = os.path.join(HERE, "results", "ol")

TUNING = ("ottawa2019fall", "suburb", "prairie")     # DATA_SPLITS.md TUNING sites (for the choice of the assumed limits)
BAD2 = ["static std 5", "static std 30", "static std 60", "frozen picture", "frozen + fresh noise", "NO SIGNAL screen",
        "breakup bursts"]


def rep_ground(period_m, seed=7):
    """Repeated texture: one random tile of period_m (multi-scale noise, grey 120, std 25) repeated, 6 mm per pixel."""
    res = 0.006
    n = max(8, int(round(period_m / res)))
    rng = np.random.default_rng(seed)
    tile = C._octaves(rng, 256, 1, 8, 0.1)
    import cv2
    tile = cv2.resize(tile.astype(np.float32), (n, n), interpolation=cv2.INTER_CUBIC)
    reps = int(math.ceil(8192 / n))
    img = np.tile(tile, (reps, reps))[:8192, :8192]
    img = np.clip(120.0 + 25.0 * img / max(float(img.std()), 1e-6), 0, 255).astype(np.uint8)
    return C.Ground(img, res)


def bad2(kind, img, store, rng, t_in):
    """The bad kinds of tests/bad_pictures.py at this picture size. t_in: s since the bad window started."""
    H, W = img.shape
    if kind.startswith("static"):
        return np.clip(90 + rng.normal(0, float(kind.split()[-1]), img.shape), 0, 255).astype(np.uint8)
    if kind == "frozen picture":
        return store.setdefault("img", img)
    if kind == "frozen + fresh noise":
        return np.clip(store.setdefault("img", img) + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
    if kind == "NO SIGNAL screen":
        if "ns" not in store:
            import cv2
            ns = np.full((H, W), 60, np.uint8)
            cv2.putText(ns, "NO SIGNAL", (int(W * 0.3), int(H * 0.5)), cv2.FONT_HERSHEY_SIMPLEX, 1.8, 235, 3, cv2.LINE_AA)
            cv2.putText(ns, "AV1  NTSC", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 235, 2, cv2.LINE_AA)
            store["ns"] = ns
        return np.clip(store["ns"] + rng.normal(0, 2, img.shape), 0, 255).astype(np.uint8)
    if kind == "breakup bursts" and t_in % 0.5 < 0.11:      # tests/bad_pictures.py breakup(), a band of 40 rows
        out = img.astype(np.int16)
        y0 = int(rng.integers(0, img.shape[0] - 40))
        band = np.roll(out[y0:y0 + 40], int(rng.integers(5, 30)), axis=1) + int(rng.integers(20, 60))
        salt = rng.random(band.shape) < 0.2
        band[salt] = rng.integers(0, 256, salt.sum())
        out[y0:y0 + 40] = band
        return np.clip(out, 0, 255).astype(np.uint8)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", action="append", required=True)
    ap.add_argument("--height", type=float, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--ground", default="ottawa2022")
    ap.add_argument("--lens", default="crop", choices=list(CM.LENSES))
    ap.add_argument("--seconds", type=float, default=55.0)
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--reanchor", default="last", choices=["truth", "last"])
    ap.add_argument("--force", default="")
    ap.add_argument("--place_from", type=float, default=None)
    ap.add_argument("--place_shift", default="0:0")
    ap.add_argument("--place_ground", default=None)
    ap.add_argument("--bad", default=None, choices=list(OLL.BAD) + BAD2)
    ap.add_argument("--bad_from", type=float, default=12.0)
    ap.add_argument("--bad_to", type=float, default=22.0)
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--tag", default="")
    ap.add_argument("--threads", type=int, default=1, help="cv2.setNumThreads (1: one core per replay)")
    a = ap.parse_args()
    if a.seed not in (11, 12, 13):
        raise SystemExit("validation seeds 11, 12, 13 only (DATA_SPLITS.md; the flights of seeds 14-19 do not exist)")
    if not (a.ground in ("ottawa2022", "farmland", "lowh_fine") + TUNING or a.ground.startswith("rep")):
        raise SystemExit("grounds: ottawa2022, farmland (VALIDATION), ottawa2019fall, suburb, prairie (TUNING), lowh_fine, "
                         "rep<P> (synthetic)")
    label = (f"kf_{a.ground}_{a.lens}_{a.height:g}m_s{a.seed}{a.tag}" + ("_relast" if a.reanchor == "last" else "_retruth"))
    versions = []
    for x in a.hold:
        name, rest = x.split("=", 1)
        path, cfg = (rest.split(":", 1) + ["{}"])[:2] if ":{" in rest else (rest, "{}")
        versions.append((name, path, json.loads(cfg)))
    for name, _, _ in versions:
        if os.path.exists(os.path.join(OUT, f"{label}_{name}.json.gz")):
            raise SystemExit(f"{label}_{name}.json.gz exists")
    import cv2
    cv2.setNumThreads(a.threads)
    gr = (rep_ground(float(a.ground[3:])) if a.ground.startswith("rep") else
          C.photo_ground(a.ground) if a.ground in TUNING else None)
    cam, c = CM.camera("ottawa2022" if gr is not None else a.ground, a.lens, a.seed, 0.2, gr=gr)
    cam2 = None
    if a.place_ground:
        cam2, _ = CM.camera(a.place_ground, a.lens, a.seed, 0.2)
    pose0, att, hgt, dy, te, src = OL.flight(a.seed, a.height)
    shift = np.array([float(v) for v in a.place_shift.split(":")])

    def pose_seen(t):              # the pose the camera renders: another place after place_from
        N, E, D, q = pose0(t)
        if a.place_from is not None and np.any(np.atleast_1d(t) - te >= a.place_from):
            m = (np.atleast_1d(t) - te) >= a.place_from
            N, E = N + shift[0] * m, E + shift[1] * m
        return N, E, D, q
    Rf = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])

    def truth(t):
        N, E, _, q = pose0(t)
        return Rf @ np.array([N[0], E[0]]), float(C.q_yaw(q)[0]) + dy

    forces = []
    for f in [x for x in a.force.split(",") if x]:
        p = [float(v) for v in f.split(":")] + [0.0, 0.0]
        forces.append([p[0], p[1], p[2], False])

    V = {}
    for name, path, cfg in versions:
        hm = CM.load_hold(os.path.abspath(path), "hold_" + name)
        CM.capture_ds(hm)
        dn = hm.Hold._dense

        def dense(self, kf, img, idx, guess, J, p_lk, _dn=dn):
            r = _dn(self, kf, img, idx, guess, J, p_lk)
            self._calls.append(dict(cur=kf is self.kf, ok=not isinstance(r, str)))
            return r
        hm.Hold._dense = dense
        if hasattr(hm.Hold, "_reloc"):
            rl = hm.Hold._reloc

            def reloc(self, img, t0, att_, hgt_, tref, h_, pos_now, _rl=rl):
                w = time.perf_counter()
                r = _rl(self, img, t0, att_, hgt_, tref, h_, pos_now)
                self._reloc_ms.append(round(1e3 * (time.perf_counter() - w), 2))
                return r
            hm.Hold._reloc = reloc
        K, D = CM.est_lens(hm, cam, c)
        h = hm.Hold(K, D, hm.make_R_bc(), c["readout_s"], **dict(CM.HOLD_CFG, **cfg))
        h._reloc_ms = []
        h.reanchor(truth(te)[0])
        V[name] = dict(h=h, rows=[], good=0, wall=0.0, cfg=cfg, path=path, relocs=[], forced=[], done=set())
    n = int(a.seconds * cam.fps)
    w0 = time.time()
    store, brng = {}, np.random.default_rng(300 + a.seed)
    for k in range(0, n, a.every):
        t0 = te + k / cam.fps + cam.exposure / 2
        ts = t0 - te
        cm = cam2 if (cam2 is not None and a.place_from is not None and ts >= a.place_from) else cam
        img = cm.finish(cm.render(t0, pose_seen))
        if a.bad and a.bad_from <= ts < a.bad_to:
            img = (OLL.bad_picture(a.bad, img, store, brng) if a.bad in OLL.BAD else
                   bad2(a.bad, img, store, brng, ts - a.bad_from))
        tref = t0 + c["readout_s"] / 2
        p_true = truth(tref)[0]
        for name, v in V.items():
            h = v["h"]
            h._calls = []
            nlog, n_ms = len(getattr(h, "reloc_log", [])), len(h._reloc_ms)
            forced = None
            for fi, f in enumerate(forces):
                if fi not in v["done"] and ts >= f[0]:
                    v["done"].add(fi)
                    f_pos = h.pos.copy() + np.array(f[1:3])
                    h.reanchor(f_pos)
                    v["good"], forced = 0, [round(float(x), 3) for x in f_pos - p_true]
                    v["forced"].append(dict(t=round(ts, 3), err_at_reanchor=round(float(np.linalg.norm(f_pos - p_true)), 3)))
            w = time.perf_counter()
            e = h.process(img, t0, att, hgt)
            dt_ms = 1e3 * (time.perf_counter() - w)
            v["wall"] += dt_ms / 1e3
            if a.probe and getattr(h, "map", None) and not h.in_map:
                keep, n_after = (h.RELOC_N, h.t_reloc, list(h.reloc_run), h.note), len(h.reloc_log)
                h.RELOC_N = 10 ** 6
                h._reloc(img, t0, att, hgt, tref, float(hgt.at(tref)), h.pos.copy())
                h.RELOC_N, h.t_reloc, h.reloc_run, h.note = keep
                for rr_ in h.reloc_log[n_after:]:
                    rr_["probe"] = True
            re = False
            if e.state == "lost":
                h.reanchor(p_true if a.reanchor == "truth" else h.pos.copy())
                v["good"], re = 0, True
            if e.var is None:
                if e.state != "degraded":
                    v["good"] = 0
                sent, why = False, e.why or e.state
            else:
                v["good"] += 1
                sent = v["good"] >= 10
                why = "" if sent else f"good fix {v['good']} of 10"
            err = None if e.var is None else float(np.linalg.norm(e.pos - p_true))
            cd = [g for g in h._calls if g["cur"]]
            path = ("dense" if cd and cd[-1]["ok"] else "corner") if e.var is not None else ""
            for rrow in getattr(h, "reloc_log", [])[nlog:]:
                rrow = dict(rrow, t_s=round(ts, 3), ms=h._reloc_ms[-1] if h._reloc_ms else None,
                            fix_err_now=None if err is None else round(err, 3))
                if rrow.get("pos") is not None:
                    rrow["map_err"] = round(float(np.linalg.norm(np.array(rrow["pos"]) - p_true)), 3)
                v["relocs"].append(rrow)
            v["rows"].append(dict(t=round(ts, 4), sent=sent, err=round(err, 4) if (sent and err is not None) else None,
                                  err_all=None if err is None else round(err, 4), state=e.state, why=why, n=e.n, path=path,
                                  dense=[], corner=None, new_kf=None, reanchor=re, forced=forced,
                                  pos=[float(x) for x in e.pos], var=e.var,
                                  ramp=None if getattr(h, "ramp", None) is None else round(float(np.linalg.norm(h.ramp)), 3),
                                  proc_ms=round(dt_ms, 2), tried=len(h._reloc_ms) > n_ms))
    os.makedirs(OUT, exist_ok=True)
    for name, v in V.items():
        s = OLL.summary(v["rows"], a.seconds)
        acc = [r for r in v["relocs"] if r.get("accepted")]
        tries = [r for r in v["relocs"] if not r.get("probe")]
        s.update(relocalizations=len(acc), reloc_tries=len(tries), reloc_passes=sum(1 for r in tries if r["ok"]),
                 reloc_ms_median=float(np.median(v["h"]._reloc_ms)) if v["h"]._reloc_ms else None,
                 reloc_ms_max=float(np.max(v["h"]._reloc_ms)) if v["h"]._reloc_ms else None,
                 forced=v["forced"])
        res = dict(label=f"{label}_{name}", version=name, hold_py=os.path.abspath(v["path"]), hold_cfg=dict(CM.HOLD_CFG, **v["cfg"]),
                   ground=a.ground, lens=a.lens, height=a.height, seed=a.seed, flight=src, every=a.every, hz=cam.fps / a.every,
                   seconds=a.seconds, reanchor_at=a.reanchor, force=a.force, place_from=a.place_from,
                   place_shift=a.place_shift, place_ground=a.place_ground, bad=a.bad, bad_window=[a.bad_from, a.bad_to] if a.bad else None,
                   probe=a.probe, hold_ms_per_frame=round(1e3 * v["wall"] / max(len(v["rows"]), 1), 2),
                   wall_s=round(time.time() - w0, 1), summary=s, relocs=v["relocs"], reloc_ms=v["h"]._reloc_ms,
                   rows=v["rows"])
        if a.bad:
            res["bad_summary"] = OLL.bad_summary(v["rows"], a.bad_from, a.bad_to)
        with gzip.open(os.path.join(OUT, res["label"] + ".json.gz"), "wt") as f:
            json.dump(res, f)
        print(f"{res['label']}: fix {s['fix_hz']}/s, gap {s['longest_gap_s']} s, err RMS {s['err_rms_m']} max {s['err_max_m']} m, "
              f"reanchors {s['reanchors']}, relocs {s['relocalizations']} (tries {s['reloc_tries']}, passes {s['reloc_passes']}), "
              f"reloc ms {s['reloc_ms_median']}/{s['reloc_ms_max']}, hold {res['hold_ms_per_frame']} ms; why {list(s['why'].items())[:4]}",
              flush=True)


if __name__ == "__main__":
    main()
