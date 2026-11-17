#!/usr/bin/env python3
"""Regression runner: one line per review finding in the CORE scope. ID, PASS / FAIL / INFO, and the
key number before (hold.py version 1, frozen in tests/hold_v1.py, on the same scene) and after.

Scenes use sim_ground's settings (PAL 25 fps, correlated noise 3.7 grey, vibration the FC does not
see), so 'before' can differ from the numbers in review/FINDINGS.md, which used 30 fps and noise 5.
FINDINGS numbers are quoted where a scene is copied from a reviewer script.
Run: ~/.venvs/vio/bin/python tests/run_regression.py     (about 3 min on 14 processes) -> tests/regression.txt
"""
import importlib.util
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2                  # noqa: E402
import numpy as np          # noqa: E402

import bad_pictures as BP   # noqa: E402
import harness as X         # noqa: E402
import hold_v1              # noqa: E402
import hold as HM           # noqa: E402

S, G = X.S, X.G


def make(v1, **cfg):
    return (hold_v1.Hold if v1 else HM.Hold)(S.K, S.D, S.R_BC_NADIR, S.READOUT, **cfg)


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, os.path.dirname(path))
    spec.loader.exec_module(mod)
    return mod


def summ(rec, sel=None):
    s = rec.stats(sel)
    return dict(sent=s["sent_share"], rms=s.get("rms", np.nan), ff=s.get("false_fix", np.nan),
                cw=s.get("conf_wrong", 0), kfs=int(rec.kfs.max()), e2=list(rec.err[rec.sent] ** 2),
                m2=list(rec.m2[rec.sent]), deg_m2=list(rec.m2[rec.sent & (rec.state == "degraded")]),
                ex=list(rec.ex[rec.sent]), ey=list(rec.ey[rec.sent]), t=list(rec.t[rec.sent]),
                end=float(np.mean(rec.err[rec.sent][-25:])) if rec.sent.any() else np.nan)


# ---- scene jobs (module level for the process pool) ----
def j_hover(v1, h0, seed, site="ottawa2019fall", T=20.0, t_err=0.0, yaw=None, cfg=None):
    att_fn = None
    if yaw is not None:
        _, a, _ = S.hover(h0)

        def att_fn(t):
            r = a(t)
            r[:, 2] = np.radians(yaw) * np.sin(2 * np.pi * 0.05 * np.asarray(t))
            return r
    fr, att, hgt, pos = X.scene(site, h0, T, seed, att_fn=att_fn)
    rec = X.run(fr, att, hgt, pos, hold=make(v1, **(cfg or {})) if not v1 else make(v1), t_err=S.FRAME_T_ERR + t_err)
    return summ(rec)


def j_e1(v1, case, seed=1):
    if case.startswith("static"):
        r = BP.job(case, seed, v1=v1, reanchor_err=np.array([0.4, -0.3]))
        return dict(sent_late=r["sent_late"], frames_late=r["frames_late"], kf=r["kf_late"])
    fx = lambda img, P, tr: 128 + 0.1 * (img - 128)
    fr, att, hgt, pos = X.scene(h0=100, T=10.0, seed=seed, ground_fx=fx)
    return summ(X.run(fr, att, hgt, pos, hold=make(v1)))


def j_e4scene(v1, case, seed=1):
    """e4_outliers.py scenes at 100 m; event at 5 s."""
    TE = 5.0
    p, a, hf = S.hover(100)
    kw = {}
    if case == "frozen":
        kw["pos_fn"] = lambda t: p(t) + np.c_[0.5 * np.clip(np.asarray(t) - TE, 0, None), np.zeros(len(t))]
        held = {}
        kw["img_mod"] = lambda t0, img: held.setdefault("img", img) if t0 >= TE else img
    elif case == "big object":
        pyr2, mpp2 = X.site("suburb")
        tex2 = pyr2[1]

        def fx(img, P, tr):
            c = np.c_[np.zeros_like(tr) - 10.0, -20.0 + 3.0 * tr]
            dn, de = P[..., 0] - c[:, None, 0], P[..., 1] - c[:, None, 1]
            m = ((np.abs(dn) < 45) & (np.abs(de) < 45)).astype(np.float32)
            k = 1.0 / (mpp2 * 2)
            v = cv2.remap(tex2, (de * k + 900).astype(np.float32), (-dn * k + 900).astype(np.float32), cv2.INTER_LINEAR)
            return img * (1 - m) + np.clip(128 + 2.0 * (v - v.mean()), 0, 255) * m
        kw["ground_fx"] = fx
    fr, att, hgt, pos = X.scene(h0=100, T=15.0, seed=seed, **kw)
    rec = X.run(fr, att, hgt, pos, hold=make(v1))
    out = summ(rec, rec.t >= TE)
    out["n_deg"] = int(np.sum(rec.sent & (rec.state == "degraded")))
    out["sent_late"] = int(np.sum(rec.sent & (rec.t >= TE + 0.5)))          # 0.5 s after the event
    return out


def j_osd(v1, seed=1):
    import overlay as O
    r = O.job("overlay, no mask, ground 40 % contrast", seed) if not v1 else None
    if r is not None:
        return dict(rms=r["rms"], cw=r["cw"], ff=r["ff"])
    pic, drawn, _ = O.osd()
    p, a, hf = S.hover(100)
    pos = lambda t: p(t) + np.c_[0.5 * np.clip(np.asarray(t) - 5.0, 0, None), np.zeros(len(t))]

    def mod(t0, img):
        out = img.copy()
        out[drawn] = pic[drawn]
        return out
    fr, att, hgt, pos = X.scene(h0=100, T=15.0, seed=seed, pos_fn=pos, ground_fx=lambda img, P, tr: 128 + 0.4 * (img - 128),
                                img_mod=mod)
    s = summ(X.run(fr, att, hgt, pos, hold=make(True)))
    return dict(rms=s["rms"], cw=s["cw"], ff=s["ff"])


def j_e7(v1, h0):
    return BP.job_e7(h0, 1, v1=v1)


def j_h4(v1, case):
    r = BP.job(case, 1, v1=v1)
    return dict(sent_bad=r["sent_bad"], sent_late=r["sent_late"], frames_late=r["frames_late"], back=r["back"])


def j_compute(v1):
    cv2.setNumThreads(1)
    fr, att, hgt, pos = X.scene(h0=100, T=12.0, seed=1)
    best = []
    for _ in range(3):
        hold = make(v1)
        hold.reanchor(pos(np.array([0.0]))[0])
        ts = []
        for i, (t0, img) in enumerate(fr):
            c0 = time.perf_counter()
            hold.process(img, t0 + S.FRAME_T_ERR, att, hgt)
            if i >= 30:
                ts.append(time.perf_counter() - c0)
        best.append((1e3 * np.mean(ts), 1e3 * np.percentile(ts, 95)))
    return dict(ms=sorted(best)[1])


def j_e5(v1, h0, dpsi, beta=1.0):
    p, a, hf = S.hover(h0)

    def att2(t):
        r = a(t)
        r[:, 2] = np.clip(1.0 * (np.asarray(t) - 3.0), 0, math.radians(dpsi))
        return r
    bias = S.ATT_BIAS.copy()
    S.ATT_BIAS[:] = np.radians([beta, 0.0, 0.0])
    try:
        fr, att, hgt, pos = X.scene(h0=h0, T=10.0, seed=1, att_fn=att2)
    finally:
        S.ATT_BIAS[:] = bias
    rec = X.run(fr, att, hgt, pos, hold=make(v1))
    return summ(rec, rec.t > 8.0)


def j_geom(v1, case):
    """review/geometry scenes (their own renderer). G3: 300 m shore, yaw error 0 -> 1 deg; G6: reversed sensor."""
    I = _load("indep", os.path.join(X.PROJ, "review", "geometry", "indep.py"))
    CH = _load("common_hover", os.path.join(X.PROJ, "review", "geometry", "common_hover.py"))
    CH.Hold = hold_v1.Hold if v1 else HM.Hold
    if case == "G3":
        pos, att, hf = CH.hover_path(300, still=True)
        tex = I.make_texture(seed=5, water_half="west")
        ramp = lambda t: np.clip(t / 10.0, 0, 1) * math.radians(1.0)
        out, _ = CH.run(300, pos, att, hf, tex=tex, att_err_fn=lambda t: np.c_[0 * t, 0 * t, ramp(t)])
        e = np.array([[o[2], o[3]] for o in out if o[0] > 9.5], float)
        return dict(err=float(np.nanmean(np.linalg.norm(e, axis=1))))
    pos, att, hf = CH.hover_path(300)
    out, _ = CH.run(300, pos, att, hf, T=6.0, tex=I.make_texture(seed=5), reverse=True, ro_est=-0.016,
                    t0_shift=0.016 * (I.H - 1) / I.H)          # t0 = time of the top row, read last (t4_readout_direction.py)
    return dict(rms=CH.rms(out)[0], used=CH.rms(out)[1])


def j_h9(v1, baro):
    """t8_height.py part B: true height 3 m, the drone moves 2 m east in 10 s, the baro reads `baro`."""
    HX = _load("hw_harness", os.path.join(X.PROJ, "review", "hardware_gap", "harness.py"))
    HX.Hold = hold_v1.Hold if v1 else HM.Hold
    g = HX.Ground("synth", 3.0)
    _, att_fn, _ = HX.hover(3.0)
    small = lambda t: 0.3 * att_fn(t)
    pos = lambda t: np.c_[np.zeros(len(t)), 0.2 * np.asarray(t)]
    h_fn = lambda t: np.full(len(t), 3.0)
    frames = HX.frames_for(pos, small, h_fn, g, T=4.0)
    att, _ = HX.fc_feeds(small, h_fn, 4.0)
    hgt = HX.Series(1e9)
    for t in np.arange(-1, 5, 0.1):
        hgt.add(t, baro)
    try:
        rows = HX.run(frames, att, hgt, pos)
    except ZeroDivisionError as ex:
        return dict(crash=str(ex), sent=0, ok=0)
    return dict(crash=None, sent=sum(r[2] is not None for r in rows), ok=sum(r[1] == "ok" for r in rows), n=len(rows))


def j_e6(v1, seed):
    fr, att, hgt, pos = X.scene("prairie", 50, 20.0, seed)
    rec = X.run(fr, att, hgt, pos, hold=make(v1))
    s = summ(rec)
    s["end"] = float(np.nanmean(rec.err[-25:]))
    return s


def lens_checks():
    path = os.path.join(X.PROJ, "justice_lens_profiles.json")
    K0, _ = HM.load_lens(path, "Round 2 independent k2", 360, 240)
    K_even, _ = HM.load_lens(path, "Round 2 independent k2", 360, 240, field=0)
    K_odd, _ = HM.load_lens(path, "Round 2 independent k2", 360, 240, field=1)
    K, D = HM.load_lens(path, "Round 2 Metashape Frame", 360, 240)
    h = HM.Hold(K, D, HM.R_BC_NADIR, 0.0)
    th = math.radians(40)
    x, y = math.tan(th), 0.0                              # Brown model forward, as t6_lens_field.py
    r2 = x * x
    p = [kk for kk in D]
    k1, k2, p1, p2, k3 = p
    xd = x * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    n = h._undistort(np.array([[K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]]]))[0]
    v1K, v1D = hold_v1.load_lens(path, "Round 2 Metashape Frame", 360, 240)
    nv1 = cv2.fisheye.undistortPoints(np.array([[[K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]]]]), v1K, v1D).reshape(2)
    try:
        HM.load_lens(path, "Round 1 Metashape fisheye", 360, 240)
        raised = False
    except ValueError:
        raised = True
    return dict(cy_shift=(K_even[1, 2] - K0[1, 2], K_odd[1, 2] - K0[1, 2]), ang=math.degrees(math.atan(n[0])),
                ang_v1=math.degrees(math.atan(nv1[0])), raised=raised)


JOBS = ([("E1", j_e1, dict(v1=v, case=c)) for v in (True, False) for c in ("static std 5", "static std 30", "contrast 10 %")]
        + [("E2", j_e4scene, dict(v1=v, case="frozen")) for v in (True, False)]
        + [("E2osd", j_osd, dict(v1=v)) for v in (True, False)]
        + [("E3", j_e4scene, dict(v1=v, case="big object", seed=s)) for v in (True, False) for s in (1, 2, 3)]
        + [("E4", j_hover, dict(v1=v, h0=h, seed=s)) for v in (True, False) for h in (100, 200, 300, 400) for s in range(1, 7)]
        + [("E4b0", j_hover, dict(v1=False, h0=h, seed=s, cfg=dict(sig_bias=0.0))) for h in (100, 400) for s in range(1, 7)]
        + [("E4yaw", j_hover, dict(v1=v, h0=100, seed=s, yaw=60)) for v in (True, False) for s in range(1, 4)]
        + [("E5", j_e5, dict(v1=v, h0=100, dpsi=90)) for v in (True, False)]
        + [("E6", j_hover, dict(v1=v, h0=100, seed=s, yaw=y)) for v in (True, False) for y in (60, 90) for s in (1, 3)]
        + [("E6p", j_e6, dict(v1=v, seed=1)) for v in (True, False)]
        + [("E7", j_e7, dict(v1=v, h0=h)) for v in (True, False) for h in (20, 50, 100)]
        + [("E9", j_compute, dict(v1=v)) for v in (True, False)]
        + [("G3", j_geom, dict(v1=v, case="G3")) for v in (True, False)]
        + [("G6", j_geom, dict(v1=v, case="G6")) for v in (True, False)]
        + [("H4", j_h4, dict(v1=v, case=c)) for v in (True, False) for c in ("frozen picture", "frozen + fresh noise", "NO SIGNAL screen")]
        + [("H9", j_h9, dict(v1=v, baro=b)) for v in (True, False) for b in (0.0, -0.5, 3.0)]
        + [("H13", j_hover, dict(v1=v, h0=100, seed=s, t_err=e)) for v in (True, False) for s in range(1, 4) for e in (0.005, 0.020)])


def main():
    t_start = time.time()
    jobs = sorted(JOBS, key=lambda j: j[1] is not j_compute)        # timing first, while the pool is quiet
    res = X.pmap(_run, [dict(i=i, fn=fn, kw=kw) for i, (_, fn, kw) in enumerate(jobs)])
    R = {}
    for (tag, _, kw), r in zip(jobs, res):
        R.setdefault(tag, []).append((kw, r))
    get = lambda tag, **sel: [r for kw, r in R[tag] if all(kw.get(k) == v for k, v in sel.items())]
    pool = lambda rs, key: np.concatenate([np.array(r[key], float) for r in rs]) if rs else np.array([])
    ff = lambda rs: float(np.mean(pool(rs, "m2") > X.CHI2_99)) if len(pool(rs, "m2")) else np.nan
    rms = lambda rs: float(np.sqrt(np.mean(pool(rs, "e2")))) if len(pool(rs, "e2")) else np.nan
    L = []

    def line(fid, ok, text):
        L.append(f"{fid:4s} {ok:4s}  {text}")
        print(L[-1], flush=True)

    b = {c: get("E1", v1=True, case=c)[0] for c in ("static std 5", "static std 30", "contrast 10 %")}
    a = {c: get("E1", v1=False, case=c)[0] for c in ("static std 5", "static std 30", "contrast 10 %")}
    ok = all(a[c]["sent_late"] == 0 and a[c]["kf"] == 0 for c in ("static std 5", "static std 30")) and \
        (a["contrast 10 %"]["sent"] == 0 or a["contrast 10 %"]["ff"] <= 0.03)
    line("E1", "PASS" if ok else "FAIL",
         "static std 5/30, caller re-anchors: fixes sent 0.5-5 s into static "
         f"{b['static std 5']['sent_late']}/{b['static std 30']['sent_late']} -> {a['static std 5']['sent_late']}/{a['static std 30']['sent_late']}"
         f" of {a['static std 5']['frames_late']}, keyframes on static {b['static std 5']['kf']}/{b['static std 30']['kf']} -> "
         f"{a['static std 5']['kf']}/{a['static std 30']['kf']}; ground at 10 % contrast: sent {b['contrast 10 %']['sent']:.0%} "
         f"RMS {b['contrast 10 %']['rms']:.2f} m false-fix {b['contrast 10 %']['ff']:.1%} -> sent {a['contrast 10 %']['sent']:.0%} "
         f"RMS {a['contrast 10 %']['rms']:.2f} m false-fix {a['contrast 10 %']['ff']:.1%} (FINDINGS: 29/75 fixes, 9/4 keyframes)")
    b, a = get("E2", v1=True)[0], get("E2", v1=False)[0]
    ob, oa = get("E2osd", v1=True)[0], get("E2osd", v1=False)[0]
    line("E2", "PASS" if a["cw"] == 0 and a["sent_late"] == 0 and oa["rms"] < 1.0 and oa["ff"] <= 0.03 else "FAIL",
         f"frozen frame + 0.5 m/s drift: confident-wrong fixes {b['cw']} -> {a['cw']}, fixes sent 0.5-10 s after the freeze "
         f"{b['sent_late']} -> {a['sent_late']}; static OSD on 40 % contrast ground, no mask: RMS {ob['rms']:.2f} -> {oa['rms']:.2f} m, "
         f"false-fix {ob['ff']:.1%} -> {oa['ff']:.1%}, confident-wrong {ob['cw']} -> {oa['cw']} "
         f"(FINDINGS: 288 confident-wrong; OSD RMS 8.07 m)")
    b, a = get("E3", v1=True), get("E3", v1=False)
    line("E3", "PASS" if rms(a) < 1.5 and ff(a) <= 0.03 else "FAIL",
         f"textured object on ~27 % of the picture moving 3 m/s, 3 seeds: RMS after 5 s {rms(b):.2f} -> {rms(a):.2f} m "
         f"(per seed {'/'.join(f'{r['rms']:.2f}' for r in a)}), confident-wrong {sum(r['cw'] for r in b)} -> {sum(r['cw'] for r in a)}, "
         f"false-fix {ff(b):.1%} -> {ff(a):.1%} (FINDINGS: 29.57 m, 296)")
    bs, as_ = get("E4", v1=True), get("E4", v1=False)
    per_h = " ".join(f"{h} m {ff(get('E4', v1=True, h0=h)):.1%}->{ff(get('E4', v1=False, h0=h)):.1%}" for h in (100, 200, 300, 400))
    m2 = pool(as_, "m2")
    yb, ya = get("E4yaw", v1=True), get("E4yaw", v1=False)
    b0 = get("E4b0")
    line("E4", "PASS" if ff(as_) <= 0.03 and ff(ya) <= 0.03 else "FAIL",
         f"outside the 99 % bound, 6 seeds x 4 heights: {ff(bs):.1%} -> {ff(as_):.1%} ({per_h}); +-60 deg yaw at 100 m "
         f"{ff(yb):.1%} -> {ff(ya):.1%}; outside the 50 % bound {np.mean(m2 > 1.386):.1%} (ideal 50 %; "
         f"{np.mean(pool(b0, 'm2') > 1.386):.1%} with sig_bias=0, false-fix {ff(b0):.1%}) (FINDINGS: 12.9-23.7 %)")
    b, a = get("E5", v1=True)[0], get("E5", v1=False)[0]
    line("E5", "INFO", f"not in CORE scope (boresight calibration): 1 deg mount error, 90 deg turn at 100 m: RMS {b['rms']:.2f} -> "
         f"{a['rms']:.2f} m, false-fix {b['ff']:.0%} -> {a['ff']:.0%} (v2 assumes BETA_UNC 0.6 deg)")
    b, a = get("E6", v1=True), get("E6", v1=False)
    kb, ka = [r["kfs"] - 1 for r in b], [r["kfs"] - 1 for r in a]
    pb, pa = get("E6p", v1=True)[0], get("E6p", v1=False)[0]
    line("E6", "PASS" if rms(a) <= rms(b) and ff(a) <= 0.03 else "FAIL",
         f"100 m with +-60 / +-90 deg yaw swings, seeds 1 and 3: keyframes made {kb} -> {ka}, RMS {rms(b):.2f} -> {rms(a):.2f} m, "
         f"end error {[round(r['end'], 2) for r in b]} -> {[round(r['end'], 2) for r in a]} m, false-fix {ff(b):.1%} -> {ff(a):.1%}; "
         f"prairie 50 m (noise-dominated photo): v1 {pb['kfs'] - 1} keyframes, RMS {pb['rms']:.2f} m, end {pb['end']:.2f} m -> "
         f"v2 sends {pa['sent']:.0%} ('no ground') (FINDINGS: 3/11/15 changes, RMS 0.82/1.24/2.10 m at +-30/60/90 deg)")
    b, a = get("E7", v1=True), get("E7", v1=False)
    fmt_back = lambda rs: "/".join("never" if np.isnan(r["back"]) else f"{r['back']:.2f}" for r in rs)
    line("E7", "PASS" if all(r["back"] < 1.0 for r in a) else "FAIL",
         f"3 s static while flying 1 m/s, no reanchor(), 20/50/100 m: fixes back after {fmt_back(b)} -> {fmt_back(a)} s, "
         f"RMS after {'/'.join(f'{r['err_after']:.2f}' for r in a)} m (FINDINGS: never)")
    rb, ra = [r for kw, r in R["E4"] if kw["v1"] and kw["h0"] == 400], [r for kw, r in R["E4"] if not kw["v1"] and kw["h0"] == 400]
    w5b, w5a = [r for kw, r in R["E4"] if kw["v1"] and kw["h0"] == 400 and kw["seed"] == 5][0], \
        [r for kw, r in R["E4"] if not kw["v1"] and kw["h0"] == 400 and kw["seed"] == 5][0]
    line("E8", "PASS" if w5a["ff"] <= 0.03 else "FAIL",
         f"sim_ground reports pooled seeds (sim_ground.py --seeds); 400 m RMS per seed {[round(r['rms'], 2) for r in ra]} m; "
         f"seed 5 false-fix {w5b['ff']:.1%} -> {w5a['ff']:.1%} (FINDINGS: 50.8 %)")
    b, a = get("E9", v1=True)[0]["ms"], get("E9", v1=False)[0]["ms"]
    line("E9", "PASS" if a[0] <= 10 else "FAIL", f"ms per frame, 1 thread, 360 x 240: mean {b[0]:.1f} -> {a[0]:.1f}, p95 {b[1]:.1f} -> "
         f"{a[1]:.1f} (same machine and run; FINDINGS: 17.7 mean)")
    line("H6", "PASS" if a[0] <= 10 else "FAIL", f"same numbers as E9 (FINDINGS: 29.08 ms mean); A53 estimate x10: {10 * a[0]:.0f} ms")
    degs = [get("E2", v1=False)[0], get("E3", v1=False)[0]] + get("E4", v1=False)
    dm2 = pool(degs, "deg_m2")
    line("E10", "PASS" if len(dm2) == 0 or np.mean(dm2 > X.CHI2_99) <= 0.03 else "FAIL",
         f"'degraded' fixes carry {HM.Hold.DEGRADED_K:g}x the covariance; outside the 99 % bound among degraded fixes "
         f"(frozen, big object, hovers): {np.mean(dm2 > X.CHI2_99) if len(dm2) else float('nan'):.1%} of {len(dm2)} (FINDINGS: 97 %)")
    rb1, ra1 = rms(get("E4", v1=True, h0=100)), rms(get("E4", v1=False, h0=100))
    line("E11", "PASS" if ra1 <= 1.05 * rb1 else "FAIL",
         f"velocity filter removed (Hold.vel stays 0); RMS at 100 m, 6 seeds {rb1:.2f} -> {ra1:.2f} m (FINDINGS: no effect on RMS)")
    jit = lambda rs: float(np.mean([np.std(np.diff(np.array(r["ex"]))) / math.sqrt(2) for r in rs]))
    line("E12", "INFO", f"not in CORE scope (EKF R / pre-filter): frame jitter per axis at 100 m {jit(get('E4', v1=True, h0=100)):.3f} -> "
         f"{jit(get('E4', v1=False, h0=100)):.3f} m (FINDINGS: 0.241 m)")
    b, a = get("G3", v1=True)[0], get("G3", v1=False)[0]
    line("G3", "PASS" if a["err"] < 0.3 else "FAIL", f"300 m, water on one side, FC yaw error 0 -> 1 deg: error at the end {b['err']:.2f} -> "
         f"{a['err']:.2f} m (FINDINGS: 2.59 m; similarity fit in t5b 0.04 m)")
    line("G4", "PASS" if "mid-exposure" in HM.Hold.process.__doc__ else "FAIL",
         "t0 is the mid-exposure time of row 0 (Hold.process and _rays docstrings); the exposure offset is calibrated outside Hold")
    b, a = get("G6", v1=True)[0], get("G6", v1=False)[0]
    line("G6", "PASS" if a["rms"] < 0.1 and a["used"] > 0.9 else "FAIL",
         f"sensor read bottom to top, readout -16 ms, 300 m: RMS {b['rms']:.2f} -> {a['rms']:.2f} m, fixes {b['used']:.0%} -> {a['used']:.0%} "
         "(FINDINGS: 0.04 m)")
    lc = lens_checks()
    line("G8", "PASS" if abs(lc["cy_shift"][0] - 0.25) < 1e-9 and abs(lc["cy_shift"][1] + 0.25) < 1e-9 else "FAIL",
         f"load_lens(field=0/1) moves cy by {lc['cy_shift'][0]:+.2f}/{lc['cy_shift'][1]:+.2f} row against an area resize (G8: +-0.25)")
    line("G9", "PASS" if abs(lc["ang"] - 40) < 0.1 and lc["raised"] else "FAIL",
         f"'Round 2 Metashape Frame': a 40.0 deg ray reads {lc['ang_v1']:.1f} -> {lc['ang']:.1f} deg; a fisheye profile with p1/p2 "
         f"{'raises ValueError' if lc['raised'] else 'is accepted'} (FINDINGS: 48.1 deg)")
    b = {c: get("H4", v1=True, case=c)[0] for c in ("frozen picture", "frozen + fresh noise", "NO SIGNAL screen")}
    a = {c: get("H4", v1=False, case=c)[0] for c in b}
    line("H4", "PASS" if all(a[c]["sent_late"] == 0 for c in a) else "FAIL",
         "fixes sent 0.5-5 s into bad video (frozen / frozen + noise / NO SIGNAL): " +
         "/".join(str(b[c]["sent_late"]) for c in b) + " -> " + "/".join(str(a[c]["sent_late"]) for c in a) +
         f" of {a['frozen picture']['frames_late']}; fixes back {'/'.join(f'{a[c]['back']:.2f}' for c in a)} s after the video returns")
    b = {kw["baro"]: r for kw, r in R["H9"] if kw["v1"]}
    a = {kw["baro"]: r for kw, r in R["H9"] if not kw["v1"]}
    line("H9", "PASS" if all(a[x]["crash"] is None and a[x]["sent"] == 0 for x in (0.0, -0.5, 3.0)) else "FAIL",
         "true height 3 m, baro 0.0 / -0.5 / 3.0 m: " + " / ".join(
             ("crash" if b[x]["crash"] else f"{b[x]['sent']} sent") for x in (0.0, -0.5, 3.0)) + " -> " + " / ".join(
             ("crash" if a[x]["crash"] else f"{a[x]['sent']} sent") for x in (0.0, -0.5, 3.0)) +
         f" (H_MIN {HM.Hold.H_MIN:g} m; FINDINGS: ZeroDivisionError, 'ok' 93 % at -0.5 m)")
    b5, a5 = get("H13", v1=True, t_err=0.005), get("H13", v1=False, t_err=0.005)
    b20, a20 = get("H13", v1=True, t_err=0.020), get("H13", v1=False, t_err=0.020)
    line("H13", "PASS" if ff(a5) <= 0.03 else "FAIL",
         f"Est.cov is a 2x2 per-axis covariance (var = trace) with a timing term (TIME_ERR {HM.Hold.TIME_ERR * 1e3:g} ms x rate x h); "
         f"100 m, 3 seeds, outside the 99 % bound with a +5 ms stamp error {ff(b5):.1%} -> {ff(a5):.1%}, with +20 ms {ff(b20):.1%} -> "
         f"{ff(a20):.1%} (RMS {rms(b20):.2f} -> {rms(a20):.2f} m) (FINDINGS: +20 ms 89 %)")
    L.append(f"run time {time.time() - t_start:.0f} s")
    open(os.path.join(X.HERE, "regression.txt"), "w").write(
        "ID   result  before (hold.py version 1 on the same scene) -> after (version 2)\n" + "\n".join(L) + "\n")


def _run(i, fn, kw):
    return fn(**kw)


if __name__ == "__main__":
    main()
