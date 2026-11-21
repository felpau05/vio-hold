#!/usr/bin/env python3
"""Controlled open-loop probe of Hold's whole-picture path at low height: the halves gap against the drift from the
keyframe, with nothing else moving.

One keyframe is rendered at a ground spot with a level attitude; then single frames are rendered with the drone moved
by d (body forward, back, right or left) at the same height and heading, or with d = 0 (still hover, fresh sensor noise
and camera vibration). Each frame goes through Hold._track1 in the whole-picture mode (kf['dense'] forced on, also on
fine ground where the corner path would run), and the probe keeps the halves gap (Hold.dq[1]), the two halves' drone
moves (instrumented, common.capture_ds), the peak and the match quality, the refusal text and the fix error.
The attitude given to Hold is the true one (no FC noise); the camera vibration (vib_deg, FC does not see it) stays on
unless --vib 0. --tilt r,p: the test frames are rolled / pitched by r, p deg against the level keyframe (lens-error
test). --exact_lens: Hold gets the true lens (diagnostic). --pos: where Hold's last fix (Hold.pos) is set before each
frame: kf (the keyframe position, as if no fix since the keyframe) or truth (the true drift: a perfect last fix).
Only the patched hold.py reads Hold.pos in the whole-picture path; the current one does not.

Each frame also keeps the gap at the picture centre's scale (gap_centre, the repository's metric) and in px of each
half's own texture (gap_local, common.local_gap), whatever the loaded hold.py measures (its own value: gap).
Also written: the geometric prediction of the same gap, without pictures (geom_gap): every pixel of the dense picture
(160 x 90) is moved by the true lens for the drone move d (camera.py geometry); the model displacement is
J_N d_N + J_E d_E with J from 1 m forward steps north and east at the keyframe position (as hold.py _track1); each half is
fitted by least squares (uniform weight) and the gap is |Jc (dl_top - dl_bottom)|. The same with a tangent J (0.01 m
central step) is written as geom_gap_tangent.

Run: ~/.venvs/vio/bin/python integ/lowh/probe.py --hold hold.py --label before --ground ottawa2022 [--lens crop]
     [--heights 5,10,20,40] [--seeds 11,12,13] [--vib 0.2] [--tilt 0,0] [--exact_lens] [--pos kf]
Writes integ/lowh/results/probe/<label>_<ground>_<lens>[...].json (refuses to write over a file).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import common as CM   # noqa: E402

C = CM.C
DIRS = {"fwd": (1.0, 0.0), "back": (-1.0, 0.0), "right": (0.0, 1.0), "left": (0.0, -1.0)}


def geom_gap(cam, K_true, D_true, h, d_ne, yaw, step=1.0, central=False):
    """Geometric halves gap (px) for drone move d_ne at height h, level attitude, true lens, no picture."""
    W, H, s = cam.W, cam.H, 0.25
    xs, ys = np.meshgrid((np.arange(round(W * s)) + 0.5) / s - 0.5, (np.arange(round(H * s)) + 0.5) / s - 0.5)
    rays_c = C.unproject(xs.ravel(), ys.ravel(), K_true, D_true)        # camera frame
    R_nb = C.q_matrix(CM.quat(0.0, 0.0, yaw))
    b = rays_c @ cam.R_bc.T @ R_nb.T                                    # NED
    ok = b[:, 2] > math.cos(math.radians(80))
    G = h * b[:, :2] / b[:, 2:3]                                        # ground offsets from the keyframe position

    def px(dn, de):
        v = np.c_[G[:, 0] - dn, G[:, 1] - de, np.full(len(G), h)]
        c = v @ R_nb @ cam.R_bc
        u, w = C.project(c, K_true, D_true)
        return np.c_[u, w]
    p0 = px(0, 0)
    if central:
        e = 0.01
        JN, JE = (px(e, 0) - px(-e, 0)) / (2 * e), (px(0, e) - px(0, -e)) / (2 * e)
    else:
        JN, JE = px(step, 0) - p0, px(0, step) - p0
    true = px(*d_ne) - p0
    half = ys.ravel() < H / 2
    dls = []
    for m in (half & ok, ~half & ok):
        A = np.stack([JN[m], JE[m]], -1)                                # n x 2 (px) x 2 (N, E)
        M = np.einsum("nki,nkj->ij", A, A)
        rhs = np.einsum("nki,nk->i", A, true[m])
        dls.append(np.linalg.solve(M, rhs))
    c = np.argmin(np.hypot(xs.ravel() - (W - 1) / 2, ys.ravel() - (H - 1) / 2))
    Jc = np.stack([JN[c], JE[c]], -1)
    return float(np.linalg.norm(Jc @ (dls[0] - dls[1]))), [x.tolist() for x in dls]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", default=os.path.join(CM.PROJ, "hold.py"))
    ap.add_argument("--label", required=True)
    ap.add_argument("--ground", default="ottawa2022")
    ap.add_argument("--lens", default="crop", choices=list(CM.LENSES))
    ap.add_argument("--heights", default="5,10,20,40")
    ap.add_argument("--seeds", default="11,12,13")
    ap.add_argument("--shares", default="0.05,0.1,0.2,0.3")
    ap.add_argument("--still", type=int, default=6, help="still-hover frames per height and seed")
    ap.add_argument("--vib", type=float, default=0.2)
    ap.add_argument("--tilt", default="0,0")
    ap.add_argument("--exact_lens", action="store_true")
    ap.add_argument("--pos", default="kf", choices=["kf", "truth"])
    a = ap.parse_args()
    seeds = [int(x) for x in a.seeds.split(",")]
    if any(s not in (11, 12, 13) for s in seeds):
        raise SystemExit("validation seeds 11, 12, 13 only (DATA_SPLITS.md)")
    tilt = [math.radians(float(x)) for x in a.tilt.split(",")]
    name = (f"{a.label}_{a.ground}_{a.lens}" + (f"_vib{a.vib:g}" if a.vib != 0.2 else "")
            + (f"_tilt{a.tilt.replace(',', '_')}" if a.tilt != "0,0" else "") + ("_exactlens" if a.exact_lens else "")
            + (f"_pos{a.pos}" if a.pos != "kf" else ""))
    out = os.path.join(HERE, "results", "probe", name + ".json")
    if os.path.exists(out) or os.path.exists(out + ".gz"):
        raise SystemExit(f"{out} exists")
    hm = CM.load_hold(os.path.abspath(a.hold), "hold_probe")
    CM.capture_ds(hm)
    import cv2
    cv2.setNumThreads(2)
    gr = CM.ground(a.ground)
    rows = []
    for seed in seeds:
        cam, c = CM.camera(a.ground, a.lens, seed, a.vib, gr)
        K, D = CM.est_lens(hm, cam, c, exact=a.exact_lens)
        Kt, Dt = C.lens(c["true_lens"], cam.W, cam.H)
        rng = np.random.default_rng(5000 + seed)
        for h in [float(x) for x in a.heights.split(",")]:
            span = 8.0 if a.ground == "lowh_fine" else 400.0
            N0, E0 = rng.uniform(-span, span, 2)
            yaw = float(rng.uniform(0, 2 * math.pi))
            fwd, rgt = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
            tests = [("still", 0.0, (0.0, 0.0))] * a.still + [
                (dn, sh, DIRS[dn]) for sh in [float(x) for x in a.shares.split(",")] for dn in DIRS]
            hld = hm.Hold(K, D, hm.make_R_bc(), c["readout_s"], **CM.HOLD_CFG)
            T = 10.0 * seed + 1000.0 * h

            def feeds(t, roll, pitch):
                att, hgt = hm.Series(1e9), hm.Series(1e9)
                for tt in np.arange(t - 1.0, t + 1.0, 0.01):
                    att.add(tt, roll, pitch, yaw, 0.0, 0.0, 0.0)
                    hgt.add(tt, h)
                return att, hgt

            def pose_at(dne, roll, pitch):
                q = CM.quat(roll, pitch, yaw)

                def pose(t):
                    n = len(np.atleast_1d(t))
                    return np.full(n, N0 + dne[0]), np.full(n, E0 + dne[1]), np.full(n, h), np.tile(q, (n, 1))
                return pose
            t0 = T + cam.exposure / 2
            img = cam.finish(cam.render(t0, pose_at((0.0, 0.0), 0.0, 0.0)))
            att, hgt = feeds(t0, 0.0, 0.0)
            chain = dict(h=h, t=t0, yaw=yaw, carried=0.0, w=0.0)
            kf = hld._make_kf(img, t0, att, hgt, t0 + c["readout_s"] / 2, np.zeros(2), 0.0, chain, pending=False)
            corners = int((~kf["virt"]).sum())
            kf["dense"] = True
            hld.kf, hld.home = kf, kf
            for j, (dname, share, (uf, ur)) in enumerate(tests):
                d = share * h * (uf * fwd + ur * rgt)
                t0 = T + 0.5 + 0.0731 * j + cam.exposure / 2      # irregular times: a new vibration phase each frame
                img = cam.finish(cam.render(t0, pose_at(d, tilt[0], tilt[1])))
                att, hgt = feeds(t0, tilt[0], tilt[1])
                hld.pos = d.copy() if a.pos == "truth" else np.zeros(2)
                hld.dq, hld._ds, hld._Jc, hld._AB = [None, None, None], None, None, None
                r = hld._track1(kf, img, t0, att, hgt, t0 + c["readout_s"] / 2, False, None, True)
                row = dict(seed=seed, h=h, dir=dname, share=share, d_ne=d.round(4).tolist(), yaw=round(yaw, 4),
                           kf_corners=corners, resp=hld.dq[0], gap=hld.dq[1], rho=hld.dq[2],
                           why=r if isinstance(r, str) else "", ok=isinstance(r, dict))
                if hld._ds is not None:
                    ds = [np.asarray(x) for x in hld._ds]
                    row.update(dl_top=[float(ds[0] @ fwd), float(ds[0] @ rgt)], dl_bot=[float(ds[1] @ fwd), float(ds[1] @ rgt)])
                    row.update(gap_centre=float(np.linalg.norm(hld._Jc @ (ds[0] - ds[1]))),
                               gap_local=CM.local_gap(hld._AB, ds, hm.Hold.DENSE_SCALE))
                if isinstance(r, dict):
                    e = r["pos"] - d
                    row.update(err_fwd=float(e @ fwd), err_right=float(e @ rgt))
                if share > 0:
                    row["geom_gap"], row["geom_dl"] = geom_gap(cam, Kt, Dt, h, d, yaw)
                    row["geom_gap_tangent"], _ = geom_gap(cam, Kt, Dt, h, d, yaw, central=True)
                rows.append(row)
            st = [r["gap"] for r in rows if r["seed"] == seed and r["h"] == h and r["dir"] == "still" and r["gap"] is not None]
            print(f"seed {seed} h {h:g} m: keyframe corners {corners}, still gap median "
                  f"{np.median(st) if st else float('nan'):.2f} px", flush=True)
    res = dict(label=name, hold_py=os.path.abspath(a.hold), ground=a.ground, lens=a.lens, true_lens=CM.LENSES[a.lens],
               vib_deg=a.vib, tilt_deg=a.tilt, exact_lens=a.exact_lens, pos=a.pos, hold_cfg=CM.HOLD_CFG,
               dense_agree_px=getattr(hm.Hold, "DENSE_AGREE_PX", None), rows=rows)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(res, open(out, "w"))
    print("wrote", out)


if __name__ == "__main__":
    main()
