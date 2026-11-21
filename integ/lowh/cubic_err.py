#!/usr/bin/env python3
"""Offline geometry check (no pictures): how well does hold.py's cubic whole-picture map, fitted to the 8 x 6 keyframe
grid points that lie within 55 deg off nadir (as _make_kf keeps them), follow the true fisheye displacement field of a
drone move d at height h? Per off-nadir band, level attitude, true lens (camera.py geometry), crop and full-sensor lens.
The error depends only on d / h. Run: ~/.venvs/vio/bin/python integ/lowh/cubic_err.py > integ/lowh/results/cubic_err.txt
"""
import sys, math, numpy as np
import os; HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
import common as CM; C = CM.C
import hold
for lens in ('crop', 'full'):
    cam, c = CM.camera('ottawa2022', lens, 11, 0.0, gr=C.Ground(np.zeros((300,300),np.uint8), 1.0))
    K, D = C.lens(c['true_lens'], cam.W, cam.H)
    W, H, s = cam.W, cam.H, 0.25
    xs, ys = np.meshgrid((np.arange(round(W*s))+0.5)/s-0.5, (np.arange(round(H*s))+0.5)/s-0.5)
    pix = np.c_[xs.ravel(), ys.ravel()]
    rc = C.unproject(pix[:,0], pix[:,1], K, D); b = rc @ cam.R_bc.T     # yaw 0, level: body = NED
    th = np.degrees(np.arccos(np.clip(b[:,2],-1,1)))
    gx, gy = 8, 6
    u, v = np.meshgrid((0.1+0.8*(np.arange(gx)+0.5)/gx)*(W-1), (0.1+0.8*(np.arange(gy)+0.5)/gy)*(H-1))
    grid = np.c_[u.ravel(), v.ravel()]
    gb = C.unproject(grid[:,0], grid[:,1], K, D) @ cam.R_bc.T
    gok = gb[:,2] > math.cos(math.radians(55))
    for h in (5.0, 40.0):
        def px(P, dn, de):
            vv = np.c_[P[:,0]-dn, P[:,1]-de, np.full(len(P), h)] @ cam.R_bc
            uu, ww = C.project(vv, K, D); return np.c_[uu, ww]
        okp = b[:,2] > 0.05
        Gp = h*b[okp,:2]/b[okp,2:3]; Gg = h*gb[gok,:2]/gb[gok,2:3]
        for share in (0.05, 0.1, 0.2):
            d = share*h
            for name, (dn, de) in (('fwd', (d, 0.0)), ('right', (0.0, d))):
                true = px(Gp, dn, de) - pix[okp]
                gd = px(Gg, dn, de) - grid[gok]
                Pc = np.linalg.lstsq(hold._cubic(grid[gok], (H, W)), gd, rcond=None)[0]
                fit = hold._cubic(pix[okp], (H, W)) @ Pc
                err = np.linalg.norm(fit - true, axis=1)
                t = th[okp]
                bands = [(0,40),(40,55),(55,90)]
                print(f"{lens} h {h:3.0f} d/h {share:4.2f} {name:5s}: grid pts {gok.sum()}, cubic error px median/max by off-nadir band " +
                      "  ".join(f"{a}-{bb}deg {np.median(err[(t>=a)&(t<bb)]) if ((t>=a)&(t<bb)).any() else float('nan'):5.2f}/{err[(t>=a)&(t<bb)].max() if ((t>=a)&(t<bb)).any() else float('nan'):5.2f}" for a,bb in bands)
                      + f"  (share of pixels >55deg {np.mean(t>55):.2f})")
