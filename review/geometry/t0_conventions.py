#!/usr/bin/env python3
"""Items 1, 2, 5: compare the project's euler_R, R_BC_NADIR and load_lens with own derivations.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t0_conventions.py
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                                    # noqa: E402
from hold import euler_R, R_BC_NADIR, load_lens      # noqa: E402  (this script compares them)

rng = np.random.default_rng(0)
deg = math.radians

# 1. euler_R vs quaternion built from the physical meaning of roll, pitch, yaw
r = rng.uniform(-math.pi, math.pi, 2000)
p = rng.uniform(-1.4, 1.4, 2000)
y = rng.uniform(-math.pi, math.pi, 2000)
Rq = I.quat_to_matrix(I.att_quat(r, p, y))
Re = euler_R(r, p, y)
print(f"[1] euler_R vs physical quaternion, 2000 random attitudes: max |diff| = {np.abs(Rq - Re).max():.2e}")
nose_up = euler_R(0.0, deg(10), 0.0) @ I.E_N
wing_down = euler_R(deg(10), 0.0, 0.0) @ I.E_E
yaw_east = euler_R(0.0, 0.0, deg(90)) @ I.E_N
print(f"    pitch +10 deg: nose in NED = {np.round(nose_up, 3)} (D < 0 means nose up)")
print(f"    roll  +10 deg: right wing in NED = {np.round(wing_down, 3)} (D > 0 means right wing down)")
print(f"    yaw   +90 deg: nose in NED = {np.round(yaw_east, 3)} (E = 1 means nose east)")
print(f"    det = {np.linalg.det(Re[:5]).round(12)}  orthogonality err = "
      f"{np.abs(np.einsum('nji,njk->nik', Re, Re) - np.eye(3)).max():.1e}")

# 2. R_BC_NADIR vs a mount built from "image top toward the nose", right-handed
Rm = I.mount((1, 0, 0))
print(f"[2] R_BC_NADIR == mount(top->nose): {np.allclose(Rm, R_BC_NADIR)}  det = {np.linalg.det(R_BC_NADIR):+.0f}")
c = np.array([[0.1, 0.2, 1.0]])
print(f"    camera ray {c[0]} -> body via '@ R_bc.T' = {(c @ R_BC_NADIR.T)[0]}, via R_bc @ c = {R_BC_NADIR @ c[0]}")
# Level, nose north, h = 10 m. Ground point under the camera; camera then moves 1 m north.
K, D = I.K_FIELD, I.D_FIELD
for yaw_deg in (0, 90):
    pts = []
    for n_off in (0.0, 1.0):
        uv, _ = I.project_ground(np.array([[0.0, 0.0]]), lambda t, n=n_off: np.c_[np.full(len(t), n), np.zeros(len(t))],
                                 lambda t: np.c_[np.zeros(len(t)), np.zeros(len(t)), np.full(len(t), deg(yaw_deg))],
                                 lambda t: np.full(len(t), 10.0), 0.0, 0.0, K, D, R_BC_NADIR, I.H)
        pts.append(uv[0])
    du, dv = pts[1] - pts[0]
    print(f"    level, yaw {yaw_deg:3d}, camera moves 1 m north at 10 m: ground point moves du={du:+.2f} px, dv={dv:+.2f} px")

# 5. load_lens vs a separate scaling, and the pixel-extent property it must keep
Kp, Dp = load_lens(I.LENS_JSON, "Round 1 independent k2", I.W, I.H)
print(f"[5] load_lens K == my field_K: {np.allclose(Kp, K)} ; D equal: {np.allclose(Dp, D)}")
print(f"    K field = fx {Kp[0,0]:.3f}  fy {Kp[1,1]:.3f}  cx {Kp[0,2]:.3f}  cy {Kp[1,2]:.3f}")
prof = I.lens_profile("Round 1 independent k2")
Kc, Dc = I.field_K(prof, 960, 720)
ray = I.unproject_cam(np.array([100.0, 800.0]), np.array([50.0, 600.0]), Kc, Dc)
u_c, v_c = I.project_cam(ray, Kc, Dc)
u_f, v_f = I.project_cam(ray, Kp, Dp)
print(f"    same ray: calib (u+0.5)/960 = {(u_c + 0.5) / 960}, field (u+0.5)/360 = {(u_f + 0.5) / 360}")
print(f"              calib (v+0.5)/720 = {(v_c + 0.5) / 720}, field (v+0.5)/240 = {(v_f + 0.5) / 240}")
# Own forward model vs OpenCV's fisheye forward model, then cv2.fisheye.undistortPoints round trip
th = np.radians(np.linspace(0, 60, 61))
phi = np.radians(np.linspace(0, 350, 36))
TH, PH = np.meshgrid(th, phi)
rays = np.stack([np.sin(TH) * np.cos(PH), np.sin(TH) * np.sin(PH), np.cos(TH)], -1).reshape(-1, 3)
u, v = I.project_cam(rays, Kp, Dp)
cvp, _ = cv2.fisheye.projectPoints(rays.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Kp, Dp)
print(f"    my equidistant model vs cv2.fisheye.projectPoints: max |diff| = {np.abs(cvp.reshape(-1, 2) - np.c_[u, v]).max():.2e} px")
n = cv2.fisheye.undistortPoints(np.c_[u, v].reshape(-1, 1, 2), Kp, Dp).reshape(-1, 2)
back = np.c_[n, np.ones(len(n))]
back /= np.linalg.norm(back, axis=1, keepdims=True)
ang = np.degrees(np.arccos(np.clip(np.sum(back * rays, 1), -1, 1)))
for lim in (45, 55, 60):
    m = np.degrees(TH.ravel()) <= lim
    print(f"    undistortPoints round trip, rays up to {lim} deg off axis: max angle error {ang[m].max() * 3600:.2f} arcsec")
