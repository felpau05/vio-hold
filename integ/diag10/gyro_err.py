#!/usr/bin/env python3
"""Attitude error of the attitude Hold gets (FC ATTITUDE as sent, or apb_main.GyroAtt of it) against the true attitude,
over a recorded flight. Prints the tilt error (deg) of each source per 5 s and its RMS over the hold.
Run: ~/.venvs/vio/bin/python integ/diag10/gyro_err.py <label> [every]   (every: use every n-th ATTITUDE row, default 2)"""
import math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(PROJ, "px4sim")]
sys.dont_write_bytecode = True
import camera as C
import replay as RP
hm = RP.load_hold(os.path.join(HERE, "hold_base.py"), "hb")
lab = sys.argv[1]; every = int(sys.argv[2]) if len(sys.argv) > 2 else 2
d = np.load(os.path.join(PROJ, "apsim", "runs", lab + ".npz")); tr, att, te = d["truth"], d["att"], float(d["t_engage"])
ty = C.q_yaw(tr[:, 4:])
dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
rows = att[(att[:, 0] > te - 30) & (att[:, 0] < te + 62)][::every]
g = RP.GyroAtt(20.0, hm.euler_R)
ga = np.array([g(tuple(r)) for r in rows])
t = rows[:, 0]
# true body->NED at these times (yaw + FC yaw offset)
i = np.clip(np.searchsorted(tr[:, 0], t), 1, len(tr) - 1)
q = tr[i, 4:]
rT, pT = C.q_roll_pitch(q); yT = C.q_yaw(q) + dy
RT = hm.euler_R(rT, pT, yT)
def err(a):
    R = hm.euler_R(a[:, 1], a[:, 2], a[:, 3])
    E = np.einsum("nji,njk->nik", RT, R)          # RT^T R
    v = np.array([np.array([E[k, 2, 1] - E[k, 1, 2], E[k, 0, 2] - E[k, 2, 0], E[k, 1, 0] - E[k, 0, 1]]) / 2 for k in range(len(E))])
    return np.degrees(v)                          # small-angle body-frame error (roll, pitch, yaw) deg
eF, eG = err(rows), err(ga)
m = (t >= te) & (t <= te + 60)
print(f"{lab}: tilt error RMS over the hold (deg): FC roll {np.sqrt(np.mean(eF[m,0]**2)):.2f} pitch {np.sqrt(np.mean(eF[m,1]**2)):.2f} yaw {np.sqrt(np.mean(eF[m,2]**2)):.2f}"
      f" | gyro roll {np.sqrt(np.mean(eG[m,0]**2)):.2f} pitch {np.sqrt(np.mean(eG[m,1]**2)):.2f} yaw {np.sqrt(np.mean(eG[m,2]**2)):.2f}")
for tt in np.arange(-10, 60, 5):
    k = np.searchsorted(t, te + tt)
    print(f"  {tt:5.0f}  FC {eF[k,0]:+.2f} {eF[k,1]:+.2f} {eF[k,2]:+.2f}   gyro {eG[k,0]:+.2f} {eG[k,1]:+.2f} {eG[k,2]:+.2f}   true roll {math.degrees(rT[k]):+.1f} pitch {math.degrees(pT[k]):+.1f}")
