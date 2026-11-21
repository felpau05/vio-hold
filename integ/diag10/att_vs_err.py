#!/usr/bin/env python3
"""Relate the fix error of a replay to the attitude error of the feed it got: the error of each sent fix (N, E) against
the attitude error of that feed at the fix time (true attitude minus feed attitude, as the ground offset h x delta of a
nadir ray, N and E, m), per keyframe segment. Prints the regression gain (fix error per m of h x delta).
Run: ~/.venvs/vio/bin/python integ/diag10/att_vs_err.py <result.json.gz>"""
import gzip, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(PROJ, "px4sim")]
sys.dont_write_bytecode = True
import camera as C
import replay as RP
hm = RP.load_hold(os.path.join(HERE, "hold_base.py"), "hb")
r = json.load(gzip.open(sys.argv[1]))
d = np.load(os.path.join(PROJ, "apsim", "runs", r["flight"] + ".npz")); tr, att, te = d["truth"], d["att"], float(d["t_engage"])
ty = C.q_yaw(tr[:, 4:])
dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
rows = att[(att[:, 0] > te + r["start"] - 12) & (att[:, 0] < te + r["start"] + r["seconds"] + 2)][::2]
if r["att"] == "gyro":
    g = RP.GyroAtt(20.0, hm.euler_R)
    rows = np.array([g(tuple(x)) for x in rows])
S = hm.Series(1e9)
for x in rows:
    S.add(*x)
sent = [x for x in r["rows"] if x["sent"]]
t = np.array([x["t"] for x in sent]) + te + 0.007
e = np.array([x["err"] for x in sent])
Rf = hm.attitude_R(S, t)
i = np.clip(np.searchsorted(tr[:, 0], t), 1, len(tr) - 1)
q = tr[i, 4:]
rT, pT = C.q_roll_pitch(q)
RT = hm.euler_R(rT, pT, C.q_yaw(q) + dy)
h = np.interp(t, tr[:, 0], tr[:, 3])
# where the camera axis (body down) hits the ground: with the feed and with the truth (FC frame), relative to the drone
def hit(R):
    b = R[:, :, 2]
    return h[:, None] * b[:, :2] / b[:, 2:3]
dh = hit(Rf) - hit(RT)          # the feed's ground offset error of the optical axis (m)
# also the yaw-only part: the tilt offset rotated
print(os.path.basename(sys.argv[1]))
for ax in (0, 1):
    A = np.c_[dh[:, ax], np.ones(len(t))]
    k = np.linalg.lstsq(A, e[:, ax], rcond=None)[0]
    print(f"  axis {'NE'[ax]}: fix err = {k[0]:+.2f} x (h delta) {k[1]:+.3f};  std h delta {dh[:, ax].std():.3f} m, std err {e[:, ax].std():.3f} m, corr {np.corrcoef(dh[:, ax], e[:, ax])[0,1]:+.2f}")
for tt in range(0, 58, 4):
    j = np.searchsorted(t - te, tt)
    if j < len(t):
        print(f"   t {tt:3d}  err {e[j,0]:+.2f} {e[j,1]:+.2f}   feed axis-hit error {dh[j,0]:+.3f} {dh[j,1]:+.3f}")
