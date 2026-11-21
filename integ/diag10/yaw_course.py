#!/usr/bin/env python3
"""Yaw error of the GyroAtt feed (apb_main.GyroAtt, tau 20 s, started 60 s before the switch) and of the FC yaw against
the true yaw (plus the FC's mean yaw offset), at the switch and 10 / 30 / 60 s after, for SITL runs; with the change over
the hold and the closed-loop sent-fix error (ferr60, cl_budget.py). No pictures. Run: yaw_course.py label ..."""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(PROJ, "px4sim")]
sys.dont_write_bytecode = True
import camera as C
import replay as RP
hm = RP.load_hold(os.path.join(HERE, "hold_base.py"), "hb")
cl = {}
p = os.path.join(HERE, "cl_budget.jsonl")
if os.path.exists(p):
    for l in open(p):
        o = json.loads(l); cl[o["label"]] = o
w = lambda a: math.degrees((a + math.pi) % (2 * math.pi) - math.pi)
for lab in sys.argv[1:]:
    d = np.load(os.path.join(PROJ, "apsim", "runs", lab + ".npz")); tr, att, te = d["truth"], d["att"], float(d["t_engage"])
    js = json.load(open(os.path.join(PROJ, "apsim", "runs", lab + ".json")))
    ty = C.q_yaw(tr[:, 4:])
    m = (tr[:, 0] >= te) & (tr[:, 0] <= te + 60)
    dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[m, 0], att[:, 0], np.unwrap(att[:, 3])) - ty[m])))))   # hold-mean offset
    rows = att[(att[:, 0] > te - 60) & (att[:, 0] < te + 61)]
    rows = rows[::2] if np.median(np.diff(rows[:, 0])) < 0.0075 else rows
    g = RP.GyroAtt(20.0, hm.euler_R); ga = np.array([g(tuple(r)) for r in rows])
    t = rows[:, 0]
    yT = np.interp(t, tr[:, 0], np.unwrap(ty)) + dy
    eg = np.array([w(x) for x in ga[:, 3] - yT]); ef = np.array([w(x) for x in rows[:, 3] - yT])
    at = lambda e, s: float(e[min(np.searchsorted(t, te + s), len(t) - 1)])
    al = [x for x in js.get("ap_text", []) if "yaw alignment" in str(x)]
    o = cl.get(lab, {})
    print(f"{lab:28s} h {js['cfg']['height']:4.0f}  gyro yaw err 0/10/30/60 s {at(eg,0):+5.1f} {at(eg,10):+5.1f} {at(eg,30):+5.1f} {at(eg,59):+5.1f}"
          f"  FC {at(ef,0):+5.1f} {at(ef,10):+5.1f} {at(ef,30):+5.1f} {at(ef,59):+5.1f}  climb {js.get('climb_s')} s"
          f"  ferr60 {o.get('ferr60')} dense {o.get('share_dense_fixes_hold')}")
