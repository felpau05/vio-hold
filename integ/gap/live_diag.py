#!/usr/bin/env python3
"""Diagnostic twin of one tests_apb/test_live.py case (same scene changes as its worker), keeping the full
hand_hover.py result: per processed frame the proof Hold gave (move / still / moved), the whole-picture mode, and
whether the fix was sent. For finding which proof gives the fixes; not a test.
Run: ~/.venvs/vio/bin/python integ/gap/live_diag.py <hover|freeze_noise|freeze_repeat|overlay> <height> <seed>
     [--repo path] [--tag x]       -> integ/gap/results/live/<kind>_<height>m_s<seed><tag>.json
"""
import argparse
import glob
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind")
    ap.add_argument("height", type=float)
    ap.add_argument("seed", type=int)
    ap.add_argument("--repo", default=PROJ)
    ap.add_argument("--tag", default="")
    ap.add_argument("--t_freeze", type=float, default=10.0, help="freeze and roll start (test_live.py: 10 s)")
    a = ap.parse_args()
    repo = os.path.abspath(a.repo)
    sys.path[:0] = [repo, os.path.join(repo, "px4sim"), HERE]
    import numpy as np
    import cv2
    import camera as C
    import hand_hover as HH
    kind, height, seed = a.kind, a.height, a.seed
    tmp = tempfile.mkdtemp(prefix="live_", dir=os.path.join(HERE, "results"))
    os.makedirs(os.path.join(tmp, "results", "hh"))
    HH.HERE = tmp
    args = ["hand_hover", "--level", "calm", "--height", str(height), "--seed", str(seed), "--vib", "0.2",
            "--hover", "20", "--repo", repo]
    if kind in ("freeze_noise", "freeze_repeat"):
        args += ["--freeze", str(a.t_freeze), "--freeze_s", "6", "--freeze_kind", kind.split("_")[1]]
    if kind != "hover":                    # as test_live.py: roll 2 deg in 1.5 s from 10 s; overlay: 0.4 deg/s
        t_ramp = 3.0 if kind == "overlay" else a.t_freeze
        make = HH.make_motion

        def make_motion(level, sd, hover_s, h0, Cm):
            m = make(level, sd, hover_s, h0, Cm)
            s = np.clip((m["t"] - t_ramp) / 1.5, 0, 1)
            extra = np.radians(2.0) * s * s * (3 - 2 * s)
            if kind == "overlay":
                extra = np.radians(0.4) * (m["t"] + 4.0)
            m["roll"] = m["roll"] + extra
            E_N, E_E, E_D = np.eye(3)
            m["q"] = np.array([Cm.qmul(Cm.q_axis(E_D, y), Cm.qmul(Cm.q_axis(E_E, p), Cm.q_axis(E_N, r)))
                               for r, p, y in zip(m["roll"], m["pitch"], m["yaw"])])
            return m
        HH.make_motion = make_motion
    if kind == "overlay":
        def render(self, t0, pose):
            img = np.full((self.SH, self.SW), 128, np.uint8)
            for i, line in enumerate(("REC 00:12:34", "BAT 15.2 V  GPS 14", "ALT 40 M  SPD 0.0")):
                cv2.putText(img, line, (60, 100 + 90 * i), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 255, 4)
            return img.astype(np.float32)
        C.Camera.render = render
    if height < 15:
        import settings as S
        load = S.load
        S.load = lambda *x, **k: dict(load(*x, **k), min_height_m=0.0)
    sys.argv = args
    HH.main()
    src = glob.glob(os.path.join(tmp, "results", "hh", "*.json"))[0]
    out = os.path.join(HERE, "results", "live")
    os.makedirs(out, exist_ok=True)
    dst = os.path.join(out, f"{kind}_{height:g}m_s{seed}{a.tag}.json")
    shutil.copy(src, dst)
    shutil.rmtree(tmp)
    r = json.load(open(dst))
    rows = r["rows"]
    last, by = "", {}
    for x in rows:                         # attribute each sent fix to the last proof that renewed the live time
        if x.get("proof"):
            last = x["proof"] + ("/dense" if x.get("dense") else "")
        if x["sent"]:
            by[last] = by.get(last, 0) + 1
    print(f"{os.path.basename(dst)}: sent {sum(x['sent'] for x in rows)} of {len(rows)}; gap20 "
          f"{r['gap_before_switch_s'].get('20')}; frozen {r.get('frozen', {}) and {k: r['frozen'][k] for k in ('sent', 'frames', 'last_sent_after_freeze_s')}}; "
          f"sent fixes by last proof {by}")


if __name__ == "__main__":
    main()
