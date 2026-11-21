#!/usr/bin/env python3
"""Liveness test: Hold sends fixes in a hover by hand, and sends none on a frozen or overlay-only picture.

Uses the synthetic hover of integ/gap/hand_hover.py (Justice D camera, ottawa2022 ground, level calm, 0.2 deg
camera vibration). Each case runs in its own child process (this file with --worker).
Cases:
  hover   20 s at 10, 40 and 100 m, seeds 11 and 12 (at 10 m the test sets min_height_m to 0). Pass: the longest time with no fix in the last 10 s is under 0.3 s.
  freeze  hover at 40 m, picture frozen from 10 s (identical picture, or fresh noise), attitude moves 2 deg from
          10 s. Pass: the last fix is at most 1.0 s after the freeze starts.
  overlay a grey ground with a static text overlay, roll turns 0.4 deg/s. Pass: no fix.
Run: ~/.venvs/vio/bin/python tests_apb/test_live.py      (about 15 min)
"""
import glob
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
GAP_MAX_S, FREEZE_MAX_S = 0.3, 1.0
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + text, flush=True)


def worker(kind, height, seed, out):
    """Run one hand_hover case. Writes a small JSON file to out."""
    sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), os.path.join(PROJ, "integ", "gap")]
    import numpy as np
    import cv2
    import camera as C
    import hand_hover as HH
    import replay_run  # noqa: F401  (hand_hover imports it from its own folder)
    tmp = tempfile.mkdtemp(prefix="live_")
    os.makedirs(os.path.join(tmp, "results", "hh"))
    HH.HERE = tmp                                   # results go to a temporary folder
    args = ["hand_hover", "--level", "calm", "--height", str(height), "--seed", str(seed), "--vib", "0.2",
            "--hover", "20", "--repo", PROJ]
    if kind in ("freeze_noise", "freeze_repeat"):
        args += ["--freeze", "10", "--freeze_s", "6", "--freeze_kind", kind.split("_")[1]]

    if kind != "hover":                              # the attitude moves 2 deg (roll) in 1.5 s, from 10 s (overlay: roll turns 0.4 deg/s)
        t_ramp = 3.0 if kind == "overlay" else 10.0
        make = HH.make_motion

        def make_motion(level, sd, hover_s, h0, Cm):
            m = make(level, sd, hover_s, h0, Cm)
            s = np.clip((m["t"] - t_ramp) / 1.5, 0, 1)
            extra = np.radians(2.0) * s * s * (3 - 2 * s)
            if kind == "overlay":                    # roll turns 0.4 deg/s all the time: no still moment
                extra = np.radians(0.4) * (m["t"] + 4.0)
            m["roll"] = m["roll"] + extra
            E_N, E_E, E_D = np.eye(3)
            m["q"] = np.array([Cm.qmul(Cm.q_axis(E_D, y), Cm.qmul(Cm.q_axis(E_E, p), Cm.q_axis(E_N, r)))
                               for r, p, y in zip(m["roll"], m["pitch"], m["yaw"])])
            return m
        HH.make_motion = make_motion
    if kind == "overlay":                            # flat grey ground, static text, no ground texture
        def render(self, t0, pose):
            img = np.full((self.SH, self.SW), 128, np.uint8)
            for i, line in enumerate(("REC 00:12:34", "BAT 15.2 V  GPS 14", "ALT 40 M  SPD 0.0")):
                cv2.putText(img, line, (60, 100 + 90 * i), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 255, 4)
            return img.astype(np.float32)
        C.Camera.render = render
    if height < 15:                                  # the run program sends no fix under min_height_m (15 m): lower it for the test
        import settings as S
        load = S.load
        S.load = lambda *a, **k: dict(load(*a, **k), min_height_m=0.0)
    sys.argv = args
    HH.main()
    r = json.load(open(glob.glob(os.path.join(tmp, "results", "hh", "*.json"))[0]))
    small = dict(gap20=r["gap_before_switch_s"].get("20"), frozen=r.get("frozen"),
                 sent=sum(x["sent"] for x in r["rows"]), frames=len(r["rows"]), sent_t=[x["t"] for x in r["rows"] if x["sent"]], hold_py=r["hold_py"])
    json.dump(small, open(out, "w"))


def run_case(kind, height, seed):
    out = tempfile.mktemp(suffix=".json", prefix="live_out_")
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", kind, str(height), str(seed), out],
                       capture_output=True, text=True, timeout=3000)
    if p.returncode or not os.path.exists(out):
        return None, p.stderr[-600:]
    return json.load(open(out)), ""


def main():
    t0 = time.time()
    for h in (10, 40, 100):
        for seed in (11, 12):
            r, err = run_case("hover", h, seed)
            name = f"hover {h} m seed {seed}"
            if r is None:
                check(False, f"{name}: the run stopped. {err}")
                continue
            check(r["gap20"] < GAP_MAX_S, f"{name}: longest time with no fix in the last 10 s is {r['gap20']:.2f} s "
                  f"(limit {GAP_MAX_S} s)")
    for kind in ("freeze_repeat", "freeze_noise"):
        r, err = run_case(kind, 40, 11)
        name = f"{kind.replace('_', ' ')} 40 m"
        if r is None:
            check(False, f"{name}: the run stopped. {err}")
            continue
        last = r["frozen"]["last_sent_after_freeze_s"]
        check(last <= FREEZE_MAX_S, f"{name}: the last fix is {last:.2f} s after the freeze starts "
              f"(limit {FREEZE_MAX_S} s). Fixes on the frozen picture: {r['frozen']['sent']} of {r['frozen']['frames']}")
    r, err = run_case("overlay", 40, 11)
    if r is None:
        check(False, f"overlay: the run stopped. {err}")
    else:
        check(r["sent"] == 0, f"overlay only: {r['sent']} fixes sent in {r['frames']} frames (limit 0). Times of the fixes: {r['sent_t']}")
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--worker":
        worker(sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5])
    else:
        sys.exit(main())
