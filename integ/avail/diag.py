#!/usr/bin/env python3
"""Open-loop check of the onboard pipeline (apb_main.Pipe + Hold + the tilt filter) on a logged flight.

Re-renders the camera (px4sim/camera.py, independent of hold.py) from a closed-loop run's true pose over
any ground, feeds every frame through apb_main.Pipe with the run's own FC attitude and height, and
counts fixes, reasons and errors against the truth (in PX4's heading frame). No PX4, no slow CPU:
it finds the gates that stop fixes, it does not replace the closed loop.

Run: ~/.venvs/vio/bin/python integ/avail/diag.py RUN.npz GROUND T_FROM T_TO [key=json ...]
  keys: settings (onboard overrides, e.g. hold_cfg), case (camera case keys), still=1 (hold the pose
  fixed at T_FROM: a perfectly still hover), freeze=[t,dur] (frozen picture with fresh noise), out=path.json
"""
import json
import math
import os
import time
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "px4sim"))
import camera as C  # noqa: E402
import sitl  # noqa: E402
if os.environ.get("HOLD_DIR"):      # A/B: a folder with another hold.py / apb_main.py / capture.py
    sys.path.insert(0, os.environ["HOLD_DIR"])


def main(path, ground, t_from, t_to, settings=None, case=None, still=False, freeze=None, out=None):
    import apb_main
    import capture
    import hold
    d = np.load(path)
    tr, att, ekf = d["truth"], d["att"], d["ekf"]
    c = dict(sitl.DEFAULTS, ground=ground, **(case or {}))
    cam = C.Camera(c, C.make_ground(ground))
    t_hold = t_from

    def pose(t):
        t = np.full_like(np.asarray(t, float), t_hold) if still else np.asarray(t, float)
        i = np.clip(np.searchsorted(tr[:, 0], t) - 1, 0, len(tr) - 2)
        f = np.clip((t - tr[i, 0]) / (tr[i + 1, 0] - tr[i, 0]), 0, 1)[:, None]
        v = tr[i, 1:] * (1 - f) + tr[i + 1, 1:] * f
        return v[:, 0], v[:, 1], v[:, 2], v[:, 3:] / np.linalg.norm(v[:, 3:], axis=1)[:, None]

    ty = C.q_yaw(tr[:, 4:])
    dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
    Rz = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])
    s_path, s = sitl.sim_settings(c, "diag_avail")
    s.update(settings or {})
    rows = att
    if still:          # a still hover: the FC sees the attitude of T_FROM, rates zero
        k = int(np.searchsorted(att[:, 0], t_from))
        rows = att.copy()
        rows[:, 1:4], rows[:, 4:7] = att[k, 1:4], 0.0
    f = apb_main.att_filter(s)
    A, H = hold.Series(1e9), hold.Series(1e9)
    for r in rows:
        A.add(*(f(tuple(r)) if f else r))
    z0 = ekf[0, 3]
    for r in ekf:
        H.add(r[0], z0 - r[3])

    def fc_pos(t):
        N, E, _, _ = pose(np.array([t]))
        return Rz @ np.array([N[0], E[0]])
    pipe = apb_main.Pipe(s, fc_pos)
    judge = capture._Judge(s)
    rng = np.random.default_rng(5)
    held, res, cpu = None, [], []
    for k in range(int(math.ceil(t_from * cam.fps)), int(t_to * cam.fps)):
        t0 = k / cam.fps + cam.exposure / 2
        frozen = freeze is not None and freeze[0] <= t0 - t_from < freeze[0] + freeze[1]
        if frozen:
            held = held if held is not None else img_clean
            img = np.clip(held + rng.normal(0, 3, held.shape), 0, 255).astype(np.uint8)
        else:
            img_clean = cam.render(t0, pose)
            img = cam.finish(img_clean)
            img_clean = img.astype(np.float32)
        fr = capture.Frame(img, t0, t0, k, judge(img))
        t_cpu = time.perf_counter()
        e, ok, why = pipe.step(fr, A, H)
        cpu.append(time.perf_counter() - t_cpu)
        P = fc_pos(t0 + s["readout_s"] / 2)
        err = None if e is None else (e.pos - P)
        res.append(dict(t=round(t0 - t_from, 3), ok=bool(ok), why=(why or "").split(" (")[0],
                        state=e.state if e else "", n=int(e.n) if e else 0, frozen=frozen,
                        ex=None if err is None else float(err[0]), ey=None if err is None else float(err[1]),
                        var=None if e is None or e.var is None else float(e.var)))
    sent = [r for r in res if r["ok"]]
    whys = {}
    for r in res:
        if not r["ok"]:
            whys[r["why"]] = whys.get(r["why"], 0) + 1
    e = np.array([[r["ex"], r["ey"]] for r in sent]) if sent else np.zeros((0, 2))
    # error after removing the error at the first sent fix (the anchor's offset from the FC position)
    e0 = e - e[:1] if len(e) else e
    m2 = [((r["ex"] - e[0, 0]) ** 2 + (r["ey"] - e[0, 1]) ** 2) / (r["var"] / 2) for r in sent] if sent else []
    first = sent[0]["t"] if sent else None
    summ = dict(run=os.path.basename(path), ground=ground, t=[t_from, t_to], still=still, freeze=freeze,
                frames=len(res), sent=len(sent), sent_pct=round(100 * len(sent) / max(len(res), 1), 1),
                first_sent_s=first, sent_while_frozen=sum(1 for r in sent if r["frozen"]),
                err_rms=float(np.sqrt(np.mean(np.sum(e0 ** 2, 1)))) if len(e0) else None,
                err_max=float(np.max(np.hypot(*e0.T))) if len(e0) else None,
                outside_99=float(np.mean(np.array(m2) > 9.21)) if m2 else None,
                n_median=float(np.median([r["n"] for r in sent])) if sent else None,
                step_ms_mean=round(1e3 * float(np.mean(cpu)), 2),
                whys=dict(sorted(whys.items(), key=lambda x: -x[1])[:8]))
    print(json.dumps(summ))
    if out:
        json.dump(dict(summary=summ, rows=res), open(out, "w"))
    return summ


if __name__ == "__main__":
    a = sys.argv[1:]
    kv = {k: json.loads(v) for k, v in (x.split("=", 1) for x in a[4:])}
    main(a[0], a[1], float(a[2]), float(a[3]), **kv)
