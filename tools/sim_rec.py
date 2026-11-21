#!/usr/bin/env python3
"""A raw recording, as "apb_main.py run --raw" saves it, made from a closed-loop simulator run. For rehearsing
the fly-record-replay-tune loop on the laptop, and for testing replay_tune.py against the truth.

  sim_rec.py px4sim/runs/apb_100m_w20g40.npz OUT_DIR [--ground ottawa2019fall] [--seconds 60] [--start T] [--video V]

The picture comes from px4sim/camera.py (its own lens, rolling shutter, noise and vibration; independent of
hold.py) at the run's true pose. att = the run's FC ATTITUDE rows, hgt = the FC height, lpos = the FC position,
exactly the columns Raw saves. OUT_DIR gets raw_sim_NNN.npz (10 s chunks), settings.json with the true
camera values (a perfect calibration), and in every chunk an extra key truth_ne (t, north, east at mid-frame,
in the FC frame, one row per frame) that the program ignores. Start: the moment the run entered Position mode.
Chunks share 1 s of attitude rows, as Raw's do (replay_tune.py explains why that matters).
--video JUSTICE_D: the digital camera of px4sim/sitl.py (VIDEO_DEFAULTS: 1280x720 at 60 fps, given to Hold as 640x360).
settings.json is then what "use_profile.py apply justice_d" writes, with lens_measured = the lens a hand-turn calib
can give (camera.calib_lens, as sitl.model_lens), and Raw's record_hz (30) picks the frames. Default: PAL (analog).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "px4sim"))
import camera as C                  # noqa: E402
import settings as S                # noqa: E402
from sitl import DEFAULTS as SIM, VIDEO_DEFAULTS, JUSTICE_D_OVERLAY    # noqa: E402  (camera values only; nothing is started)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run")
    p.add_argument("out")
    p.add_argument("--ground", default="ottawa2019fall")
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--start", type=float, help="run time to start at (default: Position mode entry)")
    p.add_argument("--video", default=SIM["video"], choices=[*C.VIDEO, *C.DIGITAL], help="camera (default PAL)")
    a = p.parse_args()
    d = np.load(a.run)
    tr, att, ekf = d["truth"], d["att"], d["ekf"]
    c = dict(SIM, **VIDEO_DEFAULTS.get(a.video, {}), video=a.video, ground=a.ground)
    cam = C.Camera(c, C.make_ground(a.ground))
    ro, ex = c["readout_s"], c["exposure_s"]
    delay = abs(ro) + ex + c["video_delay_s"] - (ex / 2 + (abs(ro) if ro < 0 else 0.0))   # as px4sim sim_settings

    def pose(t):
        t = np.asarray(t, float)
        i = np.clip(np.searchsorted(tr[:, 0], t) - 1, 0, len(tr) - 2)
        f = np.clip((t - tr[i, 0]) / (tr[i + 1, 0] - tr[i, 0]), 0, 1)[:, None]
        v = tr[i, 1:] * (1 - f) + tr[i + 1, 1:] * f
        return v[:, 0], v[:, 1], v[:, 2], v[:, 3:] / np.linalg.norm(v[:, 3:], axis=1)[:, None]

    # The FC's north axis differs from the truth's by its own heading error (as integ/avail/diag.py).
    dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - C.q_yaw(tr[:, 4:]))))))
    Rz = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])
    t_from = float(d["t_engage"]) if a.start is None else a.start
    hgt = np.c_[ekf[:, 0], ekf[0, 3] - ekf[:, 3]]
    lpos = ekf[:, 0:3]
    os.makedirs(a.out, exist_ok=True)
    s = S.load("/nonexistent")
    if cam.digital:     # the justice_d overlay, then the lens a hand-turn calib writes (as px4sim/sitl.py model_lens)
        s.update({k: v for k, v in json.load(open(JUSTICE_D_OVERLAY)).items() if not k.startswith("_")})
        m = C.calib_lens(c["true_lens"])
        s["lens_measured"] = dict(by="hand turn", image_size=m["image_size"], note=m["note"], **{k: m[k] for k in S.LENS_KEYS})
    s.update(top=c["cam_top"], mirror=bool(c["cam_mirror"]), tilt_deg=[float(x) for x in c["cam_tilt"]],
             twist_deg=float(c["cam_twist"]), lens_name=c["model_lens"] or s["lens_name"], readout_s=ro, video_delay_s=delay,
             calibrated=True)
    S.save(s, os.path.join(a.out, "settings.json"))
    import apb_main
    K, D = apb_main.lens(s, (cam.H, cam.W), focal=False)
    keep = apb_main.RateGate(s["record_hz"])     # as Raw: at most record_hz frames each second (0 = each frame)
    ks = [k for k in range(int(math.ceil(t_from * cam.fps)), int((t_from + a.seconds) * cam.fps))
          if keep(k / cam.fps + ex / 2 + (abs(ro) if ro < 0 else 0.0) + delay)]
    for j, chunk in enumerate(np.array_split(np.array(ks), max(1, round(a.seconds / 10)))):
        t0 = chunk / cam.fps + ex / 2 + (abs(ro) if ro < 0 else 0.0)
        imgs = np.array([cam.finish(cam.render(t, pose)) for t in t0])
        N, E, _, _ = pose(t0 + ro / 2)                                       # true position at mid-frame
        cut = lambda x: x[(x[:, 0] >= t0[0] - 1) & (x[:, 0] <= t0[-1] + 1)]     # noqa: E731
        np.savez(os.path.join(a.out, f"raw_sim_{j:03d}.npz"), imgs=imgs, t_grab=t0 + delay, att=cut(att),
                 hgt=cut(hgt), lpos=cut(lpos), K=K, D=D, truth_ne=np.c_[t0, (Rz @ np.c_[N, E].T).T])
        print(f"raw_sim_{j:03d}.npz: {len(imgs)} frames, {t0[0] - t_from:.1f}-{t0[-1] - t_from:.1f} s", flush=True)
    print(f"settings.json: video_delay_s {delay:.4f}, readout_s {ro}, true focal scale {c['focal_scale']}")


if __name__ == "__main__":
    main()
