#!/usr/bin/env python3
"""Q2 (open loop): the competition sequence. The pilot flies to the test position, holds the drone still by hand for
T seconds, then moves the switch. Is there a time without a sent fix just before the switch?

Truth (assumed motion, stated here, not fitted to any result): 4 s of approach (north at 2 m/s, slowing to 0 with a
smooth step), then the hover in Altitude mode (AltHold) with the pilot holding the drone still by hand. Per horizontal
axis: tilt = level-hold noise of the attitude controller (std sig_c, corner 0.5 Hz) + gentle pilot inputs (std sig_p,
0.15 Hz) + the pilot's drift correction -(0.05 x + 0.3 v) / g (assumed: the pilot nulls a drift in 5-10 s); the drone
follows dv/dt = g tilt - v / 5 s (drag, assumed) + air motion (std sig_a, 0.2 Hz). Heading: the FC holds its own
heading (the ATTITUDE yaw stays put, + noise); the true heading wanders by the FC's heading error (std sig_y, 0.05 Hz).
Height: the baro hold wanders (std sig_h, 0.1 Hz). Every noise: white noise through four one-pole low-pass filters.
Levels:            sig_c     sig_p     sig_a        sig_y     sig_h
  still            0.02 deg  0         0.01 m/s^2   0.05 deg  0.05 m   (very good pilot, dead calm)
  calm             0.03 deg  0.1 deg   0.03 m/s^2   0.2 deg   0.15 m   (hand hover, calm air)
  breeze           0.05 deg  0.5 deg   0.15 m/s^2   0.5 deg   0.30 m   (hand hover, light wind)
Camera: px4sim/camera.py, Justice D (digital, 60 fps, 640x360 to Hold) or PAL, with the simulator's own noise and its
camera vibration (vib_deg, default 0.2 deg, which the FC does not see). Hold gets frames at about 12 Hz (as the 80 ms
board model of the closed loop), the FC attitude = true roll and pitch, the FC's held heading, + 0.02 deg noise per
sample, with the true body rates (+ 0.002 rad/s), through
apb_main's GyroAtt as onboard, the height = truth + 0.03 m noise at 10 Hz. Estimator lens: camera.calib_lens(true
lens) (digital) or "Round 1 independent k2" (PAL), as integ/justice_d/open_loop.py. The whole onboard Pipe runs
(min height, good_before_send). Ground: ottawa2022 or farmland (VALIDATION sites). Seeds 11-13 only.
Optional freeze: from --freeze t for --freeze_s s the camera shows one picture: with fresh sensor noise on every frame
("noise", the hard case) or the identical buffer ("repeat").

No autopilot: nothing changes at the switch (the drone keeps hovering), so the result is about the time before the
switch. Gap = longest time between two sent fixes in [T - 10 s, T] (T = hover length; also given for T = 5, 10, 20 s).
Run: ~/.venvs/vio/bin/python integ/gap/hand_hover.py --level calm --height 100 --seed 11 [--camera JD|PAL]
     [--ground ottawa2022] [--hover 20] [--vib 0.2] [--freeze 8 --freeze_s 6 --freeze_kind noise]
     [--repo /path/to/copy]  (import hold.py, apb_main.py, capture.py from a patched copy)  [--tag x]
Writes integ/gap/results/hh/<label>.json.
"""
import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
LEVELS = {"still": (0.02, 0.0, 0.01, 0.05, 0.05), "calm": (0.03, 0.1, 0.03, 0.2, 0.15),
          "breeze": (0.05, 0.5, 0.15, 0.5, 0.30)}       # sig_c deg, sig_p deg, sig_a m/s^2, sig_y deg, sig_h m
G = 9.81


def smooth_noise(rng, n, dt, fc, sigma, poles=4):
    """White noise through poles one-pole low-pass filters (corner fc), scaled to std sigma."""
    if sigma == 0:
        return np.zeros(n)
    a = math.exp(-2 * math.pi * fc * dt)
    x = rng.standard_normal(n + 8000)
    for _ in range(poles):
        y = np.empty_like(x)
        acc = 0.0
        for i, v in enumerate(x):
            acc = a * acc + (1 - a) * v
            y[i] = acc
        x = y
    x = x[8000:]                                   # drop the filter start
    return sigma * (x - x.mean()) / max(x.std(), 1e-12)


def make_motion(level, seed, hover_s, h0, C):
    """Truth arrays on a 5 ms grid from -4 s to hover_s + 1 s: t, N, E, h, q (w, x, y, z), roll, pitch, yaw, and the
    FC's own heading (fc_yaw)."""
    sc, sp, sa, sy, sh = LEVELS[level]
    rng = np.random.default_rng(7000 + seed)
    dt = 0.005
    t = np.arange(-4.0, hover_s + 1.0, dt)
    n = len(t)
    s = np.clip((t + 4.0) / 4.0, 0, 1)
    step = s * s * (3 - 2 * s)                     # smooth 0 -> 1 over the approach
    v_app = 2.0 * (1 - step)                       # approach speed north
    a_app = np.gradient(v_app, dt)
    th = {k: np.radians(smooth_noise(rng, n, dt, 0.5, sc) + smooth_noise(rng, n, dt, 0.15, sp)) for k in "NE"}
    air = {k: smooth_noise(rng, n, dt, 0.2, sa) for k in "NE"}
    pos, vel, tilt = {k: np.zeros(n) for k in "NE"}, {k: np.zeros(n) for k in "NE"}, {k: np.zeros(n) for k in "NE"}
    for k in "NE":
        x, v, x0 = 0.0, (2.0 if k == "N" else 0.0), None
        for i in range(n):
            if t[i] < 0:                           # approach: the tilt that makes the planned slow-down (+ the hover's
                tl = (a_app[i] if k == "N" else 0.0) / G + v_app[i] / 5.0 / G * (k == "N") + th[k][i] * step[i]  # noise)
                v = v_app[i] if k == "N" else 0.0
            else:
                x0 = x if x0 is None else x0          # the pilot holds the place where the hover starts
                tl = th[k][i] - (0.05 * (x - x0) + 0.3 * v) / G
                v += (G * tl - v / 5.0 + air[k][i]) * dt
            x += v * dt
            pos[k][i], vel[k][i], tilt[k][i] = x, v, tl
    yaw0 = math.radians(30.0)
    yaw = yaw0 + np.radians(smooth_noise(rng, n, dt, 0.05, sy))
    h = h0 + smooth_noise(rng, n, dt, 0.1, sh)
    # tilt toward north/east -> body pitch/roll (pitch = -forward tilt, roll = right tilt)
    afw = np.cos(yaw) * tilt["N"] + np.sin(yaw) * tilt["E"]
    art = -np.sin(yaw) * tilt["N"] + np.cos(yaw) * tilt["E"]
    pitch, roll = -np.arctan(afw), np.arctan(art)
    E_N, E_E, E_D = np.eye(3)
    q = np.array([C.qmul(C.q_axis(E_D, y), C.qmul(C.q_axis(E_E, p), C.q_axis(E_N, r)))
                  for r, p, y in zip(roll, pitch, yaw)])
    return dict(t=t, N=pos["N"], E=pos["E"], h=h, q=q, roll=roll, pitch=pitch, yaw=yaw, fc_yaw=np.full(n, yaw0))


def body_rates(m):
    """Body rates (rad/s) from the quaternion derivative: w = 2 conj(q) dq/dt."""
    dq = np.gradient(m["q"], m["t"], axis=0)
    qc = m["q"] * np.array([1, -1, -1, -1.0])
    import camera as C
    w = 2 * C.qmul(qc, dq)
    return w[:, 1:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", default="calm", choices=list(LEVELS))
    ap.add_argument("--height", type=float, default=100.0)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--camera", default="JD", choices=["JD", "PAL"])
    ap.add_argument("--ground", default="ottawa2022")
    ap.add_argument("--hover", type=float, default=20.0)
    ap.add_argument("--vib", type=float, default=None)
    ap.add_argument("--freeze", type=float, default=None)
    ap.add_argument("--freeze_s", type=float, default=6.0)
    ap.add_argument("--freeze_kind", default="noise", choices=["noise", "repeat"])
    ap.add_argument("--repo", default=PROJ)
    ap.add_argument("--hold_cfg", default="{}")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    if a.seed not in (11, 12, 13):
        raise SystemExit("validation seeds 11, 12, 13 only (DATA_SPLITS.md)")
    if a.ground not in ("ottawa2022", "farmland"):
        raise SystemExit("validation grounds ottawa2022, farmland only (DATA_SPLITS.md)")
    repo = os.path.abspath(a.repo)
    sys.path[:0] = [repo, os.path.join(repo, "px4sim")]
    import apb_main
    import camera as C
    import capture
    import hold
    import settings as S
    import sitl
    assert os.path.dirname(os.path.abspath(hold.__file__)) == repo, hold.__file__

    import cv2
    cv2.setNumThreads(2)
    case = dict(video="JUSTICE_D" if a.camera == "JD" else "PAL", ground=a.ground, seed=a.seed)
    c = {**sitl.DEFAULTS, **sitl.video_defaults(case), **case}
    if a.vib is not None:
        c["vib_deg"] = a.vib
    cam = C.Camera(c, C.photo_ground(a.ground))
    m = make_motion(a.level, a.seed, a.hover, a.height, C)
    tg = m["t"]

    def pose(t):
        t = np.atleast_1d(np.asarray(t, float))
        i = np.clip(np.searchsorted(tg, t) - 1, 0, len(tg) - 2)
        f = np.clip((t - tg[i]) / (tg[i + 1] - tg[i]), 0, 1)
        q = m["q"][i] * (1 - f[:, None]) + m["q"][i + 1] * f[:, None]
        lin = lambda k: m[k][i] * (1 - f) + m[k][i + 1] * f   # noqa: E731
        return lin("N"), lin("E"), lin("h"), q / np.linalg.norm(q, axis=1)[:, None]

    # settings as tools/profiles/justice_d.json (digital) or the analog defaults, a perfect calibration, no mask
    s = S.load("/nonexistent")
    tmp = tempfile.mkdtemp(prefix="hh_", dir=os.path.join(HERE, "results"))
    if cam.digital:
        ov = {k: v for k, v in json.load(open(os.path.join(repo, "tools", "profiles", "justice_d.json"))).items()
              if not k.startswith("_") and k not in ("lens_name", "readout_s", "video_delay_s", "video_device")}
        hc = ov.pop("hold_cfg", {})
        s.update(ov)
        sw, sh, fps, k = C.DIGITAL[c["video"]]
        s.update(camera="digital", video_size=[sw, sh], video_fps=fps, video_reduce=k, video_standard="none")
        lm = C.calib_lens(c["true_lens"])
        lens_file = os.path.join(tmp, "lens.json")
        json.dump({"models": [lm]}, open(lens_file, "w"))
        s.update(lens_file=lens_file, lens_name=lm["name"])
    else:
        hc = {}
        s.update(lens_file=C.LENS_JSON, lens_name=c["model_lens"])
    s.update(top=c["cam_top"], mirror=bool(c["cam_mirror"]), tilt_deg=[float(x) for x in c["cam_tilt"]],
             twist_deg=float(c["cam_twist"]), readout_s=c["readout_s"], mask_file="", calibrated=True,
             hold_cfg=dict(hc, **json.loads(a.hold_cfg)))
    s["video_delay_s"] = 0.0          # frame t0 is given exactly below

    # FC feeds: ATTITUDE 100 Hz (truth + 0.02 deg noise, body rates + 0.002 rad/s), through GyroAtt as onboard;
    # height 10 Hz (truth + 0.03 m)
    rng = np.random.default_rng(9000 + a.seed)
    w = body_rates(m)
    att, hgt, f = hold.Series(1e9), hold.Series(1e9), apb_main.att_filter(s)
    for i in range(0, len(tg), 2):
        row = (tg[i], *(np.array([m["roll"][i], m["pitch"][i], m["fc_yaw"][i]]) + rng.normal(0, math.radians(0.02), 3)),
               *(w[i] + rng.normal(0, 0.002, 3)))
        att.add(*(f(row) if f else row))
    for i in range(0, len(tg), 20):
        hgt.add(tg[i], m["h"][i] + rng.normal(0, 0.03))
    fc_pos = lambda t: np.array([np.interp(t, tg, m["N"]), np.interp(t, tg, m["E"])])   # noqa: E731
    pipe = apb_main.Pipe(s, fc_pos)
    judge = capture.judge(s)

    info = {}
    sys.path.insert(0, HERE)
    from replay_run import liveness_numbers
    orig = hold.Hold._track

    def track(self, kf, img, t0, att_, hgt_, tref, deep=False, start=None):
        r = orig(self, kf, img, t0, att_, hgt_, tref, deep, start)
        if isinstance(r, dict) and not deep:
            info["live"] = liveness_numbers(self, kf, r)
            info["proof"], info["dense"] = r.get("live") or "", r.get("dvar") is not None   # the proof Hold gave
        return r
    hold.Hold._track = track

    every = 5 if cam.digital else 2                       # about 12 Hz to Hold (80 ms board model)
    k0, k1 = int(math.ceil(-3.5 * cam.fps)), int(math.floor((a.hover + 0.5) * cam.fps))
    rows, frozen_clean, frozen_img = [], None, None
    for k in range(k0, k1, every):
        t0 = k / cam.fps + cam.exposure / 2
        frz = a.freeze is not None and a.freeze <= t0 < a.freeze + a.freeze_s
        if frz and frozen_clean is not None:
            img = frozen_img if a.freeze_kind == "repeat" else cam.finish(frozen_clean)
        else:
            clean = cam.render(t0, pose)
            img = cam.finish(clean)
            if frz:
                frozen_clean, frozen_img = clean, img
        fr = capture.Frame(img, t0, t0, k, judge(img))
        info.clear()
        e, ok, why = pipe.step(fr, att, hgt)
        err = None
        if e is not None and e.var is not None:
            err = float(np.linalg.norm(e.pos - fc_pos(e.t)))
        lv = info.get("live") or {}
        rows.append(dict(t=round(t0, 4), sent=int(ok), why=why or "", state=e.state if e else "", err=err, frozen=int(frz),
                         mm=lv.get("mm"), mp=lv.get("mp"), md=lv.get("md"), p90dd=lv.get("p90dd"), follow=lv.get("follow"),
                         verdict=lv.get("verdict"), proof=info.get("proof"), dense=info.get("dense"),
                         mv_mot=lv.get("mv_mot"), mv_res=lv.get("mv_res")))
    ts = np.array([r["t"] for r in rows if r["sent"]])

    def gap(lo, hi):
        x = np.r_[lo, ts[(ts > lo) & (ts < hi)], hi]
        return float(np.diff(x).max())
    hov = [r for r in rows if 0 <= r["t"] < a.hover]
    res = dict(label=None, level=a.level, motion=dict(zip(("sig_c_deg", "sig_p_deg", "sig_a", "sig_y_deg", "sig_h_m"),
                                                          LEVELS[a.level])),
               height=a.height, seed=a.seed, camera=a.camera, ground=a.ground, hover_s=a.hover, vib_deg=c["vib_deg"],
               repo=repo, hold_py=hold.__file__, move_px=hold.Hold.MOVE_PX, hold_cfg=s["hold_cfg"],
               freeze=[a.freeze, a.freeze_s, a.freeze_kind] if a.freeze is not None else None,
               sent_pct_hover=round(100.0 * sum(r["sent"] for r in hov) / max(len(hov), 1), 1),
               gap_before_switch_s={str(T): round(gap(max(0.0, T - 10), T), 2) for T in (5, 10, 20) if T <= a.hover},
               gap_max_hover_s=round(gap(0.0, a.hover), 2),
               why_hover={}, rows=rows)
    for r in hov:
        if not r["sent"]:
            kk = r["why"].split(" (")[0]
            res["why_hover"][kk] = res["why_hover"].get(kk, 0) + 1
    errs = [r["err"] for r in hov if r["sent"] and not r["frozen"]]
    res["fix_err_rms_m"] = round(float(np.sqrt(np.mean(np.square(errs)))), 3) if errs else None
    if a.freeze is not None:
        fz = [r for r in rows if r["frozen"]]
        late = [r for r in fz if r["t"] >= a.freeze + 0.5]
        res["frozen"] = dict(frames=len(fz), sent=sum(r["sent"] for r in fz), sent_after_0_5s=sum(r["sent"] for r in late),
                             frames_after_0_5s=len(late),
                             last_sent_after_freeze_s=round(max([r["t"] for r in fz if r["sent"]], default=a.freeze) - a.freeze, 2),
                             max_err_sent_m=round(max([r["err"] for r in fz if r["sent"]], default=0.0), 3),
                             mm_on_frozen=[round(r["mm"], 3) for r in fz if r["mm"] is not None],
                             mp_on_frozen=[round(r["mp"], 3) for r in fz if r.get("mp") is not None])
    label = (f"hh_{a.camera}_{a.level}_{a.height:g}m_{a.ground}_s{a.seed}"
             + (f"_vib{c['vib_deg']:g}" if a.vib is not None else "")
             + (f"_frz{a.freeze_kind}" if a.freeze is not None else "") + a.tag)
    res["label"] = label
    out = os.path.join(HERE, "results", "hh")
    os.makedirs(out, exist_ok=True)
    json.dump(res, open(os.path.join(out, label + ".json"), "w"))
    os.rmdir(tmp) if not os.listdir(tmp) else [os.remove(os.path.join(tmp, x)) for x in os.listdir(tmp)] and os.rmdir(tmp)
    print(f"{label}: MOVE_PX {res['move_px']}, sent {res['sent_pct_hover']}% of the hover, gap before switch "
          f"{res['gap_before_switch_s']}, longest hover gap {res['gap_max_hover_s']} s, fix err RMS {res['fix_err_rms_m']} m, "
          f"why {res['why_hover']}" + (f", frozen {res['frozen'] | {'mm_on_frozen': None, 'mp_on_frozen': None}}" if a.freeze is not None else ""))


if __name__ == "__main__":
    main()
