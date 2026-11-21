#!/usr/bin/env python3
"""Calibration in the loop for the Orqa Justice D (digital): the Justice D variant of integ/calib_loop.py.

A hand-turn recording is made through the onboard program's own code: fc_link reads a fake PX4 (tests_apb/fake_fc.py,
scenario "hand": 80 ms clock offset, 0-20 ms jitter, camera 60 m from the scene), the capture source gives pictures
that px4sim/camera.py renders in real time (1280x720 sensor at 60 fps, rolling shutter per sensor row, white noise,
INTER_AREA to 640x360), with the digital picture checks (capture.judge), and apb_main.record stamps and stores them.
Then apb_main calib (mode_calib --rec, calib.solve fit_lens) must find the lens, the mount and the timing.

Start (what the settings hold before calib): tools/profiles/justice_d.json - lens "Justice D full sensor (assumed)"
(fx = fy = 482.5 at 1280x720, k1 0), readout 16 ms, video delay 40 ms, mount picture top = front, no tilt, no twist.
Truth (camera.py only): lens "sim Justice D near crop" (fx 634.9, fy 645.1, k1 -0.018, k2 -0.002, centre (-7, +8)
px), picture top = left, tilt (1.2, -1.8) deg, twist 1.5 deg, video delay 58 ms (frame stamp minus mid-exposure of
row 0), readout 10 ms. The lens file is a copy (integ/justice_d/calib_loop_d_lens_<time>.json), so calib adds its new
profile there and never to justice_lens_profiles.json.

Run: ~/.venvs/vio/bin/python integ/justice_d/calib_loop_d.py
Writes integ/justice_d/calib_loop_d_<time>.txt, _settings_<time>.json (the calibrated settings) and the lens copy.
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), os.path.join(PROJ, "tests_apb")]
import apb_main            # noqa: E402
import camera as C         # noqa: E402
import capture             # noqa: E402
import fake_fc             # noqa: E402
import hold                # noqa: E402
import settings as S       # noqa: E402
import sitl                # noqa: E402

PORT, EXP, SECONDS = 14692, 0.004, 12.0
TRUE = dict(lens="sim Justice D near crop", top="left", mirror=False, tilt=(1.2, -1.8), twist=1.5, delay=0.058,
            readout=0.010)
STAMP = time.strftime("%Y%m%d_%H%M%S")


def qa(axis, ang):
    ang = np.asarray(ang, float)
    return np.concatenate([np.cos(ang / 2)[:, None], np.sin(ang / 2)[:, None] * np.asarray(axis, float)], 1)


def camera_case():
    """The sim camera of this test, as a px4sim case (also used for the closed-loop hold afterwards). video_delay_s:
    end of readout -> stamp, so that the stamp is TRUE delay after the row-0 mid-exposure (readout > 0)."""
    return dict(video="JUSTICE_D", true_lens=TRUE["lens"], cam_top=TRUE["top"], cam_mirror=TRUE["mirror"],
                cam_tilt=TRUE["tilt"], cam_twist=TRUE["twist"], readout_s=TRUE["readout"], exposure_s=EXP,
                video_delay_s=TRUE["delay"] - abs(TRUE["readout"]) - EXP / 2)


def start_settings(d, lens_copy):
    """Defaults + the justice_d overlay (as tools/use_profile.py apply justice_d writes it, starting values included)."""
    s = S.load("/nonexistent")
    ov = {k: v for k, v in json.load(open(sitl.JUSTICE_D_OVERLAY)).items() if not k.startswith("_")}
    s.update(ov)
    mask = os.path.join(d, "mask.png")
    w, h = s["video_size"][0] // s["video_reduce"], s["video_size"][1] // s["video_reduce"]
    cv2.imwrite(mask, np.full((h, w), 255, np.uint8))
    s.update(fc_device=f"udpin:127.0.0.1:{PORT}", log_dir=d, mask_file=mask, lens_file=lens_copy)
    return s


def main():
    lines = []
    say = lambda x: (lines.append(x), print(x, flush=True))   # noqa: E731
    pos_fn, att_fn, h_fn = fake_fc.scenario("hand")

    def pose(t):
        a, p = att_fn(t), pos_fn(t)
        q = C.qmul(qa(C.E_D, a[:, 2]), C.qmul(qa(C.E_E, a[:, 1]), qa(C.E_N, a[:, 0])))   # yaw, pitch, roll
        return p[:, 0], p[:, 1], h_fn(t), q

    case = camera_case()
    c = {**sitl.DEFAULTS, **sitl.video_defaults(case), **case, "vib_deg": 0.0, "seed": 5}
    cam = C.Camera(c, C.photo_ground("ottawa2019fall"))
    ro, fps = TRUE["readout"], cam.fps
    t0_off = EXP / 2 + (abs(ro) if ro < 0 else 0.0)
    lag = TRUE["delay"] + t0_off                     # frame start -> stamp; stamp - t0(row 0) = TRUE delay
    assert abs((abs(ro) + EXP + c["video_delay_s"]) - lag) < 1e-12      # the same chain as px4sim's Frames
    lens_copy = os.path.join(HERE, f"calib_loop_d_lens_{STAMP}.json")
    shutil.copy(C.LENS_JSON, lens_copy)
    n_before = len(json.load(open(lens_copy))["models"])
    say(f"truth: lens '{TRUE['lens']}' {json.dumps(C.TRUE_LENSES[TRUE['lens']])}")
    say(f"truth: top {TRUE['top']}, mirror {TRUE['mirror']}, tilt {list(TRUE['tilt'])}, twist {TRUE['twist']}, delay "
        f"{TRUE['delay'] * 1e3:.0f} ms, readout {TRUE['readout'] * 1e3:+.0f} ms; sim video_delay_s {c['video_delay_s'] * 1e3:.1f} ms")
    fails = 0
    with tempfile.TemporaryDirectory() as d:
        s = start_settings(d, lens_copy)
        say(f"start: lens '{s['lens_name']}', top {s['top']}, mirror {s['mirror']}, tilt {s['tilt_deg']}, twist "
            f"{s['twist_deg']}, delay {s['video_delay_s'] * 1e3:.0f} ms, readout {s['readout_s'] * 1e3:+.0f} ms, camera "
            f"{s['camera']} {s['video_size']}@{s['video_fps']} reduce {s['video_reduce']}")
        sp = os.path.join(d, "settings.json")
        S.save(s, sp)
        t_start = time.monotonic() + 1.0
        fake = subprocess.Popen([sys.executable, os.path.join(PROJ, "tests_apb", "fake_fc.py"), "--port", str(PORT),
                                 "--t-start", repr(t_start), "--log", os.path.join(d, "fc.jsonl"), "--scenario", "hand",
                                 "--seconds", "70"])
        st = {"k": -1, "late": []}

        def frame():                                 # real-time camera: newest frame, stamped when it is ready
            T = time.monotonic() - t_start
            k = max(st["k"] + 1, int(math.floor((T - lag) * fps)))
            st["k"] = k
            t_av = k / fps + lag
            t0 = k / fps + t0_off
            img = cam.finish(cam.render(t0, pose))   # rendered before its stamp time when the laptop keeps up
            while time.monotonic() < t_start + t_av:
                time.sleep(0.0005)
            st["late"].append(time.monotonic() - (t_start + t_av))
            return img, t_start + t_av
        src = capture.CallableSource(frame, s)
        src.judge = capture.judge(s)                 # the digital picture checks, as capture.DigitalSource uses
        try:
            fc = apb_main.open_fc(s)
            say(f"fake PX4: firmware {fc.firmware}, clock ready {fc.clock.ready()}, ATTITUDE {fc.rate('ATTITUDE', 1.0):.0f} Hz")
            rec = apb_main.record(src, fc, SECONDS, say=lambda x: None)
            late = np.array(st["late"])
            path = os.path.join(d, "rec.npz")
            np.savez(path, **rec)
            dt = np.diff(rec["t_grab"])
            say(f"recording: {len(rec['imgs'])} frames {rec['imgs'].shape[2]}x{rec['imgs'].shape[1]} in {SECONDS:g} s "
                f"(median frame gap {np.median(dt) * 1e3:.1f} ms), {len(rec['att'])} attitude rows, mode '{rec['mode']}'; "
                f"frames handed over after their stamp: median {np.median(late) * 1e3:.1f} ms, p95 {np.percentile(late, 95) * 1e3:.1f} ms")
            out = []
            t1 = time.time()
            rc = apb_main.mode_calib(S.load(sp), SimpleNamespace(rec=path, seconds=None, settings=sp), say=out.append)
            say(f"calib took {time.time() - t1:.0f} s, exit code {rc}")
            for x in out[1:]:
                say("  calib: " + x)
            w = json.load(open(sp))
            models = json.load(open(lens_copy))["models"]
            new = next((m for m in models if m["name"] == w["lens_name"]), None)
            keep = {k: w[k] for k in ("top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s",
                                      "video_delay_s", "lens_name", "calibrated", "calib_capture")}
            keep["lens_file"] = os.path.relpath(lens_copy, PROJ)
            json.dump(keep, open(os.path.join(HERE, f"calib_loop_d_settings_{STAMP}.json"), "w"), indent=1)
            tl = C.TRUE_LENSES[TRUE["lens"]]
            R_found = hold.make_R_bc(w["top"], w["mirror"], tuple(w["tilt_deg"]), w["twist_deg"])
            R_true = C.mount(TRUE["top"], TRUE["mirror"], TRUE["tilt"], TRUE["twist"])
            ang = math.degrees(np.linalg.norm(cv2.Rodrigues(R_found.T @ R_true)[0]))
            say("found against true (lens at 1280x720):")
            rows = [("fx", new and new["fx"], tl["fx"]), ("fy", new and new["fy"], tl["fy"]), ("k1", new and new["k1"], tl["k1"]),
                    ("k2 (not fitted)", new and new["k2"], tl["k2"]), ("cx (not fitted)", new and new["cx"], tl["cx"]),
                    ("cy (not fitted)", new and new["cy"], tl["cy"]),
                    ("video delay ms", w["video_delay_s"] * 1e3, TRUE["delay"] * 1e3),
                    ("readout ms", w["readout_s"] * 1e3, TRUE["readout"] * 1e3)]
            for name, got, want in rows:
                if got is None:
                    say(f"  {name:16s} not found")
                    continue
                rel = f" ({100 * (got - want) / want:+.2f} %)" if name in ("fx", "fy") else f" ({got - want:+.4f})"
                say(f"  {name:16s} found {got:9.4f}  true {want:9.4f}{rel}")
            say(f"  mount            found top {w['top']}, mirror {w['mirror']}, tilt {np.round(w['tilt_deg'], 2).tolist()}, "
                f"twist {w['twist_deg']:+.2f}; as one matrix {ang:.2f} deg from camera.py's")
            checks = [
                ("calib exit code 0 and calibrated = true", rc == 0 and w["calibrated"], f"rc {rc}, calibrated {w['calibrated']}"),
                ("a new lens profile in the lens copy, selected", new is not None and len(models) == n_before + 1,
                 f"{w['lens_name']}, {len(models)} profiles (before {n_before})"),
                ("picture top = left, not mirrored", w["top"] == TRUE["top"] and w["mirror"] is TRUE["mirror"], f"{w['top']} {w['mirror']}"),
                ("fx within 1 %", new is not None and abs(new["fx"] / tl["fx"] - 1) < 0.01, new and f"{new['fx']:.1f}"),
                ("fy within 1 %", new is not None and abs(new["fy"] / tl["fy"] - 1) < 0.01, new and f"{new['fy']:.1f}"),
                ("k1 within 0.01", new is not None and abs(new["k1"] - tl["k1"]) < 0.01, new and f"{new['k1']:+.4f}"),
                ("video delay within 5 ms", abs(w["video_delay_s"] - TRUE["delay"]) < 0.005, f"{w['video_delay_s'] * 1e3:.1f} ms"),
                ("readout within 4.5 ms", abs(w["readout_s"] - TRUE["readout"]) < 0.0045, f"{w['readout_s'] * 1e3:+.1f} ms"),
                ("mount within 0.5 deg", ang < 0.5, f"{ang:.2f} deg"),
            ]
            for name, ok, got in checks:
                fails += not ok
                say(f"{'PASS' if ok else 'FAIL'} {name}: {got}")
            fr = src.newest()
            probs = apb_main.start_problems(S.load(sp), fc, src, fr.img.shape)
            fails += bool(probs)
            say(f"{'PASS' if not probs else 'FAIL'} with the calibrated settings run has no start problem: {probs or 'none'}")
            fc.close()
        finally:
            fake.terminate()
            fake.wait()
    say(f"{'ALL PASS' if not fails else f'{fails} FAIL'}")
    open(os.path.join(HERE, f"calib_loop_d_{STAMP}.txt"), "w").write("\n".join(lines) + "\n")
    return fails


if __name__ == "__main__":
    sys.exit(main())
