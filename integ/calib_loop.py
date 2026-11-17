"""Calibration in the loop (integrator scope item 3).

A hand-rotation recording is made through the onboard program's own code: fc_link reads a fake PX4
(tests_apb/fake_fc.py, scenario "hand": 80 ms clock offset, 0-20 ms jitter), the capture source gives
pictures rendered in real time by px4sim/camera.py, and apb_main.record stamps and stores them. The
camera geometry comes from camera.py, which shares no code with hold.make_R_bc, so a convention error
in hold.py or calib.py cannot cancel out. The true camera is set up WRONG against the default settings:
picture top = rear, mirrored, mount tilt (1.0, -1.5) deg, twist 1.0 deg, video delay 70 ms (frame stamp
minus mid-exposure of row 0), readout -18 ms (bottom row first), focal scale 0.95.
Then apb_main calib (mode_calib --rec) must find all of it, and run must refuse to start without the
calibrated flag.

Run: ~/.venvs/vio/bin/python integ/calib_loop.py   (about 1 minute; writes integ/calib_loop.txt)
"""
import contextlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import cv2
import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), os.path.join(PROJ, "tests_apb")]
import apb_main            # noqa: E402
import camera as C         # noqa: E402
import capture             # noqa: E402
import fake_fc             # noqa: E402
import hold                # noqa: E402
import settings as S       # noqa: E402
import sitl                # noqa: E402

PORT, FPS, EXP = 14690, 25.0, 0.004
TRUE = dict(top="rear", mirror=True, tilt=(1.0, -1.5), twist=1.0, delay=0.070, readout=-0.018, focal=0.95)
OUT = os.path.join(PROJ, "integ", "calib_loop.txt")


def qa(axis, ang):
    ang = np.asarray(ang, float)
    return np.concatenate([np.cos(ang / 2)[:, None], np.sin(ang / 2)[:, None] * np.asarray(axis, float)], 1)


def main():
    lines = []
    say = lambda x: (lines.append(x), print(x, flush=True))   # noqa: E731
    pos_fn, att_fn, h_fn = fake_fc.scenario("hand")

    def pose(t):
        a, p = att_fn(t), pos_fn(t)
        q = C.qmul(qa(C.E_D, a[:, 2]), C.qmul(qa(C.E_E, a[:, 1]), qa(C.E_N, a[:, 0])))   # yaw, pitch, roll
        return p[:, 0], p[:, 1], h_fn(t), q

    lens = S.DEFAULTS["lens_name"]       # the true lens = the profile; only its focal length is off
    c = dict(sitl.DEFAULTS, true_lens=lens, focal_scale=TRUE["focal"], cam_top=TRUE["top"], cam_mirror=TRUE["mirror"],
             cam_tilt=TRUE["tilt"], cam_twist=TRUE["twist"], readout_s=TRUE["readout"], exposure_s=EXP, vib_deg=0.0, seed=5)
    cam = C.Camera(c, C.photo_ground("ottawa2019fall"))
    ro = TRUE["readout"]
    chain = TRUE["delay"] - EXP / 2                  # readout end -> frame stamp, so stamp - t0(row 0) = 70 ms
    lag = abs(ro) + EXP + chain
    fails = 0
    with tempfile.TemporaryDirectory() as d:
        mask = os.path.join(d, "mask.png")
        cv2.imwrite(mask, np.full((cam.H, cam.W), 255, np.uint8))
        s = S.load("/nonexistent")                   # defaults: top front, not mirrored, not calibrated
        s.update(fc_device=f"udpin:127.0.0.1:{PORT}", log_dir=d, mask_file=mask)
        sp = os.path.join(d, "settings.json")
        S.save(s, sp)
        t_start = time.monotonic() + 1.0
        fake = subprocess.Popen([sys.executable, os.path.join(PROJ, "tests_apb", "fake_fc.py"), "--port", str(PORT),
                                 "--t-start", repr(t_start), "--log", os.path.join(d, "fc.jsonl"), "--scenario", "hand",
                                 "--seconds", "70"])
        st = {"k": -1}

        def frame():                                 # real-time camera: newest frame, stamped when it is ready
            T = time.monotonic() - t_start
            k = max(st["k"] + 1, int(math.floor((T - lag) * FPS)))
            st["k"] = k
            t_av = k / FPS + lag
            while time.monotonic() < t_start + t_av:
                time.sleep(0.001)
            t0 = k / FPS + EXP / 2 + (abs(ro) if ro < 0 else 0.0)
            return cam.finish(cam.render(t0, pose)), t_start + t_av
        src = capture.CallableSource(frame, s)
        try:
            fc = apb_main.open_fc(s)
            say(f"fake PX4: firmware {fc.firmware}, clock ready {fc.clock.ready()}, ATTITUDE {fc.rate('ATTITUDE', 1.0):.0f} Hz")
            rec = apb_main.record(src, fc, 12.0, say=lambda x: None)
            path = os.path.join(d, "rec.npz")
            np.savez(path, **rec)
            say(f"recording: {len(rec['imgs'])} frames, {len(rec['att'])} attitude rows, mode '{rec['mode']}'")

            out = []
            rc = apb_main.mode_calib(S.load(sp), SimpleNamespace(rec=path, seconds=None, settings=sp), say=out.append)
            for x in out[1:]:
                say("  calib: " + x)
            w = json.load(open(sp))
            keep = {k: w[k] for k in ("top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s", "video_delay_s")}
            json.dump(keep, open(os.path.join(PROJ, "integ", "calib_loop_settings.json"), "w"), indent=1)
            R_found = hold.make_R_bc(w["top"], w["mirror"], tuple(w["tilt_deg"]), w["twist_deg"])
            R_true = C.mount(TRUE["top"], TRUE["mirror"], TRUE["tilt"], TRUE["twist"])
            ang = math.degrees(np.linalg.norm(cv2.Rodrigues(R_found.T @ R_true)[0]))
            checks = [
                ("calib exit code 0 and calibrated = true", rc == 0 and w["calibrated"], f"rc {rc}, calibrated {w['calibrated']}"),
                ("picture top = rear", w["top"] == TRUE["top"], w["top"]),
                ("mirrored", w["mirror"] is TRUE["mirror"], w["mirror"]),
                ("video delay 70 ms (within 5 ms)", abs(w["video_delay_s"] - TRUE["delay"]) < 0.005,
                 f"{w['video_delay_s'] * 1e3:.1f} ms"),
                ("readout -18 ms (within 4.5 ms, sign right)", abs(w["readout_s"] - ro) < 0.0045,
                 f"{w['readout_s'] * 1e3:+.1f} ms"),
                ("focal scale 0.95 (within 0.01)", abs(w["focal_scale"] - TRUE["focal"]) < 0.01, f"{w['focal_scale']:.4f}"),
                ("mount (top, mirror, tilt, twist) as one matrix within 0.5 deg of camera.py's", ang < 0.5,
                 f"{ang:.2f} deg; found tilt {np.round(w['tilt_deg'], 2).tolist()} twist {w['twist_deg']:+.2f} "
                 f"(camera.py: tilt {list(TRUE['tilt'])} twist {TRUE['twist']:+.1f} as a physical turn)"),
            ]
            for name, ok, got in checks:
                fails += not ok
                say(f"{'PASS' if ok else 'FAIL'} {name}: {got}")

            # run must refuse to start without the calibrated flag
            s_un = dict(S.load(sp), calibrated=False)
            sp_un = os.path.join(d, "settings_uncal.json")
            S.save(s_un, sp_un)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc_run = apb_main.main(["run", "--settings", sp_un])
            ok = rc_run == 1 and "not calibrated" in buf.getvalue()
            fails += not ok
            say(f"{'PASS' if ok else 'FAIL'} 'apb_main.py run' without the flag: exit {rc_run}, says: {buf.getvalue().strip()}")
            try:
                apb_main.run_live(s_un, src, fc, seconds=3.0, log_path=os.path.join(d, "run.csv"))
                ok, why = False, "it started"
            except RuntimeError as e:
                ok, why = "not calibrated" in str(e), str(e)
            fails += not ok
            say(f"{'PASS' if ok else 'FAIL'} run_live without the flag (same source and FC link): {why}")
            fr = src.newest()
            probs = apb_main.start_problems(S.load(sp), fc, src, fr.img.shape)
            ok = not probs
            fails += not ok
            say(f"{'PASS' if ok else 'FAIL'} with the calibrated settings run has no start problem: {probs or 'none'}")
            fc.close()
        finally:
            fake.terminate()
            fake.wait()
    say(f"{'ALL PASS' if not fails else f'{fails} FAIL'}")
    open(OUT, "w").write("\n".join(lines) + "\n")
    return fails


if __name__ == "__main__":
    sys.exit(main())
