#!/usr/bin/env python3
"""The measured lens is calibration data. It lives in settings.json under lens_measured, as the mount and the delay do.
(a) calib, then tools/use_profile.py apply justice_d ...: the measured lens stays in use (D2).
(b) calib on the board, then tools/deploy.sh pull and push with BOARD=local: the measured lens stays in use, and
    run builds the hold (D1). Push stops when only the board's lens_measured differs from the laptop's.
(c) is in test_lens_d.py: it needs a real fit.
(d) an old settings file with no lens_measured loads and gives the same lens as before.
(e) run and check refuse a measured lens of another picture shape, with a message that says what to do.
(f) deploy.sh pull copies the logs from the board's log_dir, with rsync and with tar (D5). Push stops when a fresh
    run log in log_dir shows that the program sends positions.
calib here is apb_main.mode_calib with calib.solve replaced by a fixed result (FIT). This file tests where the
result goes, not the fit: test_lens_d.py tests the fit on synthetic pictures.
Run: ~/.venvs/vio/bin/python tests_apb/test_lens_store.py      (about 20 s)
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "tools"))
import apb_main            # noqa: E402
import hold                # noqa: E402
import settings as S       # noqa: E402

PY = sys.executable
LENS = os.path.join(PROJ, "justice_lens_profiles.json")
START = "Justice D full sensor (assumed)"
# The fixed calib result: a fit_lens result at 640x360 (centre and k2-k4 are the start profile's).
FIT = dict(ok=True, top="right", mirror=False, tilt_deg=[1.5, -1.0], twist_deg=0.8, focal_scale=1.0, video_delay_s=0.046,
           readout_s=0.012, resid_deg=0.094, runner_up=dict(top="left", mirror=True, resid_deg=3.1), ratio=33.0,
           rate_dps=45.0, rate_axes_dps=[30.0, 25.0, 20.0], n_matches=25000, seconds=5.0, k1_fitted=True, cover_deg=66.0,
           corner_deg=75.0, cam_rate_dps=[33.0, 22.0, 26.0],
           lens=dict(fx=301.0, fy=289.6, k1=-0.0368, cx=319.5, cy=179.5, k2=0.0, k3=0.0, k4=0.0, w=640, h=360))
WANT = [301.0, 289.6, 319.5, 179.5, -0.0368]          # fx fy cx cy k1 that run must use at 640x360
MODE = "digital 1280x720@60 reduce=2 640x360"
# Program files that the laptop copy needs (tools/deploy.sh FILES, and the two tools).
FILES = ["apb_main.py", "hold.py", "fc_link.py", "capture.py", "calib.py", "settings.py", "justice_lens_profiles.json",
         "EVENT_DAY.txt", "DECISION_CARD.txt", "tools/board_check.sh", "tools/fc_params.py", "tools/deploy.sh",
         "tools/use_profile.py"]
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


# Runs in a program folder (cwd), so that folder's own apb_main, calib and settings are used.
CALIB = r'''
import json, sys
from types import SimpleNamespace
import apb_main, calib, settings as S
fit = json.loads(sys.argv[2])
calib.solve = lambda rec, s, mask=None, fit_lens=False: dict(fit)
sys.exit(apb_main.mode_calib(S.load(sys.argv[1]), SimpleNamespace(rec=sys.argv[3], seconds=None, settings=sys.argv[1])))
'''
PROBE = r'''
import json, sys
from types import SimpleNamespace
import apb_main, settings as S
s = S.load(sys.argv[1])
shape = (int(sys.argv[3]), int(sys.argv[2]))
try:
    K, D = apb_main.lens(s, shape)
    k = [round(float(x), 6) for x in (K[0, 0], K[1, 1], K[0, 2], K[1, 2], D[0])]
except Exception as e:
    k = repr(e)
p = apb_main.start_problems(s, SimpleNamespace(firmware="px4"), SimpleNamespace(info={"width": 1280, "height": 720}), shape)
print(json.dumps(dict(k=k, problems=p, lens_name=s["lens_name"], calibrated=s["calibrated"])))
'''


def calib_in(folder, sp, rec):
    r = subprocess.run([PY, "-c", CALIB, sp, json.dumps(FIT), rec], cwd=folder, capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr).strip().splitlines()


def probe(folder, sp, w=640, h=360):
    """The lens that run uses (fx, fy, cx, cy, k1) or the error, and start_problems, in that folder's program."""
    r = subprocess.run([PY, "-c", PROBE, sp, str(w), str(h)], cwd=folder, capture_output=True, text=True)
    try:
        return json.loads(r.stdout.strip().splitlines()[-1])
    except (IndexError, ValueError):
        return dict(k=f"probe failed: {r.stderr.strip()[-300:]}", problems=["probe failed"], lens_name=None, calibrated=None)


def in_use(p):
    return isinstance(p["k"], list) and np.allclose(p["k"], WANT, atol=1e-6)


def built(p):
    return not any(x.startswith("cannot build the hold") for x in p["problems"])


def use_profile(folder, sp, *names):
    r = subprocess.run([PY, os.path.join(folder, "tools", "use_profile.py"), "apply", *names, "--settings", sp],
                       capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr).strip()


def recording(path):
    """A tiny recording: calib.solve is replaced, so only the shape and the mode matter."""
    np.savez(path, imgs=np.zeros((10, 360, 640), np.uint8), t_grab=np.arange(10) / 60.0,
             att=np.zeros((5, 4)), K=np.eye(3), D=np.zeros(4), mode=np.array(MODE))


def deploy(laptop, board, *args, **env):
    e = dict(os.environ, BOARD="local", BOARD_DIR=board, NO_RESTART="1", WAIT="0", **env)
    r = subprocess.run(["sh", os.path.join(laptop, "tools", "deploy.sh"), *args], env=e, capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr).strip()


def edit(sp, **kw):
    s = S.load(sp)
    s.update(kw)
    S.save(s, sp)


# ---------------------------------------------------------------- (a) profile apply after calib
def apply_part(d):
    print("(a) calib, then use_profile.py apply justice_d outdoor_night goal_a ardupilot")
    lens, sp, rec = os.path.join(d, "lens.json"), os.path.join(d, "settings.json"), os.path.join(d, "rec.npz")
    shutil.copy(LENS, lens)
    recording(rec)
    rc, out = use_profile(PROJ, sp, "justice_d", "outdoor_day", "goal_a", "ardupilot")
    edit(sp, lens_file=lens, log_dir=d)
    before = open(lens).read()
    rc, lines = calib_in(PROJ, sp, rec)
    p = probe(PROJ, sp)
    check(rc == 0 and in_use(p) and open(lens).read() == before,
          f"after calib: rc {rc}, lens in use {p['k']}, lens file unchanged {open(lens).read() == before}, lens_name "
          f"'{p['lens_name']}'" + ("" if rc == 0 else f" | {' | '.join(lines[-3:])}"))
    rc, out = use_profile(PROJ, sp, "justice_d", "outdoor_night", "goal_a", "ardupilot")
    p = probe(PROJ, sp)
    check(rc == 0 and in_use(p) and p["calibrated"] and p["lens_name"] == START,
          f"after apply: rc {rc}, lens in use {p['k']} (want {WANT}), calibrated {p['calibrated']}, lens_name '{p['lens_name']}'"
          + ("" if rc == 0 else f" | {out[-300:]}"))
    import use_profile as U
    try:
        U.check_keys({"lens_measured": None}, "overlay", overlay=True)
        refused = ""
    except ValueError as e:
        refused = str(e)
    check("lens_measured" in U.CALIB_KEYS and "calib" in refused, f"an overlay may not set lens_measured: '{refused}'")
    return sp


# ---------------------------------------------------------------- (b), (f) deploy
def deploy_part(d):
    print("(b) calib on the board, then deploy.sh pull and push (BOARD=local); (f) logs from the board's log_dir")
    laptop, board, sd = os.path.join(d, "laptop"), os.path.join(d, "board"), os.path.join(d, "sdcard", "logs")
    for f in FILES:
        os.makedirs(os.path.dirname(os.path.join(laptop, f)), exist_ok=True)
        shutil.copy(os.path.join(PROJ, f), os.path.join(laptop, f))
    shutil.copytree(os.path.join(PROJ, "tools", "profiles"), os.path.join(laptop, "tools", "profiles"))
    os.makedirs(sd)
    lsp, bsp, rec = os.path.join(laptop, "settings.json"), os.path.join(board, "settings.json"), os.path.join(d, "rec.npz")
    recording(rec)
    rc, out = use_profile(laptop, lsp, "justice_d", "outdoor_day", "goal_a", "ardupilot")
    edit(lsp, log_dir=sd)                      # the digital camera's logs on the SD card (here a temporary folder)
    rc, out = deploy(laptop, board)
    check(rc == 0 and os.path.exists(bsp), f"first push: rc {rc}" + ("" if rc == 0 else f" | {out[-300:]}"))
    rc, lines = calib_in(board, bsp, rec)
    p = probe(board, bsp)
    check(rc == 0 and in_use(p), f"calib on the board: rc {rc}, lens in use {p['k']}"
                                 + ("" if rc == 0 else f" | {' | '.join(lines[-3:])}"))
    with open(os.path.join(sd, "run_20261119_101500.csv"), "w") as f:
        f.write("t,a,b,c,d,e,sent\n1,0,0,0,0,0,0\n")
    os.makedirs(os.path.join(board, "logs"), exist_ok=True)
    with open(os.path.join(board, "logs", "console.txt"), "w") as f:
        f.write("console\n")
    rc, out = deploy(laptop, board, "pull")
    got = sorted(os.listdir(os.path.join(laptop, "logs"))) if os.path.isdir(os.path.join(laptop, "logs")) else []
    check(rc == 0 and "run_20261119_101500.csv" in got and "console.txt" in got,
          f"(f) pull with rsync: rc {rc}, laptop logs {got}" + ("" if rc == 0 else f" | {out[-300:]}"))
    rc, out = deploy(laptop, board)
    p = probe(board, bsp)
    check(rc == 0 and in_use(p) and built(p), f"pull, then push: rc {rc}, lens in use on the board {p['k']} (want {WANT}), "
                                              f"start problems {p['problems']}" + ("" if rc == 0 else f" | {out[-300:]}"))
    with open(os.path.join(sd, "run_20261119_102000.csv"), "w") as f:
        f.write("t,a,b,c,d,e,sent\n1,0,0,0,0,0,0\n")
    rc, out = deploy(laptop, board, "pull", NO_RSYNC="1")
    got = sorted(os.listdir(os.path.join(laptop, "logs"))) if os.path.isdir(os.path.join(laptop, "logs")) else []
    check(rc == 0 and "run_20261119_102000.csv" in got, f"(f) pull with tar: rc {rc}, laptop logs {got}"
                                                        + ("" if rc == 0 else f" | {out[-300:]}"))
    s = S.load(bsp)
    if s.get("lens_measured"):
        s["lens_measured"] = dict(s["lens_measured"], fx=s["lens_measured"]["fx"] + 1.0)
        S.save(s, bsp)
        rc, out = deploy(laptop, board)
    else:
        rc, out = 0, "the board has no lens_measured"
    check(rc != 0 and "lens_measured" in out, f"board lens_measured differs from the laptop's: push stops: rc {rc} | "
                                              f"{out.splitlines()[-1] if out else ''}")
    rc, out = deploy(laptop, board, "pull")
    with open(os.path.join(sd, "run_20261119_103000.csv"), "w") as f:
        f.write("t,a,b,c,d,e,sent\n" + "".join(f"{i},0,0,0,0,0,1\n" for i in range(30)))
    rc, out = deploy(laptop, board)
    check(rc != 0 and "sending positions" in out, f"(f) a fresh run log in log_dir that sends: push stops: rc {rc} | "
                                                  f"{out.splitlines()[-1] if out else ''}")


# ---------------------------------------------------------------- (d) old settings file
def old_part(d):
    print("(d) an old settings file with no lens_measured")
    lens = os.path.join(d, "lens_old.json")
    old = json.load(open(LENS))
    old["models"].append(dict(next(m for m in old["models"] if m["name"] == START), name="Justice D calibrated 2026-11-18 23:43",
                              fx=602.0, fy=579.2, k1=-0.0368))
    json.dump(old, open(lens, "w"))
    for name, kw, shape, field in (("analog", dict(focal_scale=1.013), (288, 360), 0),
                                   ("digital, old calibrated profile", dict(camera="digital", lens_file=lens, calibrated=True,
                                    lens_name="Justice D calibrated 2026-11-18 23:43"), (360, 640), None)):
        sp = os.path.join(d, "old.json")
        s = {k: v for k, v in S.load("/nonexistent").items() if k != "lens_measured"}
        s.update(kw)
        with open(sp, "w") as f:
            json.dump(s, f)
        try:
            s2 = S.load(sp)
            K, D = apb_main.lens(s2, shape)
            Kw, Dw = hold.load_lens(S.here(s2["lens_file"]), s2["lens_name"], shape[1], shape[0], field=field)
            Kw[0, 0] *= s2["focal_scale"]
            Kw[1, 1] *= s2["focal_scale"]
            ok, txt = s2["lens_measured"] is None and np.allclose(K, Kw) and np.allclose(D, Dw), f"fx {K[0, 0]:.2f} (profile {Kw[0, 0]:.2f})"
        except Exception as e:
            ok, txt = False, repr(e)
        check(ok, f"{name}: loads, lens_measured null, lens = profile x focal_scale as before: {txt}")
    bad = os.path.join(d, "bad.json")
    with open(bad, "w") as f:
        json.dump({"lens_measured": {"by": "target"}}, f)
    try:
        S.load(bad)
        why = ""
    except ValueError as e:
        why = str(e)
    check("lens_measured" in why and "unknown key" not in why, f"a lens_measured without fx, fy ... is refused: '{why}'")


# ---------------------------------------------------------------- (e) another picture shape
def shape_part(d, sp):
    print("(e) run and check with a measured lens of another picture shape")
    real = apb_main.open_fc, apb_main.open_video

    def none(*a, **k):
        raise RuntimeError("no device in this test")

    def check_lines(s):
        apb_main.open_fc, apb_main.open_video = none, none
        lines = []
        try:
            apb_main.mode_check(s, SimpleNamespace(set_params=False), say=lines.append)
        except Exception as e:
            lines.append(f"mode_check raised {e!r}")
        finally:
            apb_main.open_fc, apb_main.open_video = real
        return lines

    lines = check_lines(S.load(sp))
    m = [x for x in lines if "lens_measured" in x]
    check(len(m) == 1 and m[0].startswith("PASS"), f"digital camera, 640x360 measured lens: check says {m}")
    asp = os.path.join(d, "analog.json")
    shutil.copy(sp, asp)
    rc, out = use_profile(PROJ, asp, "outdoor_day", "goal_a", "ardupilot")
    p = probe(PROJ, asp, 360, 288)
    why = [x for x in p["problems"] if "lens_measured" in x]
    check(rc == 0 and len(why) == 1 and "null" in why[0], f"analog camera 360x288: run refuses: {why or p['problems']}")
    lines = check_lines(S.load(asp))
    m = [x for x in lines if "lens_measured" in x]
    check(len(m) == 1 and m[0].startswith("FAIL") and "null" in m[0] and lines[-1].endswith("FAIL"),
          f"analog camera 360x288: check says {m}, last line '{lines[-1]}'")


def main():
    t0 = time.time()
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(os.path.join(d, "a"))
        sp = apply_part(os.path.join(d, "a"))
        deploy_part(os.path.join(d, "b"))
        old_part(d)
        shape_part(d, sp)
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
