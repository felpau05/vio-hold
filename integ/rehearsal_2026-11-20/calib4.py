#!/usr/bin/env python3
"""Audit item B: the hand-turn calibration (apb_main calib --rec) from each start lens, on synthetic pictures of
each true lens. Also the no-SD profile's record_hz 10 (item A.3).

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/calib4.py TRUE_LENS START_LENS [RECORD_HZ] [SEED]

The recording is made as integ/justice_d/calib_loop_d.py makes it (its camera case, pose and fake FC; scenario
"hand", 12 s): px4sim/camera.py renders the Justice D in real time with the true lens (a profile name of
justice_lens_profiles.json), apb_main.record stamps and keeps record_hz frames each second. Then apb_main.mode_calib
fits from the start lens (settings lens_name). Prints the calib lines, the fitted lens against the true lens
(focal error of fx and fy), the residual, and the delay and readout against the truth.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), os.path.join(PROJ, "tests_apb"), os.path.join(PROJ, "integ", "justice_d")]
import apb_main            # noqa: E402
import calib_loop_d as L   # noqa: E402
import camera as C         # noqa: E402
import capture             # noqa: E402
import fake_fc             # noqa: E402
import settings as S       # noqa: E402
import sitl                # noqa: E402

PORT = 14697


def main():
    true_lens, start_lens = sys.argv[1], sys.argv[2]
    rhz = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 5
    L.TRUE["lens"] = true_lens
    pos_fn, att_fn, h_fn = fake_fc.scenario("hand")

    def pose(t):
        a, p = att_fn(t), pos_fn(t)
        q = C.qmul(L.qa(C.E_D, a[:, 2]), C.qmul(L.qa(C.E_E, a[:, 1]), L.qa(C.E_N, a[:, 0])))
        return p[:, 0], p[:, 1], h_fn(t), q

    case = L.camera_case()
    c = {**sitl.DEFAULTS, **sitl.video_defaults(case), **case, "vib_deg": 0.0, "seed": seed}
    cam = C.Camera(c, C.photo_ground("ottawa2019fall"))
    ro, fps = L.TRUE["readout"], cam.fps
    t0_off = L.EXP / 2 + (abs(ro) if ro < 0 else 0.0)
    lag = L.TRUE["delay"] + t0_off
    print(f"true lens '{true_lens}', start lens '{start_lens}', record_hz {rhz:g}, seed {seed}, load "
          f"{' '.join(f'{x:.2f}' for x in os.getloadavg())}", flush=True)
    with tempfile.TemporaryDirectory() as d:
        s = L.start_settings(d, C.LENS_JSON)
        s.update(fc_device=f"udpin:127.0.0.1:{PORT}", lens_name=start_lens, record_hz=rhz)
        sp = os.path.join(d, "settings.json")
        S.save(s, sp)
        t_start = time.monotonic() + 1.0
        fake = subprocess.Popen([sys.executable, os.path.join(PROJ, "tests_apb", "fake_fc.py"), "--port", str(PORT),
                                 "--t-start", repr(t_start), "--log", os.path.join(d, "fc.jsonl"), "--scenario", "hand",
                                 "--seconds", "70"])
        st = {"k": -1, "late": []}

        def frame():
            k = max(st["k"] + 1, int(math.floor((time.monotonic() - t_start - lag) * fps)))
            st["k"] = k
            t_av, t0 = k / fps + lag, k / fps + t0_off
            img = cam.finish(cam.render(t0, pose))
            while time.monotonic() < t_start + t_av:
                time.sleep(0.0005)
            st["late"].append(time.monotonic() - (t_start + t_av))
            return img, t_start + t_av
        src = capture.CallableSource(frame, s)
        try:
            fc = apb_main.open_fc(s)
            rec = apb_main.record(src, fc, L.SECONDS, say=lambda x: None)
            fc.close()
        finally:
            fake.terminate()
            fake.wait()
        late = np.array(st["late"])
        tag = f"{true_lens.split()[2]}_{start_lens.split()[2]}_{rhz:g}hz_s{seed}"
        path = os.path.join(HERE, "runs", f"calib4_{tag}_rec.npz")      # kept: EVENT_DAY step 4 "calib --rec" uses it
        np.savez(path, **rec)
        print(f"recording: {len(rec['imgs'])} frames in {L.SECONDS:g} s; frames late after their stamp p95 "
              f"{np.percentile(late, 95) * 1e3:.1f} ms", flush=True)
        out = []
        rc = apb_main.mode_calib(S.load(sp), SimpleNamespace(rec=path, seconds=None, settings=sp), say=out.append)
        for x in out[1:]:
            print("  calib: " + x)
        w = S.load(sp)
        tl = C.profile(true_lens)
        m = w["lens_measured"]
        resid = next((float(x.split("residual ")[1].split(" deg")[0]) for x in out if "INFO quality" in x), None)
        if rc == 0 and m:
            k = tl["image_size"][0] / m["image_size"][0]       # calib writes the lens at the recording's size (640x360)
            m = dict(m, fx=m["fx"] * k, fy=m["fy"] * k)
            print(f"RESULT true '{true_lens}' start '{start_lens}' rhz {rhz:g}: exit {rc}, fx {m['fx']:.1f} (true "
                  f"{tl['fx']:.1f}, {100 * (m['fx'] / tl['fx'] - 1):+.2f} %), fy {m['fy']:.1f} (true {tl['fy']:.1f}, "
                  f"{100 * (m['fy'] / tl['fy'] - 1):+.2f} %), k1 {m['k1']:+.4f} (true {tl.get('k1', 0):+.4f}), residual "
                  f"{resid} deg, delay {w['video_delay_s'] * 1e3:.1f} ms (true {L.TRUE['delay'] * 1e3:.0f}), readout "
                  f"{w['readout_s'] * 1e3:+.1f} ms (true {L.TRUE['readout'] * 1e3:+.0f}), top {w['top']} (true {L.TRUE['top']})")
        else:
            print(f"RESULT true '{true_lens}' start '{start_lens}' rhz {rhz:g}: exit {rc}, residual {resid}, "
                  f"lens_measured {'set' if m else 'none'}: {[x for x in out if 'FAIL' in x]}")
        json.dump(dict(rc=rc, lens=m, resid=resid, delay=w["video_delay_s"], readout=w["readout_s"], out=out),
                  open(os.path.join(HERE, f"calib4_{tag}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
